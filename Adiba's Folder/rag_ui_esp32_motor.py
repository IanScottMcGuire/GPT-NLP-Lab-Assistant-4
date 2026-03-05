# rag_ui_esp32.py
#
# Motor-control integration changes (marked with # [MOTOR]):
#   - ESP32Controller class replaces uart_client.send_component_request
#   - Mirrors the ESP32 state machine: BOOT→HOMING→IDLE→GATE_LOCKOUT→INVENTORY_PENDING
#   - Emergency stop button always visible in sidebar
#   - Real-time motor status banner on chatbot page
#   - Admin dashboard stock uses live HI/LO readings from last inventory run
#   - LLM classification extended to all 4 bins (resistors, capacitors, LEDs)
#   - Dispense workflow drives the full sequence: move→gate wait→inventory→result

import os
import io
import json
import re
import tempfile
import threading
import time
import queue
import wave
import subprocess
import sys
import webbrowser
import pickle

import streamlit as st
import requests
import numpy as np
from sentence_transformers import SentenceTransformer, util
import faiss
from gtts import gTTS
from pytector import PromptInjectionDetector
from datetime import datetime
import sounddevice as sd
import whisper

# ─── [MOTOR] Serial import with graceful fallback ────────────────────────────
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

from config import STT_ENGINE, TTS_ENGINE

# ─── Configuration ────────────────────────────────────────────────────────────
API_KEY  = os.getenv("TAMUS_AI_CHAT_API_KEY")
API_URL  = os.getenv("TAMUS_AI_CHAT_API_ENDPOINT")

INDEX_PATH        = "index/vector_index.pkl"
MODEL_NAME        = "all-MiniLM-L6-v2"
FAQS_PATH         = "faqs.json"
INAPPROPRIATE_LOG = "inappropriate_queries.txt"
WHISPER_MODEL     = "base"
WHISPER_DEVICE    = "cuda"

# [MOTOR] Serial port used for TX/RX to ESP32 on Jetson Orin Nano
#   ESP32 TX  → Jetson pin 8  (/dev/ttyTHS1 RX)
#   ESP32 RX  → Jetson pin 10 (/dev/ttyTHS1 TX)
ESP32_PORT = "/dev/ttyTHS1"
ESP32_BAUD = 115200

# [MOTOR] Bin mapping – must stay in sync with ESP32 firmware and PDF spec
#   BIN 0 → 1 kΩ resistor
#   BIN 1 → 2.2 kΩ resistor
#   BIN 2 → 470 nF capacitor
#   BIN 3 → LED lights
COMPONENT_TO_BIN = {
    "1kohm":  0,
    "2k2ohm": 1,
    "470nf":  2,
    "led":    3,
}

# [MOTOR] Inventory depth threshold (matches ESP32 firmware constant)
INVENTORY_THRESHOLD_CM = 12.0


# ─── [MOTOR] ESP32 Controller ────────────────────────────────────────────────
class ESP32Controller:
    """
    Mirrors the ESP32 state machine over UART (newline-terminated strings).

    States:
      DISCONNECTED  – serial not open
      BOOT          – connected, homing not yet done
      HOMING        – 'h' sent, waiting for result
      IDLE          – homed and ready for bin commands
      GATE_LOCKOUT  – bin selected; gate blocked or within 2s re-arm window
      INVENTORY_PENDING – gate re-armed; waiting to send 'i'
      ESTOP         – 'e' sent; requires physical ESP32 reset to recover

    A background reader thread feeds every incoming line into _line_q so
    the Streamlit main thread never blocks on Serial.readline().
    """

    def __init__(self):
        self.state        = "DISCONNECTED"
        self.current_bin  = -1
        self.last_dist_cm = None      # most recent ultrasonic reading (cm)
        self.last_stock   = {}        # {bin_num: "HI" | "LO"}
        self._ser         = None
        self._line_q      = queue.Queue()
        self._reader_thread = None

    # ── Connection ────────────────────────────────────────────────
    def connect(self, port=ESP32_PORT, baud=ESP32_BAUD) -> bool:
        if not SERIAL_AVAILABLE:
            return False
        try:
            self._ser  = serial.Serial(port, baud, timeout=1)
            self.state = "BOOT"
            self._start_reader()
            return True
        except Exception as e:
            print(f"[ESP32] Serial connect failed: {e}")
            self.state = "DISCONNECTED"
            return False

    def _start_reader(self):
        def _read():
            while self._ser and self._ser.is_open:
                try:
                    raw = self._ser.readline()
                    if raw:
                        line = raw.decode("utf-8", errors="ignore").strip()
                        if line:
                            self._line_q.put(line)
                            print(f"[ESP32] {line}")
                except Exception:
                    break
        self._reader_thread = threading.Thread(target=_read, daemon=True)
        self._reader_thread.start()

    # ── Send ──────────────────────────────────────────────────────
    def send(self, cmd: str):
        """Send a newline-terminated command string to ESP32."""
        if self._ser and self._ser.is_open:
            self._ser.write((cmd.strip() + "\n").encode())

    # ── Read ──────────────────────────────────────────────────────
    def _read_line(self, timeout=10.0):
        try:
            return self._line_q.get(timeout=timeout)
        except queue.Empty:
            return None

    def _drain_and_watch(self, timeout=30.0, stop_on=None, fail_on=None):
        """
        Consume lines until a stop_on phrase appears or timeout.
        Returns (matched_phrase, line) or (None, None).
        """
        stop_on = stop_on or []
        fail_on = fail_on or ["E-STOP", "ERROR", "ABORTED", "failed"]
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self._read_line(timeout=1.0)
            if line is None:
                continue
            self._update_state_from_line(line)
            for phrase in stop_on:
                if phrase in line:
                    return phrase, line
            for phrase in fail_on:
                if phrase in line:
                    return None, line
        return None, None

    def _update_state_from_line(self, line: str):
        """Parse any ESP32 output line and update Jetson-side state."""
        if "Homing complete" in line:
            self.state = "IDLE"
            self.current_bin = 0
        elif "Done. Now at BIN" in line:
            m = re.search(r"BIN(\d)", line)
            if m:
                self.current_bin = int(m.group(1))
            self.state = "GATE_LOCKOUT"
        elif "GATE: BLOCKED" in line:
            self.state = "GATE_LOCKOUT"
        elif "GATE: Ready" in line or "Press 'i'" in line:
            self.state = "INVENTORY_PENDING"
        elif "Inventory complete" in line:
            self.state = "IDLE"
        elif "E-STOP" in line or "E-stop" in line:
            self.state = "ESTOP"
        elif "Distance(cm)" in line:
            m = re.search(r"Distance\(cm\)\s*=\s*([\d.]+)", line)
            if m:
                self.last_dist_cm = float(m.group(1))
                result = "HI" if self.last_dist_cm > INVENTORY_THRESHOLD_CM else "LO"
                if self.current_bin >= 0:
                    self.last_stock[self.current_bin] = result

    # ── High-level commands ───────────────────────────────────────
    def home(self, timeout=90.0) -> bool:
        if self.state == "ESTOP":
            return False
        self.send("h")
        self.state = "HOMING"
        matched, _ = self._drain_and_watch(timeout=timeout,
                                           stop_on=["Homing complete"])
        return matched is not None

    def move_to_bin(self, bin_num: int, timeout=60.0) -> bool:
        if self.state != "IDLE":
            return False
        self.send(f"bin{bin_num}")
        matched, _ = self._drain_and_watch(timeout=timeout,
                                           stop_on=["Done. Now at BIN"])
        return matched is not None

    def run_inventory(self, timeout=20.0):
        """Send 'i', wait for distance reading. Returns 'HI', 'LO', or None."""
        if self.state != "INVENTORY_PENDING":
            return None
        self.send("i")
        matched, _ = self._drain_and_watch(timeout=timeout,
                                           stop_on=["Distance(cm)"])
        if matched and self.last_dist_cm is not None:
            return "HI" if self.last_dist_cm > INVENTORY_THRESHOLD_CM else "LO"
        return None

    def emergency_stop(self):
        """Send 'e' immediately, update state."""
        self.send("e")
        self.state = "ESTOP"

    def wait_for_gate_reinsert(self, timeout=120.0) -> bool:
        """Wait for user to pull bin and push it back in (up to timeout seconds)."""
        matched, _ = self._drain_and_watch(timeout=timeout,
                                           stop_on=["Press 'i'", "GATE: Ready"])
        return matched is not None

    def dispense(self, bin_num: int, gate_timeout=120.0, progress_cb=None) -> dict:
        """
        Full dispense workflow:
          1. Home if BOOT/DISCONNECTED
          2. Move carousel to bin_num
          3. Wait for user to pull and re-insert bin (gate cycle)
          4. Run inventory sensor
          5. Return result dict with success, stock_level ('HI'/'LO'), distance_cm
        """
        def _cb(msg):
            if progress_cb:
                progress_cb(msg)
            print(f"[DISPENSE] {msg}")

        if self.state in ("BOOT", "DISCONNECTED"):
            _cb("Homing motor system…")
            if not self.home():
                return {"success": False, "error": "Homing failed"}

        if self.state == "ESTOP":
            return {"success": False, "error": "System is in E-STOP. Restart ESP32."}

        if self.state != "IDLE":
            return {"success": False, "error": f"Cannot dispense in state: {self.state}"}

        _cb(f"Moving carousel to BIN {bin_num}…")
        if not self.move_to_bin(bin_num):
            return {"success": False, "error": f"Failed to move to BIN {bin_num}"}

        _cb("Bin presented at access point. Pull component, then push bin back in.")
        if not self.wait_for_gate_reinsert(timeout=gate_timeout):
            return {"success": False, "error": "Gate timeout – bin not re-inserted in time"}

        _cb("Running inventory sensor (≈4 s)…")
        stock = self.run_inventory()
        if stock is None:
            return {"success": False, "error": "Inventory sensing failed"}

        return {
            "success":     True,
            "bin":         bin_num,
            "stock_level": stock,
            "distance_cm": self.last_dist_cm,
        }

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    @property
    def status_label(self) -> str:
        return {
            "DISCONNECTED":      "🔴 Disconnected",
            "BOOT":              "🟡 Waiting for home",
            "HOMING":            "🔄 Homing…",
            "IDLE":              "🟢 Ready",
            "GATE_LOCKOUT":      "🟠 Bin out – waiting for re-insert",
            "INVENTORY_PENDING": "🔵 Re-inserted – running inventory",
            "ESTOP":             "🔴 EMERGENCY STOP – restart ESP32",
        }.get(self.state, self.state)


# ─── [MOTOR] Singleton controller in session state ────────────────────────────
def get_controller() -> ESP32Controller:
    if "esp32" not in st.session_state:
        ctrl = ESP32Controller()
        ctrl.connect()
        st.session_state["esp32"] = ctrl
    return st.session_state["esp32"]


# ─── [MOTOR] LLM component classification ────────────────────────────────────
def classify_component(text: str):
    """Map natural language to a COMPONENT_TO_BIN key, or None."""
    t = text.lower().replace(" ", "").replace("-", "").replace("_", "")

    if any(k in t for k in ["1kohm", "1k", "1000ohm", "1kresistor"]):
        return "1kohm"
    if any(k in t for k in ["2.2kohm", "2k2", "2200ohm", "2.2k", "2k2ohm",
                              "2.2kresistor"]):
        return "2k2ohm"
    if any(k in t for k in ["470nf", "470nanofarad", "0.47uf", "0.47µf",
                              "470n", "capacitor", "cap"]):
        return "470nf"
    if any(k in t for k in ["led", "lightemitting", "diode"]):
        return "led"
    return None


# ─── Whisper ──────────────────────────────────────────────────────────────────
@st.cache_resource
def load_whisper_model():
    try:
        import torch
        device = WHISPER_DEVICE if torch.cuda.is_available() else "cpu"
        return whisper.load_model(WHISPER_MODEL, device=device)
    except Exception as e:
        st.error(f"Failed to load Whisper model: {e}")
        return None


def transcribe_with_whisper(duration=5):
    model = load_whisper_model()
    if model is None:
        raise RuntimeError("Whisper model not loaded")
    audio_data = []

    def callback(indata, frames, t, status):
        audio_data.append(indata.copy())

    with sd.InputStream(samplerate=16000, channels=1,
                        dtype="float32", callback=callback):
        time.sleep(duration)

    audio_np = np.concatenate(audio_data, axis=0).flatten()
    import torch
    result = model.transcribe(audio_np, fp16=torch.cuda.is_available(),
                              language="en")
    return result["text"].strip()


# ─── TTS ──────────────────────────────────────────────────────────────────────
def speak_text(text):
    if TTS_ENGINE == "gtts":
        try:
            tts = gTTS(text)
            with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as fp:
                tts.save(fp.name)
                st.audio(fp.name, format="audio/mp3", autoplay=True)
                for cmd in [["mpg123", fp.name],
                            ["ffplay", "-nodisp", "-autoexit", fp.name]]:
                    try:
                        subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                         stderr=subprocess.DEVNULL)
                        break
                    except FileNotFoundError:
                        continue
        except Exception as e:
            st.warning(f"gTTS failed: {e}")
            speak_text_offline(text)
    elif TTS_ENGINE == "pyttsx3":
        speak_text_offline(text)
    elif TTS_ENGINE == "edge_tts":
        import edge_tts, asyncio
        async def _speak():
            communicate = edge_tts.Communicate(text, "en-US-JennyNeural")
            with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as fp:
                await communicate.save(fp.name)
                st.audio(fp.name, format="audio/mp3", autoplay=True)
        asyncio.run(_speak())


def speak_text_offline(text):
    try:
        import pyttsx3
        engine = pyttsx3.init()
        engine.setProperty("rate", 150)
        engine.setProperty("volume", 0.9)
        with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as fp:
            engine.save_to_file(text, fp.name)
            engine.runAndWait()
            st.audio(fp.name, format="audio/wav", autoplay=True)
    except Exception as e:
        st.error(f"Offline TTS failed: {e}")


# ─── FAQ helpers ──────────────────────────────────────────────────────────────
def load_faqs():
    if os.path.exists(FAQS_PATH):
        try:
            with open(FAQS_PATH, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    default = {
        "ECEN 214": [{"question": "How do I measure the output of an op-amp circuit?",
                      "answer":   "Connect the oscilloscope probe across the output terminal and ground."}],
        "Equipment Troubleshooting": [{"question": "The oscilloscope is not displaying a waveform.",
                                       "answer":   "Check probe connection, vertical scale, time base, and trigger level."}],
    }
    with open(FAQS_PATH, "w", encoding="utf-8") as f:
        json.dump(default, f, indent=2)
    return default


def save_faqs(faqs):
    with open(FAQS_PATH, "w", encoding="utf-8") as f:
        json.dump(faqs, f, indent=2)


def match_faq_local(user_input, embed_model, faqs, threshold=0.78):
    best_match, best_score = None, 0.0
    user_vec = embed_model.encode(user_input)
    for cat, qalist in faqs.items():
        for qa in qalist:
            score = util.cos_sim(user_vec, embed_model.encode(qa["question"])).item()
            if score > best_score:
                best_score, best_match = score, qa
    return best_match["answer"] if best_score >= threshold else None


def load_index():
    if not os.path.exists(INDEX_PATH):
        st.error(f"FAISS index not found at {INDEX_PATH}.")
        return None, None
    with open(INDEX_PATH, "rb") as f:
        index, texts = pickle.load(f)
    return index, texts


def retrieve_context(query, index, texts, embed_model, k=3):
    if index is None or texts is None:
        return ""
    qvec = embed_model.encode([query])
    _, indices = index.search(qvec, k)
    return "\n\n".join(texts[i] for i in indices[0] if 0 <= i < len(texts))


def query_tamuai(prompt):
    if not API_KEY or not API_URL:
        raise RuntimeError("API_KEY or API_URL not set in environment.")
    headers = {"Authorization": f"Bearer {API_KEY}",
               "Content-Type": "application/json"}
    payload = {"model": "protected.llama3.2", "stream": False,
               "messages": [{"role": "user", "content": prompt}]}
    r = requests.post(f"{API_URL}/api/chat/completions",
                      headers=headers, json=payload, timeout=60)
    r.raise_for_status()
    return r.json()["choices"][0]["message"]["content"]


# ─── CSS ──────────────────────────────────────────────────────────────────────
def inject_css():
    st.markdown("""
    <style>
        * { color: black !important; }
        .stApp { background-color: white !important; }
        section[data-testid="stSidebar"] { background-color: white !important; }
        div.stButton > button {
            color: white !important;
            background-color: #444444 !important;
            border-radius: 8px !important;
        }
        div[data-baseweb="input"] > div { background-color: black !important; }
        div[data-baseweb="input"] input { color: white !important; }
        div[data-baseweb="input"] input::placeholder { color: #cccccc !important; }
    </style>
    """, unsafe_allow_html=True)


# ─── Prompt injection ─────────────────────────────────────────────────────────
GROQ_API_KEY = os.getenv("GROQ_API_KEY")
if GROQ_API_KEY:
    detector = PromptInjectionDetector(use_groq=True, api_key=GROQ_API_KEY)
else:
    detector = PromptInjectionDetector(model_name_or_url="deberta")

detector.enable_keyword_blocking = True
detector.add_input_keywords(["ignore all previous", "bypass", "system prompt",
                              "jailbreak", "override"])
detector.add_output_keywords(["i am hacked", "i am compromised", "system instructions"])
detector.set_input_block_message("Input blocked: {matched_keywords}")
detector.set_output_block_message("Output blocked: {matched_keywords}")


# ─── Session state defaults ───────────────────────────────────────────────────
def init_session():
    st.session_state.setdefault("dispense_state",      "IDLE")
    st.session_state.setdefault("selected_component",  None)
    st.session_state.setdefault("selected_quantity",   1)
    st.session_state.setdefault("stt_result",          "")
    st.session_state.setdefault("page",                "Chatbot")
    st.session_state.setdefault("dispense_log",        [])   # [MOTOR] audit log


# ─── [MOTOR] Sidebar motor controls ──────────────────────────────────────────
def render_motor_sidebar(ctrl: ESP32Controller):
    st.sidebar.markdown("---")
    st.sidebar.subheader("🔧 Motor System")
    st.sidebar.markdown(f"**Status:** {ctrl.status_label}")

    if not ctrl.is_connected:
        if st.sidebar.button("Connect to ESP32"):
            if ctrl.connect():
                st.sidebar.success("Connected!")
            else:
                st.sidebar.error("Connection failed. Check wiring/port.")
            st.rerun()
    else:
        if ctrl.state not in ("ESTOP",):
            if st.sidebar.button("Home Motor"):
                with st.sidebar:
                    with st.spinner("Homing…"):
                        ok = ctrl.home()
                st.sidebar.success("Homing complete.") if ok else \
                    st.sidebar.error("Homing failed.")
                st.rerun()

        # Emergency stop always shown when connected
        if st.sidebar.button("⛔ EMERGENCY STOP"):
            ctrl.emergency_stop()
            st.sidebar.error("E-STOP sent. Restart ESP32 to continue.")
            st.rerun()

    # Live stock summary
    if ctrl.last_stock:
        st.sidebar.markdown("**Last Stock Readings:**")
        bin_names = {0: "BIN 0 – 1kΩ", 1: "BIN 1 – 2.2kΩ",
                     2: "BIN 2 – 470nF", 3: "BIN 3 – LED"}
        for b, level in ctrl.last_stock.items():
            icon = "🟢" if level == "HI" else "🔴"
            st.sidebar.markdown(f"{icon} {bin_names.get(b, f'BIN {b}')}: **{level}**")


# ─── [MOTOR] Dispenser UI section ─────────────────────────────────────────────
def render_dispenser_section(ctrl: ESP32Controller):
    """
    Dispenser widget with full motor-control state flow:
    IDLE → CONFIRM → PROCESSING (move+gate wait+inventory) → SUCCESS / ERROR
    """
    st.subheader("Component Dispenser")

    display_map = {
        "1kΩ Resistor (BIN 0)":     "1kohm",
        "2.2kΩ Resistor (BIN 1)":   "2k2ohm",
        "470 nF Capacitor (BIN 2)": "470nf",
        "LED Light (BIN 3)":        "led",
    }

    dstate = st.session_state["dispense_state"]

    # ── IDLE ──────────────────────────────────────────────────────
    if dstate == "IDLE":
        selected_label = st.selectbox("Component",
                                      ["Select…"] + list(display_map.keys()),
                                      key="disp_select")
        qty = st.number_input("Quantity", min_value=1, max_value=10, value=1,
                              key="disp_qty")
        if st.button("Review Request") and selected_label != "Select…":
            st.session_state["selected_component"] = display_map[selected_label]
            st.session_state["selected_quantity"]  = qty
            st.session_state["dispense_state"]     = "CONFIRM"
            st.rerun()

    # ── CONFIRM ───────────────────────────────────────────────────
    elif dstate == "CONFIRM":
        comp_key = st.session_state["selected_component"]
        bin_num  = COMPONENT_TO_BIN[comp_key]
        st.warning("Confirm Component Request")
        st.write(f"**Component:** {comp_key}  →  BIN {bin_num}")
        st.write(f"**Quantity:** {st.session_state['selected_quantity']}")
        col_a, col_b = st.columns(2)
        with col_a:
            if st.button("Confirm"):
                st.session_state["dispense_state"] = "PROCESSING"
                st.rerun()
        with col_b:
            if st.button("Cancel"):
                st.session_state["dispense_state"] = "IDLE"
                st.rerun()

    # ── PROCESSING ────────────────────────────────────────────────
    elif dstate == "PROCESSING":
        if ctrl.state == "ESTOP":
            st.error("E-STOP active. Restart ESP32 before dispensing.")
            st.session_state["dispense_state"] = "IDLE"
            return

        comp_key  = st.session_state["selected_component"]
        bin_num   = COMPONENT_TO_BIN[comp_key]
        status_ph = st.empty()

        def update_status(msg):
            status_ph.info(f"⏳ {msg}")

        result = ctrl.dispense(bin_num, gate_timeout=120.0,
                               progress_cb=update_status)

        # [MOTOR] Append to audit log
        st.session_state["dispense_log"].append({
            "timestamp":   datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "component":   comp_key,
            "bin":         bin_num,
            "success":     result["success"],
            "stock":       result.get("stock_level", "—"),
            "distance_cm": result.get("distance_cm", "—"),
        })

        st.session_state["dispense_result"] = result
        st.session_state["dispense_state"]  = "SUCCESS" if result["success"] else "ERROR"
        st.rerun()

    # ── SUCCESS ───────────────────────────────────────────────────
    elif dstate == "SUCCESS":
        result = st.session_state.get("dispense_result", {})
        st.success("✅ Component dispensed successfully!")
        stock = result.get("stock_level", "Unknown")
        dist  = result.get("distance_cm")
        icon  = "🟢" if stock == "HI" else "🔴"
        st.write(f"**BIN {result.get('bin')}** presented at access point.")
        st.write(f"{icon} **Remaining stock:** {stock}"
                 + (f" ({dist:.1f} cm from sensor)" if dist else ""))
        if stock == "LO":
            st.warning("⚠️ Stock is low in this bin. Notify the lab technician.")
        if st.button("Request another component"):
            st.session_state["dispense_state"]     = "IDLE"
            st.session_state["selected_component"] = None
            st.session_state.pop("dispense_result", None)
            st.rerun()

    # ── ERROR ─────────────────────────────────────────────────────
    elif dstate == "ERROR":
        result = st.session_state.get("dispense_result", {})
        st.error(f"❌ Dispense failed: {result.get('error', 'Unknown error')}")
        col_retry, col_cancel = st.columns(2)
        with col_retry:
            if st.button("Retry"):
                st.session_state["dispense_state"] = "CONFIRM"
                st.rerun()
        with col_cancel:
            if st.button("Cancel"):
                st.session_state["dispense_state"] = "IDLE"
                st.rerun()


# ─── Admin pages ──────────────────────────────────────────────────────────────
def admin_login_page():
    st.subheader("Admin Login")
    username = st.text_input("Username", key="admin_user")
    password = st.text_input("Password", type="password", key="admin_pass")
    col1, col2 = st.columns(2)
    with col1:
        login_clicked = st.button("Login")
    with col2:
        if st.button("Cancel"):
            st.session_state["page"] = "Chatbot"
            st.rerun()
    if login_clicked:
        if username == "admin" and password == "password":
            st.session_state["admin_logged_in"] = True
            st.session_state["page"] = "Admin Dashboard"
            st.rerun()
        else:
            st.error("Invalid credentials")


def admin_dashboard_page(faqs):
    if not st.session_state.get("admin_logged_in", False):
        st.warning("You must log in as admin.")
        st.session_state["page"] = "Admin Login"
        st.rerun()

    st.title("Admin Dashboard")

    col1, col2, col3 = st.columns(3)
    with col1:
        if st.button("Logout"):
            st.session_state["admin_logged_in"] = False
            st.session_state["page"] = "Chatbot"
            st.rerun()
    with col2:
        st.metric("FAQ categories", len(faqs))
    with col3:
        count = 0
        if os.path.exists(INAPPROPRIATE_LOG):
            with open(INAPPROPRIATE_LOG, "r", encoding="utf-8") as f:
                count = sum(1 for _ in f)
        st.metric("Inappropriate queries", count)

    # ── [MOTOR] Live stock table ──────────────────────────────────
    st.markdown("---")
    st.subheader("Component Stock (live from ESP32)")
    ctrl = get_controller()
    bin_info = {0: "1kΩ Resistor", 1: "2.2kΩ Resistor",
                2: "470 nF Capacitor", 3: "LED Lights"}
    rows = []
    for b, name in bin_info.items():
        level = ctrl.last_stock.get(b, "Unknown")
        dist  = (f"{ctrl.last_dist_cm:.1f} cm"
                 if b == ctrl.current_bin and ctrl.last_dist_cm else "—")
        icon  = {"HI": "🟢", "LO": "🔴"}.get(level, "⚪")
        rows.append({"Bin": f"BIN {b}", "Component": name,
                     "Stock": f"{icon} {level}", "Last Distance": dist})
    st.table(rows)

    # ── [MOTOR] Dispense audit log ────────────────────────────────
    st.markdown("---")
    st.subheader("Dispense Audit Log")
    log = st.session_state.get("dispense_log", [])
    if log:
        st.dataframe(log)
        if st.button("Clear dispense log"):
            st.session_state["dispense_log"] = []
            st.rerun()
    else:
        st.info("No dispense events yet.")

    # ── Inappropriate query log ────────────────────────────────────
    st.markdown("---")
    st.subheader("Inappropriate Queries")
    if os.path.exists(INAPPROPRIATE_LOG):
        with open(INAPPROPRIATE_LOG, "r", encoding="utf-8") as f:
            lines = [l.strip() for l in f if l.strip()]
        if lines:
            st.dataframe({"timestamped_query": lines})
            if st.button("Clear log"):
                open(INAPPROPRIATE_LOG, "w", encoding="utf-8").close()
                st.rerun()
        else:
            st.info("No inappropriate queries logged.")
    else:
        st.info("No log file found.")

    # ── FAQ editor ────────────────────────────────────────────────
    st.markdown("---")
    st.subheader("FAQ Editor")
    categories = list(faqs.keys())
    edit_mode  = st.radio("Mode:", ["Add FAQ", "Edit FAQ", "Delete FAQ"])

    if edit_mode == "Add FAQ":
        new_cat = st.text_input("Category", key="new_cat")
        new_q   = st.text_input("Question",  key="new_q")
        new_a   = st.text_area("Answer",     key="new_a")
        if st.button("Add FAQ"):
            if new_cat and new_q and new_a:
                faqs.setdefault(new_cat, []).append({"question": new_q, "answer": new_a})
                save_faqs(faqs)
                st.success("FAQ added.")
                st.rerun()
            else:
                st.error("Fill all fields.")

    elif edit_mode == "Edit FAQ":
        if categories:
            sel_cat = st.selectbox("Category", categories, key="edit_cat")
            q_list  = faqs.get(sel_cat, [])
            if q_list:
                idx = st.selectbox("Question", range(len(q_list)),
                                   format_func=lambda i: q_list[i]["question"],
                                   key="edit_qidx")
                edited_q = st.text_input("Question", value=q_list[idx]["question"], key="edited_q")
                edited_a = st.text_area("Answer",    value=q_list[idx]["answer"],   key="edited_a")
                if st.button("Save changes"):
                    faqs[sel_cat][idx] = {"question": edited_q, "answer": edited_a}
                    save_faqs(faqs)
                    st.success("FAQ updated.")
                    st.rerun()

    elif edit_mode == "Delete FAQ":
        if categories:
            sel_cat = st.selectbox("Category", categories, key="del_cat")
            q_list  = faqs.get(sel_cat, [])
            if q_list:
                idx = st.selectbox("Question", range(len(q_list)),
                                   format_func=lambda i: q_list[i]["question"],
                                   key="del_qidx")
                if st.button("Delete selected FAQ"):
                    faqs[sel_cat].pop(idx)
                    if not faqs[sel_cat]:
                        del faqs[sel_cat]
                    save_faqs(faqs)
                    st.success("FAQ deleted.")
                    st.rerun()

    st.markdown("---")
    st.subheader("Redirect to Database")
    if st.button("Open Component Database"):
        try:
            subprocess.Popen([sys.executable, "dummy_db.py"])
            time.sleep(1)
            webbrowser.open("http://localhost:5001")
            st.success("Database opened in new tab.")
        except Exception as e:
            st.error(f"Failed to launch: {e}")


# ─── Chatbot page ─────────────────────────────────────────────────────────────
def chatbot_page(index, texts, embed_model, faqs):
    st.title("ECEN Chatbot")

    ctrl = get_controller()

    # [MOTOR] Motor status banner
    state_color = {
        "IDLE":              "success",
        "HOMING":            "info",
        "GATE_LOCKOUT":      "warning",
        "INVENTORY_PENDING": "info",
        "ESTOP":             "error",
        "DISCONNECTED":      "error",
        "BOOT":              "warning",
    }.get(ctrl.state, "info")
    getattr(st, state_color)(f"Motor: {ctrl.status_label}")

    st.write("Ask lab-related or course questions by typing or speaking.")

    # Sidebar FAQ picks
    with st.sidebar:
        st.header("Frequently Asked Questions")
        for category, qa_list in faqs.items():
            with st.expander(category):
                for qa in qa_list:
                    if st.button(qa["question"], key=f"faq_{qa['question']}"):
                        st.session_state["prefilled_question"] = qa["question"]
                        st.session_state["prefilled_answer"]   = qa["answer"]

    # STT + text input row
    col1, col2 = st.columns([4, 1])
    with col2:
        if st.button("Speak"):
            with st.spinner("Listening for 5 seconds…"):
                try:
                    transcribed = transcribe_with_whisper(duration=5)
                    if transcribed:
                        st.session_state["stt_result"] = transcribed
                        st.session_state.pop("prefilled_question", None)
                        st.session_state.pop("prefilled_answer", None)
                        st.success(f"You said: {transcribed}")
                        st.rerun()
                    else:
                        st.warning("No speech detected.")
                except Exception as e:
                    st.error(f"STT error: {e}")
    with col1:
        current_input = st.session_state.get(
            "stt_result",
            st.session_state.get("prefilled_question", "")
        )
        user_input = st.text_input("Type your question here:",
                                   value=current_input, key="user_input")

    # Camera + Dispenser columns
    camera_col, component_col = st.columns(2)
    with camera_col:
        if st.button("Camera Vision", use_container_width=True):
            st.markdown("Camera vision coming soon.")
    with component_col:
        render_dispenser_section(ctrl)   # [MOTOR] full motor-aware dispenser

    # FAQ prefill display
    default_q = st.session_state.get("prefilled_question", "")
    if "prefilled_answer" in st.session_state and default_q \
            and not st.session_state.get("stt_result"):
        st.subheader("FAQ Answer")
        st.write(st.session_state["prefilled_answer"])
        col_play, col_clear = st.columns(2)
        with col_play:
            if st.button("Play FAQ answer"):
                speak_text(st.session_state["prefilled_answer"])
        with col_clear:
            if st.button("Clear FAQ"):
                st.session_state.pop("prefilled_question", None)
                st.session_state.pop("prefilled_answer", None)
                st.rerun()
        return

    # ── Natural language query pipeline ───────────────────────────
    if user_input:
        # [MOTOR] Check if the query is a component request first
        comp_key = classify_component(user_input)
        if comp_key:
            bin_num = COMPONENT_TO_BIN[comp_key]
            st.info(f"Detected component request: **{comp_key}** → BIN {bin_num}")
            if ctrl.state == "ESTOP":
                st.error("Motor system is in E-STOP. Cannot dispense.")
                return
            st.session_state["selected_component"] = comp_key
            st.session_state["dispense_state"]     = "CONFIRM"
            st.session_state["stt_result"]         = ""
            st.rerun()
            return

        st.session_state["stt_result"] = ""

        # Injection safety check
        is_injection, _ = detector.detect_injection(user_input)
        blocked, _      = detector.check_input_keywords(user_input)
        if is_injection or blocked:
            st.error("Input blocked: potential injection or unsafe content.")
            with open(INAPPROPRIATE_LOG, "a", encoding="utf-8") as f:
                f.write(f"[{datetime.now()}] {user_input}\n")
            return

        # Local FAQ match
        local_answer = match_faq_local(user_input, embed_model, faqs)
        if local_answer:
            st.subheader("FAQ Match (Local)")
            st.write(local_answer)
            if st.button("Play FAQ Response"):
                speak_text(local_answer)
            return

        # RAG + LLM
        context = retrieve_context(user_input, index, texts, embed_model)
        prompt  = f"Context:\n{context}\n\nQuestion: {user_input}\nAnswer clearly and concisely."
        try:
            with st.spinner("Thinking…"):
                answer = query_tamuai(prompt)
        except Exception as e:
            st.error(f"LLM request failed: {e}")
            return

        safe, _ = detector.check_response_safety(answer)
        if not safe:
            st.error("Unsafe content detected in model output.")
            with open(INAPPROPRIATE_LOG, "a", encoding="utf-8") as f:
                f.write(f"[{datetime.now()}] {user_input}\n")
            return

        st.subheader("Response")
        st.write(answer)
        if st.button("Play Response"):
            speak_text(answer)


# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    st.set_page_config(page_title="ECEN Chatbot", layout="wide")
    inject_css()
    init_session()

    embed_model = None
    try:
        embed_model = SentenceTransformer(MODEL_NAME)
    except Exception as e:
        st.error(f"Failed to load embedding model: {e}")

    index, texts = load_index()
    faqs         = load_faqs()

    # [MOTOR] Motor sidebar rendered on every page
    ctrl = get_controller()
    render_motor_sidebar(ctrl)

    st.sidebar.title("Navigation")
    page = st.sidebar.radio(
        "Go to:",
        ["Chatbot", "Admin Login", "Admin Dashboard"],
        index=["Chatbot", "Admin Login", "Admin Dashboard"].index(
            st.session_state["page"]
        )
    )
    st.session_state["page"] = page

    if page == "Admin Login":
        admin_login_page()
    elif page == "Admin Dashboard":
        admin_dashboard_page(faqs)
    else:
        if embed_model is None or index is None or texts is None:
            st.error("Embeddings or index not loaded.")
            return
        chatbot_page(index, texts, embed_model, faqs)


if __name__ == "__main__":
    main()
# rag_ui.py
import os
import json
import tempfile
import time
import csv
import subprocess
import sys
import webbrowser

import numpy as np
import serial
import streamlit as st
import requests
import whisper
import sounddevice as sd

from gtts import gTTS
from sentence_transformers import SentenceTransformer, util
import pickle
from datetime import datetime
from pytector import PromptInjectionDetector
from config import STT_ENGINE, TTS_ENGINE

# Load .env file if present (for local dev — on Jetson vars come from .bashrc)
try:
    from dotenv import load_dotenv
    load_dotenv("dotenv.env")  # explicit filename
except ImportError:
    pass  # python-dotenv not installed — rely on environment variables directly

# ═══════════════════════════════════════════════════════════
#                     CONFIGURATION
# ═══════════════════════════════════════════════════════════

API_KEY      = os.getenv("TAMUS_AI_CHAT_API_KEY")
API_URL      = os.getenv("TAMUS_AI_CHAT_API_ENDPOINT")
INDEX_PATH   = "index/vector_index.pkl"
MODEL_NAME   = "all-MiniLM-L6-v2"
FAQS_PATH    = "faqs.json"
INAPPROPRIATE_LOG = "inappropriate_queries.txt"
WHISPER_MODEL     = "base"
WHISPER_DEVICE    = "cuda"

# ── UART ─────────────────────────────────────────────────────
SERIAL_PORT = "/dev/ttyTHS1"
BAUD_RATE   = 115200

# ── CSV log ───────────────────────────────────────────────────
LOG_FILE    = os.path.join(os.path.dirname(os.path.abspath(__file__)), "inventory_log.csv")
CSV_HEADERS = ["timestamp", "result", "distance_cm", "bin"]

# ── Component -> Bin mapping ──────────────────────────────────
COMPONENT_TO_BIN = {
    "1kohm":     0,
    "10kohm":    1,
    "cap_100nf": 2,
    "led_red":   3,
}
COMPONENT_DISPLAY = {
    "1kohm":     "1kΩ Resistor",
    "10kohm":    "10kΩ Resistor",
    "cap_100nf": "0.1µF Capacitor",
    "led_red":   "Red LED",
}

# ── Timeouts ──────────────────────────────────────────────────
HOME_TIMEOUT_S = 30
MOVE_TIMEOUT_S = 30
GATE_TIMEOUT_S = 30
INV_TIMEOUT_S  = 20


# ═══════════════════════════════════════════════════════════
#               SERIAL / BIN CONTROLLER LAYER
# ═══════════════════════════════════════════════════════════

def _open_serial() -> serial.Serial | None:
    os.system(f"sudo stty -F {SERIAL_PORT} raw {BAUD_RATE} cs8 -cstopb -parenb")
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        ser.reset_input_buffer()
        return ser
    except serial.SerialException as e:
        print(f"[SERIAL] ERROR: {e}")
        return None


def _send(ser: serial.Serial, cmd: str):
    ser.write((cmd + "\n").encode("utf-8"))
    ser.flush()
    print(f"[SERIAL] Sent: {cmd}")


def _wait_for(ser: serial.Serial, target: str, timeout_s: float) -> tuple[bool, list[str]]:
    """Read lines from ESP32 until target string found or timeout. Returns (found, lines)."""
    deadline = time.time() + timeout_s
    lines = []
    while time.time() < deadline:
        if ser.in_waiting:
            try:
                raw = ser.readline().decode("utf-8", errors="replace").strip()
                if raw:
                    print(f"[ESP32] {raw}")
                    lines.append(raw)
                    if target in raw:
                        return True, lines
            except Exception as e:
                print(f"[SERIAL] Read error: {e}")
        time.sleep(0.01)
    return False, lines


def _log_inventory(result: str, distance: str, bin_num: int):
    if not os.path.exists(LOG_FILE):
        with open(LOG_FILE, "w", newline="") as f:
            csv.writer(f).writerow(CSV_HEADERS)
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(LOG_FILE, "a", newline="") as f:
        csv.writer(f).writerow([timestamp, result, distance or "N/A", bin_num])
    print(f"[LOG] {timestamp} | {result} | {distance} | bin{bin_num}")


def _get_last_inventory(bin_num: int) -> str | None:
    if not os.path.exists(LOG_FILE):
        return None
    last = None
    with open(LOG_FILE, "r", newline="") as f:
        for row in csv.DictReader(f):
            try:
                if int(row["bin"]) == bin_num:
                    last = row["result"]
            except (ValueError, KeyError):
                continue
    return last


def _inventory_background(ser: serial.Serial, gate_lines: list, bin_num: int):
    """
    Sends 'i' and logs the inventory result silently in a background thread.
    Called after the user has already taken their component — UI doesn't wait for this.
    Closes the serial port when done.
    """
    try:
        for _ in range(20):
            _send(ser, "i")
        _, inv_lines = _wait_for(ser, "Inventory complete", INV_TIMEOUT_S)

        inv_result   = "UNKNOWN"
        inv_distance = "N/A"
        for line in gate_lines + inv_lines:
            if line == "HI":
                inv_result = "HI"
            elif line == "LO":
                inv_result = "LO"
            elif line.startswith("(Distance(cm)"):
                try:
                    inv_distance = line.strip("()").split("=")[-1].strip()
                except Exception:
                    pass

        _log_inventory(inv_result, inv_distance, bin_num)
        print(f"[INV] bin{bin_num} → {inv_result} ({inv_distance} cm)")
    finally:
        ser.close()


def dispense_component(component_key: str) -> dict:
    """
    Phase 1 (blocks inside st.spinner):
      - Home carousel
      - Move to bin
      - Wait for user to pull bin, take component, reinsert bin (GATE: Ready)

    Once GATE: Ready is received the user already has their component.
    UI immediately clears and shows success.

    Phase 2 (fire-and-forget background thread):
      - Send 'i', wait for Inventory complete, log HI/LO to CSV.
      - User never waits for this — it runs silently after the UI is already free.
    """
    bin_num      = COMPONENT_TO_BIN.get(component_key)
    display_name = COMPONENT_DISPLAY.get(component_key, component_key)

    if bin_num is None:
        return {"success": False, "message": f"No bin mapped for '{component_key}'.", "inventory": "UNKNOWN"}

    ser = _open_serial()
    if ser is None:
        return {"success": False, "message": "Could not open serial port.", "inventory": "UNKNOWN"}

    try:
        # Step 1: Home — only runs once per UI session
        if not st.session_state.get("carousel_homed", False):
            for _ in range(20):
                _send(ser, "h")
            ok, _ = _wait_for(ser, "Homing complete", HOME_TIMEOUT_S)
            if not ok:
                ser.close()
                return {"success": False, "message": "Homing timed out.", "inventory": "UNKNOWN"}
            st.session_state["carousel_homed"] = True
            print("[HOME] Homing complete — will not home again this session.")
        else:
            print("[HOME] Already homed this session — skipping.")

        # Step 2: Move to bin
        for _ in range(20):
            _send(ser, f"bin{bin_num}")
        ok, _ = _wait_for(ser, f"Done. Now at BIN{bin_num}", MOVE_TIMEOUT_S)
        if not ok:
            ser.close()
            return {"success": False, "message": f"Failed to move to bin {bin_num}.", "inventory": "UNKNOWN"}

        # Step 3: Wait for user to pull bin, take component, reinsert bin
        ok, gate_lines = _wait_for(ser, "GATE: Ready", GATE_TIMEOUT_S)
        if not ok:
            ser.close()
            return {"success": False, "message": "Timed out waiting for bin to be reinserted.", "inventory": "UNKNOWN"}

        # User has their component — hand off to background thread for inventory sensing.
        # Do NOT close ser here — the background thread owns it and will close it.
        import threading
        t = threading.Thread(
            target=_inventory_background,
            args=(ser, gate_lines, bin_num),
            daemon=True
        )
        t.start()

        return {
            "success": True,
            "message": f"{display_name} dispensed successfully.",
            "inventory": "UNKNOWN",  # logged async — check CSV for result
        }

    except Exception as e:
        ser.close()
        return {"success": False, "message": f"Dispense error: {e}", "inventory": "UNKNOWN"}


# ═══════════════════════════════════════════════════════════
#                     WHISPER / TTS
# ═══════════════════════════════════════════════════════════

@st.cache_resource
def load_whisper_model():
    try:
        import torch
        device = WHISPER_DEVICE if torch.cuda.is_available() else "cpu"
        return whisper.load_model(WHISPER_MODEL, device=device)
    except Exception as e:
        st.error(f"Failed to load Whisper: {e}")
        return None


def transcribe_with_whisper(duration=5):
    model = load_whisper_model()
    if model is None:
        raise RuntimeError("Whisper not loaded")
    audio_data = []
    def cb(indata, frames, t, status):
        audio_data.append(indata.copy())
    with sd.InputStream(samplerate=16000, channels=1, dtype="float32", callback=cb):
        time.sleep(duration)
    audio_np = np.concatenate(audio_data, axis=0).flatten()
    import torch
    result = model.transcribe(audio_np, fp16=torch.cuda.is_available(), language="en")
    return result["text"].strip()


def speak_text(text):
    if TTS_ENGINE == "gtts":
        try:
            tts = gTTS(text)
            with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as fp:
                tts.save(fp.name)
                st.audio(fp.name, format="audio/mp3", autoplay=True)
        except Exception as e:
            st.warning(f"TTS failed: {e}")
    elif TTS_ENGINE == "pyttsx3":
        try:
            import pyttsx3
            engine = pyttsx3.init()
            with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as fp:
                engine.save_to_file(text, fp.name)
                engine.runAndWait()
                st.audio(fp.name, format="audio/wav", autoplay=True)
        except Exception as e:
            st.error(f"Offline TTS failed: {e}")


# ═══════════════════════════════════════════════════════════
#                     FAQ / RAG / LLM
# ═══════════════════════════════════════════════════════════

def load_faqs():
    if os.path.exists(FAQS_PATH):
        try:
            with open(FAQS_PATH, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    default = {
        "ECEN 214": [{
            "question": "How do I measure the output of an op-amp circuit in the lab?",
            "answer": "Connect the oscilloscope probe across the output terminal and ground."
        }],
        "Equipment Troubleshooting": [{
            "question": "The oscilloscope is not displaying a waveform — what should I check?",
            "answer": "Confirm probe connection, vertical scale, time base settings, and trigger level."
        }]
    }
    with open(FAQS_PATH, "w", encoding="utf-8") as f:
        json.dump(default, f, indent=2)
    return default


def save_faqs(faqs):
    with open(FAQS_PATH, "w", encoding="utf-8") as f:
        json.dump(faqs, f, indent=2)


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


def query_tamuai(messages: list) -> str:
    """
    Send a full message history to the LLM.
    messages = [{"role": "user"|"assistant"|"system", "content": "..."}]
    """
    if not API_KEY or not API_URL:
        raise RuntimeError("API_KEY or API_URL not set.")
    headers = {"Authorization": f"Bearer {API_KEY}", "Content-Type": "application/json"}
    payload = {
        "model": "protected.llama3.2",
        "stream": False,
        "messages": messages,
    }
    r = requests.post(f"{API_URL}/api/chat/completions", headers=headers, json=payload, timeout=60)
    r.raise_for_status()
    return r.json()["choices"][0]["message"]["content"]


def match_faq_local(user_input, embed_model, faqs, threshold=0.78):
    best_match, best_score = None, 0.0
    user_vec = embed_model.encode(user_input)
    for cat, qalist in faqs.items():
        for qa in qalist:
            score = util.cos_sim(user_vec, embed_model.encode(qa["question"])).item()
            if score > best_score:
                best_score = score
                best_match = qa
    return best_match["answer"] if best_score >= threshold else None


def extract_component_request(text: str) -> str | None:
    t = text.lower().replace(" ", "")
    if any(x in t for x in ["1kohm", "1kresistor", "1k", "1000ohm"]):
        return "1kohm"
    if any(x in t for x in ["10kohm", "10kresistor", "10k", "10000ohm"]):
        return "10kohm"
    if any(x in t for x in ["0.1uf", "0.1µf", "100nf", "capacitor"]):
        return "cap_100nf"
    if any(x in t for x in ["redled", "rled", "ledr", "redlight"]):
        return "led_red"
    return None


# ═══════════════════════════════════════════════════════════
#               PROMPT INJECTION DETECTOR
# ═══════════════════════════════════════════════════════════

GROQ_API_KEY = os.getenv("GROQ_API_KEY")
if GROQ_API_KEY:
    detector = PromptInjectionDetector(use_groq=True, api_key=GROQ_API_KEY)
else:
    detector = PromptInjectionDetector(model_name_or_url="deberta")

detector.enable_keyword_blocking = True
detector.add_input_keywords(["ignore all previous", "bypass", "system prompt", "jailbreak", "override"])
detector.add_output_keywords(["i am hacked", "i am compromised", "system instructions"])
detector.set_input_block_message("Input blocked: {matched_keywords}")
detector.set_output_block_message("Output blocked: {matched_keywords}")



# ═══════════════════════════════════════════════════════════
#                   CAMERA VISION
# ═══════════════════════════════════════════════════════════

# Path to the camera vision script (same folder as this file)
VISION_SCRIPT      = "/home/am1/GPT-NLP-Lab-Assistant-4/Raquel's Folder/403model/camera_local.py"
VISION_RESULT_FILE = "/home/am1/GPT-NLP-Lab-Assistant-4/Raquel's Folder/403model/prediction_result.txt"


def scan_component() -> dict:
    """
    Runs the camera vision script as a subprocess and reads the result string
    it writes to prediction_result.txt.

    The vision script handles everything: camera warm-up, inference against
    the local Roboflow Docker server, averaging, and writing the final
    component name to prediction_result.txt as "class,confidence".

    Returns { "success": bool, "component": str, "message": str }
    """
    # Remove stale result file so we don't accidentally read a previous run
    if os.path.exists(VISION_RESULT_FILE):
        os.remove(VISION_RESULT_FILE)

    try:
        # Run the vision script synchronously — blocks until it completes
        result = subprocess.run(
            [sys.executable, VISION_SCRIPT],
            timeout=60,
            capture_output=True,
            text=True,
        )
        print(f"[VISION] stdout: {result.stdout[-500:] if result.stdout else ''}")
        if result.returncode != 0:
            print(f"[VISION] stderr: {result.stderr[-300:] if result.stderr else ''}")
            return {"success": False, "component": "",
                    "message": f"Camera vision script failed (exit {result.returncode})."}
    except subprocess.TimeoutExpired:
        return {"success": False, "component": "",
                "message": "Camera vision timed out after 60 seconds."}
    except Exception as e:
        return {"success": False, "component": "", "message": f"Camera vision error: {e}"}

    # Read the result file written by the vision script
    if not os.path.exists(VISION_RESULT_FILE):
        return {"success": False, "component": "",
                "message": "Vision script ran but produced no result file."}

    try:
        with open(VISION_RESULT_FILE, "r") as f:
            line = f.read().strip()
        # Format is "class,confidence" e.g. "1k-resistor,0.8732"
        component = line.split(",")[0].strip()
        if not component or component == "none":
            return {"success": False, "component": "",
                    "message": "No component detected. Try repositioning and scanning again."}
        return {"success": True, "component": component,
                "message": f"Detected: {component}"}
    except Exception as e:
        return {"success": False, "component": "", "message": f"Could not read result: {e}"}

# ═══════════════════════════════════════════════════════════
#                       ADMIN PAGES
# ═══════════════════════════════════════════════════════════

def admin_login_page():
    st.subheader("Admin Login")
    username = st.text_input("Username", key="admin_user")
    password = st.text_input("Password", type="password", key="admin_pass")
    col1, col2 = st.columns(2)
    with col1:
        if st.button("Login"):
            if username == "admin" and password == "password":
                st.session_state["admin_logged_in"] = True
                st.session_state["page"] = "Admin Dashboard"
                st.rerun()
            else:
                st.error("Invalid credentials")
    with col2:
        if st.button("Cancel"):
            st.session_state["page"] = "Chatbot"
            st.rerun()


def admin_dashboard_page(faqs):
    if not st.session_state.get("admin_logged_in", False):
        st.session_state["page"] = "Admin Login"
        st.rerun()

    st.title("Admin Dashboard")
    col1, col2, col3 = st.columns([1, 2, 2])
    with col1:
        if st.button("Logout"):
            st.session_state["admin_logged_in"] = False
            st.session_state["page"] = "Chatbot"
            st.rerun()
    with col2:
        st.metric("FAQ categories", len(faqs.keys()))
    with col3:
        count = 0
        if os.path.exists(INAPPROPRIATE_LOG):
            with open(INAPPROPRIATE_LOG, "r", encoding="utf-8") as f:
                count = sum(1 for _ in f)
        st.metric("Inappropriate queries", count)

    st.markdown("---")
    st.subheader("Inventory Log")
    if os.path.exists(LOG_FILE):
        with open(LOG_FILE, "r", newline="") as f:
            rows = list(csv.DictReader(f))
        if rows:
            st.dataframe(rows)
        else:
            st.info("No inventory records yet.")
    else:
        st.info("No inventory log found.")

    st.markdown("---")
    st.subheader("Inappropriate Queries")
    if os.path.exists(INAPPROPRIATE_LOG):
        with open(INAPPROPRIATE_LOG, "r", encoding="utf-8") as f:
            lines = [l.strip() for l in f if l.strip()]
        if lines:
            st.dataframe({"timestamped_query": lines})
            if st.button("Clear log"):
                open(INAPPROPRIATE_LOG, "w").close()
                st.rerun()
        else:
            st.info("No inappropriate queries.")

    st.markdown("---")
    st.subheader("FAQ Editor")
    categories = list(faqs.keys())
    edit_mode = st.radio("Mode:", ["Add FAQ", "Edit FAQ", "Delete FAQ"])

    if edit_mode == "Add FAQ":
        new_cat = st.text_input("Category", key="new_cat")
        new_q   = st.text_input("Question", key="new_q")
        new_a   = st.text_area("Answer",    key="new_a")
        if st.button("Add FAQ"):
            if new_cat and new_q and new_a:
                faqs.setdefault(new_cat, []).append({"question": new_q, "answer": new_a})
                save_faqs(faqs)
                st.success("FAQ added.")
                st.rerun()
            else:
                st.error("Fill all fields.")

    elif edit_mode == "Edit FAQ" and categories:
        sel_cat = st.selectbox("Category", categories, key="edit_cat")
        q_list  = faqs.get(sel_cat, [])
        if q_list:
            idx = st.selectbox("Question", range(len(q_list)),
                               format_func=lambda i: q_list[i]["question"], key="edit_idx")
            eq = st.text_input("Question", value=q_list[idx]["question"], key="eq")
            ea = st.text_area("Answer",   value=q_list[idx]["answer"],   key="ea")
            if st.button("Save"):
                faqs[sel_cat][idx] = {"question": eq, "answer": ea}
                save_faqs(faqs)
                st.success("Updated.")
                st.rerun()

    elif edit_mode == "Delete FAQ" and categories:
        sel_cat = st.selectbox("Category", categories, key="del_cat")
        q_list  = faqs.get(sel_cat, [])
        if q_list:
            idx = st.selectbox("Question", range(len(q_list)),
                               format_func=lambda i: q_list[i]["question"], key="del_idx")
            if st.button("Delete"):
                faqs[sel_cat].pop(idx)
                if not faqs[sel_cat]:
                    del faqs[sel_cat]
                save_faqs(faqs)
                st.success("Deleted.")
                st.rerun()

    st.markdown("---")
    if st.button("Open Database"):
        import socket
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            already_running = s.connect_ex(("localhost", 5000)) == 0
        try:
            if not already_running:
                subprocess.Popen([sys.executable, "db.py"])
                time.sleep(1)
            webbrowser.open("http://localhost:5000")
        except Exception as e:
            st.error(f"Failed to launch database: {e}")


# ═══════════════════════════════════════════════════════════
#                      CHATBOT PAGE
# ═══════════════════════════════════════════════════════════

SYSTEM_PROMPT = (
    "You are a helpful electronics lab assistant at Texas A&M University. "
    "Answer questions about electronic components, circuits, lab equipment, "
    "and procedures clearly and concisely. When a component is identified by "
    "the camera, provide its full name, typical specifications, and how it is "
    "commonly used in electronics lab work."
)


def _render_chat_history():
    """Render the chat history bubbles."""
    for msg in st.session_state.get("chat_history", []):
        if msg["role"] == "system":
            continue
        with st.chat_message(msg["role"]):
            # If the message has an image attached, show it
            if msg.get("image") is not None:
                st.image(msg["image"], caption="Camera scan", width=320)
            st.write(msg["content"])


def _add_to_history(role: str, content: str, image=None):
    """Append a message to chat history."""
    entry = {"role": role, "content": content}
    if image is not None:
        entry["image"] = image
    st.session_state["chat_history"].append(entry)


def _llm_messages() -> list:
    """Build the messages list for the API — system prompt + history, no image entries."""
    msgs = [{"role": "system", "content": SYSTEM_PROMPT}]
    for msg in st.session_state.get("chat_history", []):
        if msg["role"] == "system":
            continue
        msgs.append({"role": msg["role"], "content": msg["content"]})
    return msgs


def chatbot_page(index, texts, embed_model, faqs):
    st.title("ECEN Chatbot")

    # ── Session state defaults ────────────────────────────────
    st.session_state.setdefault("chat_history", [])
    st.session_state.setdefault("stt_result", "")

    # ── Sidebar: FAQs + Clear History ────────────────────────
    with st.sidebar:
        if st.button("🗑 Clear Chat History", use_container_width=True):
            st.session_state["chat_history"] = []
            st.session_state["stt_result"] = ""
            st.session_state.pop("prefilled_question", None)
            st.session_state.pop("prefilled_answer", None)
            st.rerun()

        st.markdown("---")
        st.header("Frequently Asked Questions")
        for category, qa_list in faqs.items():
            with st.expander(category):
                for qa in qa_list:
                    if st.button(qa["question"], key=f"faq_{qa['question']}"):
                        st.session_state["prefilled_question"] = qa["question"]
                        st.session_state["prefilled_answer"]   = qa["answer"]

    # ── Dispense result banner (shown above chat) ─────────────
    if "dispense_result" in st.session_state:
        msg = st.session_state.pop("dispense_result")
        if msg.startswith("✅"):
            st.success(msg)
        else:
            st.error(msg)

    # ── Confirm dispense dialog (separate from chat) ──────────
    if "pending_dispense" in st.session_state:
        comp_key  = st.session_state["pending_dispense"]
        comp_name = COMPONENT_DISPLAY.get(comp_key, comp_key)
        st.warning(f"Confirm: dispense **{comp_name}**?")
        c1, c2 = st.columns(2)
        with c1:
            if st.button("✅ Confirm"):
                st.session_state.pop("pending_dispense")
                with st.spinner(f"Dispensing {comp_name}... please wait."):
                    result = dispense_component(comp_key)
                st.session_state["dispense_result"] = (
                    f"✅ {result['message']}" if result["success"] else f"❌ {result['message']}"
                )
                st.session_state["stt_result"] = ""
                st.session_state.pop("prefilled_question", None)
                st.session_state.pop("prefilled_answer", None)
                st.rerun()
        with c2:
            if st.button("❌ Cancel"):
                st.session_state.pop("pending_dispense")
                st.session_state["stt_result"] = ""
                st.session_state.pop("prefilled_question", None)
                st.rerun()
        return

    # ── Render chat history ───────────────────────────────────
    _render_chat_history()

    # ── Input row: text + speak + scan ───────────────────────
    col_input, col_speak, col_scan = st.columns([5, 1, 1])

    with col_speak:
        if st.button("🎤 Speak"):
            with st.spinner("Listening for 5 seconds..."):
                try:
                    transcribed = transcribe_with_whisper(duration=5)
                    if transcribed:
                        st.session_state["stt_result"] = transcribed
                        st.rerun()
                    else:
                        st.warning("No speech detected.")
                except Exception as e:
                    st.error(f"STT error: {e}")

    with col_scan:
        if st.button("📷 Scan"):
            with st.spinner("Scanning component... please wait."):
                scan = scan_component()
            if scan["success"]:
                component = scan["component"]
                # Pass the component string directly to the LLM — no confidence or frame needed
                user_msg = (
                    f"The camera identified this component: {component}. "
                    f"Please provide its full name, typical specifications, "
                    f"and how it is commonly used in an electronics lab."
                )
                _add_to_history("user", user_msg)
                try:
                    with st.spinner("Thinking..."):
                        answer = query_tamuai(_llm_messages())
                    _add_to_history("assistant", answer)
                    with st.chat_message("user"):
                        st.write(user_msg)
                    with st.chat_message("assistant"):
                        st.write(answer)
                except Exception as e:
                    st.error(f"LLM request failed: {e}")
            else:
                st.error(f"📷 {scan['message']}")

    with col_input:
        current_input = st.session_state.get("stt_result") or st.session_state.get("prefilled_question", "")
        user_input = st.text_input("Ask a question or request a component:", value=current_input, key="main_input")

    # ── Manual component dropdown ─────────────────────────────
    with st.expander("Or select a component manually"):
        component_options = {"Select...": None} | {v: k for k, v in COMPONENT_DISPLAY.items()}
        selected_display = st.selectbox("Component", list(component_options.keys()))
        if st.button("Request Component") and component_options[selected_display]:
            component_key = component_options[selected_display]
            bin_num       = COMPONENT_TO_BIN[component_key]
            comp_name     = COMPONENT_DISPLAY.get(component_key, component_key)
            last_inv      = _get_last_inventory(bin_num)
            if last_inv == "LO":
                st.error(f"**{comp_name}** is out of stock. Please ask a lab assistant to restock bin {bin_num}.")
            else:
                st.session_state["pending_dispense"] = component_key
                st.rerun()

    # ── Process text / voice input ────────────────────────────
    if not user_input:
        return

    st.session_state["stt_result"] = ""
    st.session_state.pop("prefilled_question", None)
    st.session_state.pop("prefilled_answer", None)

    # 1. Component dispense request
    component_key = extract_component_request(user_input)
    if component_key:
        comp_name = COMPONENT_DISPLAY.get(component_key, component_key)
        bin_num   = COMPONENT_TO_BIN[component_key]
        last_inv  = _get_last_inventory(bin_num)
        if last_inv == "LO":
            st.error(f"**{comp_name}** is out of stock. Please ask a lab assistant to restock bin {bin_num}.")
            return
        st.session_state["pending_dispense"] = component_key
        st.rerun()
        return

    # 2. Safety check
    is_injection, _ = detector.detect_injection(user_input)
    blocked, _      = detector.check_input_keywords(user_input)
    if is_injection or blocked:
        st.error("Input blocked: potential injection or unsafe content.")
        with open(INAPPROPRIATE_LOG, "a", encoding="utf-8") as f:
            f.write(f"[{datetime.now()}] {user_input}\n")
        return

    # 3. Local FAQ match — answered inline, also added to chat history
    local_answer = match_faq_local(user_input, embed_model, faqs)
    if local_answer:
        _add_to_history("user", user_input)
        _add_to_history("assistant", local_answer)
        # Rerender history with new messages — no st.rerun() to avoid loop
        with st.chat_message("user"):
            st.write(user_input)
        with st.chat_message("assistant"):
            st.write(local_answer)
        if st.button("▶ Play response"):
            speak_text(local_answer)
        return

    # 4. RAG + LLM with full chat history
    _add_to_history("user", user_input)
    context = retrieve_context(user_input, index, texts, embed_model)
    # Inject RAG context as a system note — not visible in chat history
    messages = _llm_messages()
    if context:
        messages.insert(1, {
            "role": "system",
            "content": f"Relevant lab documentation:\n{context}"
        })
    try:
        with st.spinner("Thinking..."):
            answer = query_tamuai(messages)
    except Exception as e:
        st.session_state["chat_history"].pop()  # remove the user msg we just added
        st.error(f"LLM request failed: {e}")
        return

    safe, _ = detector.check_response_safety(answer)
    if not safe:
        st.session_state["chat_history"].pop()
        st.error("Unsafe content in model response.")
        with open(INAPPROPRIATE_LOG, "a", encoding="utf-8") as f:
            f.write(f"[{datetime.now()}] {user_input}\n")
        return

    _add_to_history("assistant", answer)
    # Render the new exchange directly — no st.rerun() to avoid loop
    with st.chat_message("user"):
        st.write(user_input)
    with st.chat_message("assistant"):
        st.write(answer)
    if st.button("▶ Play response"):
        speak_text(answer)


# ═══════════════════════════════════════════════════════════
#                         MAIN
# ═══════════════════════════════════════════════════════════

def main():
    st.set_page_config(page_title="ECEN Chatbot", layout="wide")

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

    st.session_state.setdefault("page", "Chatbot")
    st.session_state.setdefault("admin_logged_in", False)
    st.session_state.setdefault("carousel_homed", False)

    # Home the carousel once at startup if not already done
    if not st.session_state["carousel_homed"]:
        with st.spinner("Homing carousel on startup..."):
            ser = _open_serial()
            if ser:
                try:
                    for _ in range(20):
                        _send(ser, "h")
                    ok, _ = _wait_for(ser, "Homing complete", HOME_TIMEOUT_S)
                    if ok:
                        st.session_state["carousel_homed"] = True
                        print("[HOME] Startup homing complete.")
                    else:
                        st.warning("Carousel homing timed out at startup. Will retry on first dispense.")
                finally:
                    ser.close()
            else:
                st.warning("Could not open serial port for startup homing. Will retry on first dispense.")

    embed_model = None
    try:
        # Force CPU to avoid competing with Whisper for Jetson shared GPU memory
        embed_model = SentenceTransformer(MODEL_NAME, device="cpu")
    except Exception as e:
        st.error(f"Embedding model failed: {e}")

    index, texts = load_index()
    faqs = load_faqs()

    st.sidebar.title("Navigation")
    page = st.sidebar.radio(
        "Go to:",
        ["Chatbot", "Admin Login", "Admin Dashboard"],
        index=["Chatbot", "Admin Login", "Admin Dashboard"].index(st.session_state["page"])
    )
    st.session_state["page"] = page

    if page == "Admin Login":
        admin_login_page()
    elif page == "Admin Dashboard":
        admin_dashboard_page(faqs)
    else:
        if embed_model is None or index is None:
            st.error("Models not loaded.")
            return
        chatbot_page(index, texts, embed_model, faqs)


if __name__ == "__main__":
    main()

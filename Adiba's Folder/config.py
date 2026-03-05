# config.py

USE_JETSON = False  # Set True when running on Jetson Orin Nano Super

if USE_JETSON:
    STT_ENGINE = "whisper_local"  # Jetson will use local whisper
    TTS_ENGINE = "edge_tts"       # Jetson-friendly TTS
else:
    STT_ENGINE = "openai_whisper"
    TTS_ENGINE = "gtts"

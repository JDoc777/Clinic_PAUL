# from openai import OpenAI
# client = OpenAI(api_key="")

# client = OpenAI()

# def gpt5_nano_process(command_text: str) -> str:
#     """
#     Sends text to GPT-5 nano and returns a short, speakable reply.
#     """
#     try:
#         resp = client.responses.create(
#             model="gpt-5-nano",   # ✅ GPT-5 Nano
#             input=[
#                 {
#                     "role": "system",
#                     "content": (
#                         "You are PAUL, a voice-controlled assistant. "
#                         "You only receive commands after the wake phrase 'Hey Paul'. "
#                         "Respond briefly and clearly in plain text."
#                     )
#                 },
#                 {"role": "user", "content": command_text},
#             ],
#         )

#         return (resp.output_text or "").strip()

#     except Exception as e:
#         print("GPT error:", e)
#         return "Sorry, something went wrong."

# from openai import OpenAI

# client = OpenAI(
#     api_key=""
# )

# r = client.responses.create(
#     model="gpt-5-nano",
#     input="Say hello"
# )

# print(r.output_text)

import os
import threading
import time
import numpy as np
import sounddevice as sd
import soundfile as sf
from piper import PiperVoice

def ensure_stereo(audio: np.ndarray) -> np.ndarray:
    """
    Ensure audio is stereo (N, 2). If mono (N,), duplicate channel.
    """
    if audio.ndim == 1:
        return np.column_stack([audio, audio])
    return audio

from dotenv import load_dotenv
from openai import OpenAI

# Load .env if present (recommended)
load_dotenv()

print("OPENAI_API_KEY FOUND:", bool(os.getenv("OPENAI_API_KEY")))

# --- Audio defaults ---
sd.default.device = (None, 1)       # USB speakers output device index = 1
sd.default.samplerate = 22050       # Piper default
sd.default.channels = 2

# --- Wake word config ---
WAKE_WORD = "hey paul"

def extract_command(text: str):
    """
    If text starts with 'hey paul', return the rest as the command.
    Otherwise return None.
    """
    if not text:
        return None
    t = text.strip().lower()

    if not t.startswith(WAKE_WORD):
        return None

    cmd = t[len(WAKE_WORD):].strip()
    return cmd if cmd else None


class VoiceEngine:
    def __init__(self, model_path, poll=0.05):
        self.voice = PiperVoice.load(model_path)

        self.current_text = None
        self.speaking = threading.Event()
        self.stop_event = threading.Event()
        self.interrupt_event = threading.Event()
        self.poll = poll

        self.lib_path = "/home/shaboiken/Documents/GitHub/Clinic_PAUL/Raspberry PI/Voice_IO/mp3_lib"
        self.mp3_map = {
            "michael": "michael.mp3",
            "dust": "dust.mp3",
            "kill": "kill.mp3",
            "sonic": "sonic.mp3",
            "yay": "yay.mp3",
        }

        self.audio_thread = threading.Thread(target=self._audio_loop, daemon=True)
        self.audio_thread.start()
        print(f"Voice engine initialized (poll={self.poll}s)")

    def speak(self, text: str):
        """Request speech. If busy, ignore."""
        if self.speaking.is_set():
            print("Already speaking. Ignoring this request.")
            return
        self.current_text = text

    def stop(self):
        """Interrupt current speech ASAP."""
        self.interrupt_event.set()
        try:
            sd.stop()
        except Exception:
            pass

    def _play_mp3(self, filepath):
        try:
            data, samplerate = sf.read(filepath, dtype="int16")
            data = ensure_stereo(data)
            sd.play(data, samplerate)
            while sd.get_stream().active:
                if self.interrupt_event.is_set():
                    sd.stop()
                    break
                time.sleep(0.01)
        except Exception as e:
            print("MP3 playback error:", e)

    def _audio_loop(self):
        while not self.stop_event.is_set():
            if self.current_text is not None and not self.speaking.is_set():
                self.speaking.set()
                self.interrupt_event.clear()

                text_to_speak = self.current_text.strip().lower()
                self.current_text = None

                try:
                    # CASE 1: MP3 library
                    if text_to_speak in self.mp3_map:
                        filename = self.mp3_map[text_to_speak]
                        filepath = os.path.join(self.lib_path, filename)

                        if os.path.exists(filepath):
                            print(f"Playing MP3: {filename}")
                            self._play_mp3(filepath)
                        else:
                            print(f"MP3 file not found: {filepath}")

                    # CASE 2: Piper TTS
                    else:
                        print("Using Piper TTS")
                        for chunk in self.voice.synthesize(text_to_speak):
                            if self.interrupt_event.is_set():
                                break
                            audio = np.frombuffer(chunk.audio_int16_bytes, dtype=np.int16)
                            audio = ensure_stereo(audio)
                            sd.play(audio, samplerate=chunk.sample_rate)

                            # Wait but allow interrupt
                            while sd.get_stream().active:
                                if self.interrupt_event.is_set():
                                    sd.stop()
                                    break
                                time.sleep(0.01)

                except Exception as e:
                    print("Audio error:", e)
                finally:
                    self.speaking.clear()
                    self.interrupt_event.clear()

            else:
                time.sleep(self.poll)

    def shutdown(self):
        self.stop_event.set()
        self.audio_thread.join()
        print("Shutdown complete")


# --- OpenAI client (GPT-5 nano) ---
def make_openai_client() -> OpenAI:
    # This will use OPENAI_API_KEY from environment / .env
    return OpenAI(
        # api_key=""
    )

client = make_openai_client()

def gpt5_nano_process(command_text: str) -> str:
    """
    Send the command to GPT-5 nano and get a short, speakable response.
    """
    try:
        resp = client.responses.create(
            model="gpt-5-nano",
            input=[
                {
                    "role": "system",
                    "content": (
                        "You are PAUL, a voice-controlled assistant. "
                        "Respond in 1-2 short sentences, plain text only. "
                        "No markdown."
                    )
                },
                {"role": "user", "content": command_text},
            ],
        )
        return (resp.output_text or "").strip() or "Sorry, I didn't catch that."
    except Exception as e:
        print("GPT error:", e)
        return "Sorry, I had a problem connecting."


def main():
    # TODO: change this to your real Piper model path
    MODEL_PATH = "/home/shaboiken/Documents/GitHub/Clinic_PAUL/Raspberry PI/Voice_IO/en_GB-northern_english_male-medium.onnx"

    # Quick key check (helps debugging)
    if not os.getenv("OPENAI_API_KEY"):
        print("ERROR: OPENAI_API_KEY not found. Add it to .env or export it.")
        return

    engine = VoiceEngine(MODEL_PATH)

    print("\nType messages as if they came from STT.")
    print("Examples:")
    print("  Hey Paul what time is it")
    print("  Hey Paul tell me a joke")
    print("  Hey Paul stop")
    print("Type 'exit' to quit.\n")

    while True:
        raw_text = input("You: ").strip()
        if raw_text.lower() in ("exit", "quit"):
            break

        cmd = extract_command(raw_text)

        # Ignore anything without wake word
        if cmd is None:
            continue

        # Interrupt command
        if cmd in ("stop", "cancel", "shut up", "silence"):
            print("Paul: (stopping)")
            engine.stop()
            continue

        # Call GPT in a thread so input loop stays responsive
        def worker(command: str):
            reply = gpt5_nano_process(command)
            print("Paul:", reply)
            engine.speak(reply)

        threading.Thread(target=worker, args=(cmd,), daemon=True).start()

    engine.shutdown()


if __name__ == "__main__":
    main()
#Working version W Q's and no mp3 / wav support. 
#Thursday's plan of action is to add a switch to select between mp3 and raw mode.
#Piping for mp3's
#Need to not change instanciation, have a different input that switches what mode you're in.
#I.e engine.wav and engine.speakraw to switch modes so THEREFORE speak() can be used for both
#in file mode, use the string of speak to search a preexisting mp3 library


import threading
import numpy as np
import sounddevice as sd
import soundfile as sf
from piper import PiperVoice
import time
import os

sd.default.device = (None, 2)  # USB speakers
sd.default.samplerate = 22050  # Piper default

class VoiceEngine:
    def __init__(self, model_path, poll=0.05):
        """
        model_path: path to Piper model
        poll_interval: seconds between loop checks for new speech
        """
        self.voice = PiperVoice.load(model_path)

        self.current_text = None        # Latest requested speech
        self.speaking = threading.Event()  # True when currently speaking
        self.stop_event = threading.Event() # Signal to stop the thread
        self.poll = poll  # Sleep between checks

            #NOTE: THIS PATH WILL NEED TO BE MODIFIED IN GENUINE PAUL
        self.lib_path = "/home/shaboiken/Documents/GitHub/Clinic_PAUL/Raspberry PI/Voice_IO/mp3_lib"
        self.mp3_map = {
            "michael": "michael.mp3",
            "dust": "dust.mp3",
            "kill": "kill.mp3",
            "sonic": "sonic.mp3",
            "yay": "yay.mp3",

        }
        # Start persistent thread
        self.audio_thread = threading.Thread(target=self._audio_loop, daemon=True)
        self.audio_thread.start()
        print(f"Voice engine initialized with poll_interval={self.poll}s")


    def speak(self, text):
        """Request speech. Ignored if engine is busy."""
        if self.speaking.is_set():
            print("Already speaking. Ignoring this request.")
            return
        self.current_text = text  # Set text for the loop to pick up

    def _play_mp3(self, filepath):
        try:
            data, samplerate = sf.read(filepath, dtype='int16')
            sd.play(data, samplerate)
            sd.wait()
        except Exception as e:
            print("MP3 playback error:", e)

    def _audio_loop(self):
        while not self.stop_event.is_set():
            if self.current_text is not None and not self.speaking.is_set():
                self.speaking.set()
                text_to_speak = self.current_text.strip().lower()
                self.current_text = None

                try:
                    # 🔹 CASE 1: Check MP3 library
                    if text_to_speak in self.mp3_map:
                        filename = self.mp3_map[text_to_speak]
                        filepath = os.path.join(self.lib_path, filename)

                        if os.path.exists(filepath):
                            print(f"Playing MP3: {filename}")
                            self._play_mp3(filepath)
                        else:
                            print(f"MP3 file not found: {filepath}")

                    # 🔹 CASE 2: Use Piper (raw TTS)
                    else:
                        print("Using Piper TTS")
                        for chunk in self.voice.synthesize(text_to_speak):
                            audio = np.frombuffer(chunk.audio_int16_bytes, dtype=np.int16)
                            sd.play(audio, samplerate=chunk.sample_rate)
                            sd.wait()

                except Exception as e:
                    print("Audio error:", e)

                finally:
                    self.speaking.clear()

            else:
                time.sleep(self.poll)

    def shutdown(self):
        self.stop_event.set()
        self.audio_thread.join()
        print("Shutdown complete")
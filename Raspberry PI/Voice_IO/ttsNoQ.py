#Working version W Q's and no mp3 / wav support. 
#Thursday's plan of action is to add a switch to select between mp3 and raw mode.
#Piping for mp3's
#Need to not change instanciation, have a different input that switches what mode you're in.
#I.e engine.wav and engine.speakraw to switch modes so THEREFORE speak() can be used for both
#in file mode, use the string of speak to search a preexisting mp3 library


import threading
import numpy as np
import sounddevice as sd
from piper import PiperVoice
import time

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

    def _audio_loop(self):
        """Persistent thread that checks for new speech requests."""
        while not self.stop_event.is_set():
            if self.current_text is not None and not self.speaking.is_set():
                self.speaking.set()
                text_to_speak = self.current_text
                self.current_text = None  # Reset before speaking

                try:
                    for chunk in self.voice.synthesize(text_to_speak):
                        audio = np.frombuffer(chunk.audio_int16_bytes, dtype=np.int16)
                        sd.play(audio, samplerate=chunk.sample_rate)
                        sd.wait()
                except Exception as e:
                    print("Audio error:", e)
                finally:
                    self.speaking.clear()
            else:
                time.sleep(self.poll)  # Use poll_interval from init

    def shutdown(self):
        self.stop_event.set()
        self.audio_thread.join()
        print("Shutdown complete")
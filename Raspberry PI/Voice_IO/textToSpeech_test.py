import threading
import queue
import numpy as np
import sounddevice as sd
from piper import PiperVoice


class VoiceEngine:
    def init(self, model_path):
        self.voice = PiperVoice.load(model_path)

        self.text_queue = queue.Queue()
        self.stop_event = threading.Event()

        self.audio_thread = threading.Thread(
            target=self._audio_worker,
            daemon=True
        )
        self.audio_thread.start()

        print("Voice engine initialized.")

    def speak(self, text):
        """Add text to speech queue (non-blocking)."""
        self.text_queue.put(text)

    def shutdown(self):
        """Clean shutdown."""
        self.stop_event.set()
        self.text_queue.put(None)
        self.audio_thread.join()
        print("Voice engine shutdown complete.")

    def _audio_worker(self):
        """Runs in background thread."""
        while not self.stop_event.is_set():
            text = self.text_queue.get()

            if text is None:
                break

            try:
                for chunk in self.voice.synthesize(text):
                    audio = np.frombuffer(
                        chunk.audio_int16_bytes,
                        dtype=np.int16
                    )

                    sd.play(audio, samplerate=chunk.sample_rate)
                    sd.wait()

            except Exception as e:
                print("Audio error:", e)

            self.text_queue.task_done()

            
from dotenv import load_dotenv
from openai import OpenAI
import os
import threading

load_dotenv()

print("OPENAI_API_KEY FOUND:", bool(os.getenv("OPENAI_API_KEY")))


class chatLib:
    def __init__(self):
        self.client = self.make_openai_client()
        self.working = False
        self.last_response = None
        self._lock = threading.Lock()

    # --- OpenAI client ---
    def make_openai_client(self) -> OpenAI:
        return OpenAI()

    # --- Internal worker (runs in thread) ---
    def _gpt5_worker(self, command_text: str, agree: bool):
        try:
            # Dynamic behavior instruction
            stance_instruction = (
                "Always agree with the user."
                if agree else
                "Always respectfully disagree with the user."
            )

            resp = self.client.responses.create(
                model="gpt-5-nano",
                input=[
                    {
                        "role": "system",
                        "content": (
                            "You are PAUL, a voice-controlled assistant. "
                            "Respond in 1-2 sentences max, plain text only. "
                            "No markdown. "
                            f"{stance_instruction}"
                        )
                    },
                    {"role": "user", "content": command_text},
                ],
            )

            result = (resp.output_text or "").strip() or "Sorry, I didn't catch that."

        except Exception as e:
            print("GPT error:", e)
            result = "Sorry, I had a problem connecting."

        # Thread-safe write
        with self._lock:
            self.last_response = result
            self.working = False

    # --- Public threaded call ---
    def gpt5_nano_process(self, command_text: str, agree: bool):
        if self.working:
            return  # prevent overlapping calls

        self.working = True
        thread = threading.Thread(
            target=self._gpt5_worker,
            args=(command_text, agree),
            daemon=True
        )
        thread.start()
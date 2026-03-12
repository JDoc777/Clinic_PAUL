import chat_worker
import threading
import time
from Discord_Listen import DiscordListener
import ttsNoQ_switch as noQ
from dotenv import load_dotenv
import os


WAKE_WORD1 = "hey paul"
WAKE_WORD2 = "play"


class stringController():
    def __init__(self):
        self.inStr = ""

    def location(self, inStr):
        self.inStr = inStr

        # run processing in a worker thread
        threading.Thread(target=self.process_command, daemon=True).start()
    def process_command(self):
        w, cmd = self.extract_command(self.inStr)

        if w == "hey paul":
            chat.gpt5_nano_process(cmd)

            while chat.working:
                time.sleep(0.01)

            # speak in its own thread
            threading.Thread(
                target=engine.speak,
                args=(chat.last_response,),
                daemon=True
            ).start()

        elif w == "play":
            threading.Thread(
                target=engine.speak,
                args=(cmd,),
                daemon=True
            ).start()

        else:
            # no wake word detected
            threading.Thread(
                target=engine.speak,
                #cmd = "womp",
                args=("I dont understand",),
                daemon=True
            ).start()
        

    def extract_command(self, text: str):

        if not text:
            return None, None

        t = text.strip().lower()
        wake_words = [WAKE_WORD1, WAKE_WORD2]

        for wake in wake_words:
            if t.startswith(wake):
                cmd = t[len(wake):].strip()
                return wake, cmd

        return None, None


# ---------------- MAIN ----------------

load_dotenv()

chat = chat_worker.chatLib()

stringCtrl = stringController()

engine = noQ.VoiceEngine(
    "/home/shaboiken/Documents/GitHub/Clinic_PAUL/Raspberry PI/Voice_IO/en_GB-northern_english_male-medium.onnx",
    0.05
)

TOKEN = os.getenv("DISCORD_BOT_KEY")

print("DISCORD_BOT_KEY loaded:", bool(TOKEN))

shared_state = {
    "latest_string": "",
    "new_flag": False,
    "lock": threading.Lock()
}

listener = DiscordListener(
    token=TOKEN,
    channel_name="talk-to-paul",
    shared_state=shared_state
)

listener.start()

while True:

    with shared_state["lock"]:
        if shared_state["new_flag"]:
            msg = shared_state["latest_string"]
            shared_state["new_flag"] = False

            print("god help", msg)

            # run message processing in thread
            stringCtrl.location(msg)

    time.sleep(0.05)
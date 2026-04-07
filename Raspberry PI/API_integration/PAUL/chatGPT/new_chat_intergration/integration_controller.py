from . import chat_worker
import threading
import time
from . Discord_Listen import DiscordListener
from . import ttsNoQ_switch as noQ
#from dotenv import load_dotenv
import os
from . dumb_dict import data_dict


WAKE_WORD1 = "hey paul"
WAKE_WORD2 = "play"


class stringController():
    def __init__(self, poll, grid):
        self.inStr = ""
        self.poll = poll
        self.grid = grid

    def location(self, inStr):
        self.inStr = inStr
        threading.Thread(target=self.process_command, daemon=True).start()

    def process_command(self):
        w, cmd = self.extract_command(self.inStr)
        start = time.perf_counter()
        if w == "hey paul":
            valid = False
            cmd_lower = cmd.lower()
            first_match = None

            for key in data_dict:
                if key in cmd_lower:
                    first_match = key
                    break

            matchFound = False
            if first_match:
                matchFound = True
                valid = True
                goal_x, goal_y = data_dict[first_match]

            chat.gpt5_nano_process(cmd, valid)

            while chat.working:
                time.sleep(self.poll)
            end = time.perf_counter()
            print(f"latency: {end - start:.6f} seconds")
            def speak_then_act():
                while engine.speaking.is_set():
                    time.sleep(0.01)

                engine.speak(chat.last_response)

                start_time = time.time()
                started = False

                while time.time() - start_time < 1.0:
                    if engine.speaking.is_set():
                        started = True
                        break
                    time.sleep(0.01)

                if started:
                    while engine.speaking.is_set():
                        time.sleep(0.01)

                if matchFound:
                    self.grid.set_goal(goal_x, goal_y)

            threading.Thread(target=speak_then_act, daemon=True).start()

        elif w == "play":
            def safe_speak():
                while engine.speaking.is_set():
                    time.sleep(0.01)
                engine.speak(cmd)

            threading.Thread(target=safe_speak, daemon=True).start()

        else:
            def safe_speak():
                while engine.speaking.is_set():
                    time.sleep(0.01)
                engine.speak("I dont understand")

            threading.Thread(target=safe_speak, daemon=True).start()

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


def discord_loop(shared_state, stringCtrl, poll):
    while True:
        with shared_state["lock"]:
            if shared_state["new_flag"]:
                msg = shared_state["latest_string"]
                shared_state["new_flag"] = False
                stringCtrl.location(msg)
        time.sleep(poll)


def run(poll, grid):
    #load_dotenv()

    global chat, stringCtrl, engine

    chat = chat_worker.chatLib()
    stringCtrl = stringController(poll, grid)

    engine = noQ.VoiceEngine(
        "/home/shaboiken/Documents/GitHub/Clinic_PAUL/Raspberry PI/API_integration/PAUL/en_GB-northern_english_male-medium.onnx",
        poll
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

    threading.Thread(
        target=discord_loop,
        args=(shared_state, stringCtrl, poll),
        daemon=True
    ).start()
import threading
import time
from Discord_Listen import DiscordListener
import ttsNoQ_switch as noQ
TOKEN = ""
#reeerecd
shared_state = { #this will be needed for top level paul
    "latest_string": "",
    "new_flag": False,
    "lock": threading.Lock()
}

engine = noQ.VoiceEngine(
    "/home/shaboiken/Documents/GitHub/Clinic_PAUL/Raspberry PI/Voice_IO/en_GB-northern_english_male-medium.onnx",
    0.05
) 

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
            engine.speak(msg)

    time.sleep(0.05)  # 50 milliseconds
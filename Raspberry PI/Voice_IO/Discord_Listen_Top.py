import threading
import time
from Discord_Listen import DiscordListener
#Remember to remove token before pushing. Git will not be happy with you if not
TOKEN = ""
#reeerecd
shared_state = { #this will be needed for top level paul
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

    time.sleep(0.05)  # 50 milliseconds
import threading
from Discord_Listen import DiscordListener
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


with shared_state["lock"]: #This line is the issue it seems
    if shared_state["new_flag"]:
        msg = shared_state["latest_string"]
        shared_state["new_flag"] = False

        print("god help", msg)

while True:
    print(shared_state["new_flag"])
    pass
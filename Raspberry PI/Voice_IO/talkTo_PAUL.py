import discord
import threading
from datetime import datetime, timezone


#remove token
#aa
TOKEN = ""
CHANNEL_NAME = "talk-to-paul"

intents = discord.Intents.default()
intents.message_content = True
intents.guilds = True
intents.messages = True

client = discord.Client(intents=intents)

startup_time = None

latest_message_string = ""
message_lock = threading.Lock()


# =============================
# Background Worker Thread
# =============================
def worker():
    print("Worker thread running...")
    while True:
        with message_lock:
            current = latest_message_string

        if current:
            print("Worker sees:", current)

        # Poll rate for worker
        threading.Event().wait(0.5)


thread = threading.Thread(target=worker, daemon=True)
thread.start()


# =============================
# Discord Events
# =============================

@client.event
async def on_ready():
    global startup_time
    startup_time = datetime.now(timezone.utc)

    print(f"Logged in as {client.user}")
    print("Bot is ready and listening...")


@client.event
async def on_message(message):
    global latest_message_string

    if message.author.bot:
        return

    if message.channel.name != CHANNEL_NAME:
        return

    if message.created_at <= startup_time:
        return

    # Thread-safe update
    with message_lock:
        latest_message_string = message.content


client.run(TOKEN)
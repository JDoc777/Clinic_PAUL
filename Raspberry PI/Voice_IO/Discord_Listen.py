import discord
import threading
from datetime import datetime, timezone


class DiscordListener:

    def __init__(self, token, channel_name, shared_state):
        self.token = token
        self.channel_name = channel_name
        self.shared_state = shared_state
        self.startup_time = None

        intents = discord.Intents.default()
        intents.message_content = True
        intents.guilds = True
        intents.messages = True

        self.client = discord.Client(intents=intents)

        self._register_events()

    def _register_events(self):

        @self.client.event
        async def on_ready():
            self.startup_time = datetime.now(timezone.utc)
            print(f"Logged in as {self.client.user}")

        @self.client.event
        async def on_message(message):

           # if message.author.bot:
           #     return

            if message.channel.name != self.channel_name:
                return

            if message.created_at <= self.startup_time:
                return

            # Thread-safe update
            with self.shared_state["lock"]:
                self.shared_state["latest_string"] = message.content
                self.shared_state["new_flag"] = True

    def start(self):
        threading.Thread(
            target=self.client.run,
            args=(self.token,),
            daemon=True
        ).start()
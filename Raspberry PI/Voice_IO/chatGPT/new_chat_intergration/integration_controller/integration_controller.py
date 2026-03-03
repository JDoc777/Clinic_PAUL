import chat_worker
chat = chat_worker.chatLib()
WAKE_WORD = "hey paul"

class stringController():
    def __init__ (self):
        self.inStr = ""

    def location(inStr):
        self.inStr = inStr
        if (inStr.lower() == "hey paul"):
            #send to chat
        
    def extract_command(text: str):
        """
        If text starts with 'hey paul', return the rest as the command.
        Otherwise return None.
        """
        if not text:
            return None
        t = text.strip().lower()

        if not t.startswith(WAKE_WORD):
            return None

        cmd = t[len(WAKE_WORD):].strip()
        return cmd if cmd else None
    



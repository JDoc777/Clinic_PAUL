import chat_worker
chat = chat_worker.chatLib()

chat.gpt5_nano_process("What's the weather?")

# Main thread keeps running here

while chat.working:
    pass  # or sleep briefly

print(chat.last_response)

chat.gpt5_nano_process("What color is an apple?")
while chat.working:
    pass  # or sleep briefly
print(chat.last_response)
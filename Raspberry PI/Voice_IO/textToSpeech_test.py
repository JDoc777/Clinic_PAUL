
# import pyttsx3
# import textToSpeech
# engine = pyttsx3.init()
# engine.say("Please print")
# engine.runAndWait()
# if __name__ == "__main__":

#     v = textToSpeech.Verbalize()
#     print("Before speech")
#     v.speakRawText("Hello from a background process")
#     print("Program continues immediately")
#     v.speakRawText("it might work")
#     print("Please print")

# import subprocess
# import threading
# import time
# from queue import Queue

# def speaker(queue):
#     while True:
#         item = queue.get()
#         if item is None:      # shutdown signal
#             break

#         text, delay = item
#         time.sleep(delay)
#         subprocess.run([
#             "espeak-ng",
#             "-v", "en-us+m1+sing", #Type of voice
#             "-a", "200", #Amplitude (0-200)
#             "-p", "55", #Pitch (0-99)
#             "-s", "140", #Speed (words per minute)
#             "-k", "40", #Word gap (0-99)
#             text
#         ], check=True)
#         queue.task_done()

# # Create the queue
# speech_queue = Queue()

# # Start ONE speaker thread
# speaker_thread = threading.Thread(target=speaker, args=(speech_queue,))
# speaker_thread.start()

# # Enqueue lines (POP order)
# lines = [
#     ("Hello I am PAUL the programmable autonomous utility lift. I am your personal wink wink assistant", 0.1),
    
# ]

# for line in lines:
#     speech_queue.put(line)

# # Wait for all speech to finish
# speech_queue.join()

# # Stop the speaker thread
# speech_queue.put(None)
# speaker_thread.join()

# print("Done Speaking")

import subprocess
import threading
import time
import tempfile
import os
from queue import Queue

PIPER_MODEL = "/home/justina/piper-voices/en_US-jenny-medium.onnx"  # <-- change this

def speak_piper(text: str):
    # Make a temp wav file
    fd, wav_path = tempfile.mkstemp(suffix=".wav")
    os.close(fd)

    try:
        # Generate speech -> wav (piper reads text from stdin)
        subprocess.run(
            ["piper", "--model", PIPER_MODEL, "--output_file", wav_path],
            input=text,
            text=True,
            check=True
        )

        # Play wav (ALSA)
        subprocess.run(["aplay", wav_path], check=True)

    finally:
        # Clean up
        if os.path.exists(wav_path):
            os.remove(wav_path)

def speaker(queue: Queue):
    while True:
        item = queue.get()
        if item is None:
            queue.task_done()
            break

        text, delay = item
        time.sleep(delay)

        speak_piper(text)

        queue.task_done()

speech_queue = Queue()

speaker_thread = threading.Thread(target=speaker, args=(speech_queue,), daemon=True)
speaker_thread.start()

lines = [
    ("Hello I am PAUL the programmable autonomous utility lift. I am your personal assistant.", 0.1),
]

for line in lines:
    speech_queue.put(line)

speech_queue.join()

speech_queue.put(None)
speech_queue.join()

print("Done Speaking")


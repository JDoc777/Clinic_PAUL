from multiprocessing import Process #uses multiprocessing bc threads suck
import pyttsx3

def speak_process(text):
    engine = pyttsx3.init()
    engine.say(text)
    engine.runAndWait()

class Verbalize:
    def speakRawText(self, text):
        # Start a separate process to speak
        p = Process(target=speak_process, args=(text,))
        p.start()





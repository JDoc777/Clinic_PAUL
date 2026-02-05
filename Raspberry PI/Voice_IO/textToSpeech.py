import threading #does not currently use threading bc its a nightmare
import pyttsx3 

class Verbalize: 
    def __init__(self): 
        self.engine = pyttsx3.init() 
        print("Speech init done.") 

    def speakRawText(self, input): 
        self.engine.say(input)
        self.engine.runAndWait() 
        print("Speaking done.")
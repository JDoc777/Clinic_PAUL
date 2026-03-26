import ttsNoQ_switch as noQ
import time


#This is a file to check if it functions or not
engine = noQ.VoiceEngine(
    "/home/shaboiken/Documents/GitHub/Clinic_PAUL/Raspberry PI/Voice_IO/en_GB-northern_english_male-medium.onnx",
    0.05
) 
gggd = "are we ready?"
#engine.speak(gggd)
time.sleep(1)
# engine.speak("This should not work.")
# time.sleep(5)
# engine.speak("This should work")
time.sleep(5)
engine.speak("sonic")
time.sleep(3)
engine.speak("yay")
time.sleep(8)
engine.speak("kill")
time.sleep(15)
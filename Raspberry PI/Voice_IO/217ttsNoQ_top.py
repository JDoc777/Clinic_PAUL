import ttsNoQ as noQ
import time


#This is a file to check if it functions or not
engine = noQ.VoiceEngine(
    "/home/shaboiken/Documents/GitHub/Clinic_PAUL/Raspberry PI/Voice_IO/en_GB-northern_english_male-medium.onnx",
    0.05
) 
gggd = "hi justin I am a real boy."
engine.speak(gggd)
time.sleep(1)
engine.speak("This should not work.")
time.sleep(5)
engine.speak("This should work")
time.sleep(5)
import textToSpeech as ourTTS

#initialize voice engine (will need to be done top level)
engine = ourTTS.VoiceEngine(
    "/home/shaboiken/Documents/GitHub/Clinic_PAUL/Raspberry PI/Voice_IO/en_GB-northern_english_male-medium.onnx"
) # This path currently points locally to the PIPER library map. Will need to be changed.

print("When does this print? #1")

engine.speak("Let's Fire photon torpedos.")
#engine.text_queue.join() #waiting for playback -- this works holy shit DO NOT REMOVE

print("When does this print? #2")

engine.speak("Sharknado")

print("When does this print? #3")
engine.text_queue.join() #waiting for playback -- this works holy shit DO NOT REMOVE

print("When does this print? #4")



import textToSpeech
if __name__ == "__main__":

    v = textToSpeech.Verbalize()
    print("Before speech")
    v.speakRawText("Hello from a background process")
    print("Program continues immediately")
    v.speakRawText("it might work")
    print("Please print")
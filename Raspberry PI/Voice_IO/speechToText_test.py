import speech_recognition as sr
r = sr.Recognizer()

menu_active = False

while True:
    try:
        with sr.Microphone() as source:
            print("Listening...")
            
            r.adjust_for_ambient_noise(source, duration=0.2)
            audio = r.listen(source)
            text = r.recognize_google(audio) 
            text = text.lower()
            print("You said:", text)
            
            
            if menu_active:
                match text:
                    case "turn around":
                        print("Paul Turns Around")
                        menu_active = False
                    case "pick":
                        print("Paul Picks")
                        menu_active = False
                    case "move forward":
                        print("Paul Moves Forward")
                        menu_active = False
                    case "exit menu":
                        menu_active = False
                        print("Menu exited")
                    case _:
                        print("Unknown command. Menu: turn around, pick, move forward, exit menu")
            else:
                if "exit" in text:
                    print("Exiting program...")
                    break
                if "quit" in text:
                    print("Exiting program...")
                    break
                if "paul" in text:
                    menu_active = True
                    print("Menu activated. Say: turn around, pick, move forward, exit menu")
                    break

    except sr.RequestError as e:
        print("Could not request results; {0}".format(e))

    except sr.UnknownValueError:
        print("Could not understand audio")

    except KeyboardInterrupt:
        print("Program terminated by user")
        break
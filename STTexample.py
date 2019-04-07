import speech_recognition as sr


r = sr.Recognizer()
with sr.Microphone() as source:
    print("Say Something!")
    audio = r.listen(source)
    
    try:
        print("Google thinks you said: " + r.recognize_google(audio))
    except sr.UnknownValueError:
        print ("UVE")
    except sr.RequestError as e:
        print("Request Error; {0}".format(e))

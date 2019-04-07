# import sys
# import readline
import speech_recognition as sr

from arm import Arm

dextera = Arm()
while True:
    r = sr.Recognizer()
    with sr.Microphone() as source:
        print("Say Something!")
        audio = r.listen(source)

    try:
        command = r.recognize_google(audio)
        dextera.parse_text(command)
        print("Google thinks you said: " + command)
    except sr.UnknownValueError:
        print ("UVE")
    except sr.RequestError as e:
        print("Request Error; {0}".format(e))

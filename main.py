import GoogleCloudSpeech as gc
import pyaudio
#from arm import Arm

name = 'Graham'
key_words_arr = [name, 'move up', 'move down', 'inches', 'degrees', 'rotate in', 'rotate out', 'pan up', 'pan down']
language_code = 'en-US'  # a BCP-47 language tag

client = gc.speech.SpeechClient()
config = gc.types.RecognitionConfig(
    encoding=gc.enums.RecognitionConfig.AudioEncoding.LINEAR16,
    sample_rate_hertz=gc.RATE,
    language_code=language_code,
    model='command_and_search',
    speech_contexts=[gc.speech.types.SpeechContext(
        phrases=key_words_arr)])
streaming_config = gc.types.StreamingRecognitionConfig(
    config=config,
    single_utterance=True,
    interim_results=True)

#dextera = Arm()

while True:
    try:
        with gc.MicrophoneStream(gc.RATE, gc.CHUNK) as stream:
            audio_generator = stream.generator()
            requests = (gc.types.StreamingRecognizeRequest(audio_content=content)
                        for content in audio_generator)

            responses = client.streaming_recognize(streaming_config, requests, timeout=15)
            # Now, put the transcription responses to use.
            transcript = gc.listen_print_loop(responses)
 #           dextera.parse_text(transcript)
            if transcript == '':
                print ('NO VALID COMMAND GIVEN')
            else:
                print("Google thinks you said: " + transcript)
            if transcript == 'stop listening ' + name:
                break
    except Exception as exception:
        print('Exception handle: Exceed max stream of 65 seconds')


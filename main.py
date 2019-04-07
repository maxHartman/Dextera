import GoogleCloudSpeech as gc

#from arm import Arm

key_words_arr = ['move up', 'move down', 'inches', 'degrees', 'rotate in', 'rotate out', 'pan up', 'Jimmy', 'pan down']
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
    with gc.MicrophoneStream(gc.RATE, gc.CHUNK) as stream:
        audio_generator = stream.generator()
        requests = (gc.types.StreamingRecognizeRequest(audio_content=content)
                    for content in audio_generator)

        responses = client.streaming_recognize(streaming_config, requests)

        # Now, put the transcription responses to use.
        transcript = gc.listen_print_loop(responses)
        #dextera.parse_text(transcript)
        if transcript == '':
            print ('NO VALID COMMAND GIVEN')
        else:
            print("Google thinks you said: " + transcript)
        if transcript == 'stop listening Jimmy':
            break

from gtts import gTTS
import os

language = 'id'


text = "halo."
speech = gTTS(text=text, lang=language, slow=False)
speech.save("halo.mp3")

text1 = "1. "
speech1 = gTTS(text=text1, lang=language, slow=False)
speech1.save("1.mp3")



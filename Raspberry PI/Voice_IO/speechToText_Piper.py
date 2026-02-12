import wave
import subprocess
from piper import PiperVoice

def play_wav(path):
    subprocess.run(["aplay", path], check=True)

voice = PiperVoice.load("/home/shaboiken/Documents/GitHub/Clinic_PAUL/Raspberry PI/Voice_IO/en_GB-northern_english_male-medium.onnx")

# TODO: Does not function

# syn_config = SynthesisConfig(
#     volume = 0.5,
#     length_scale = 1.0,
#     noise_scale = 1.0,
#     noise_w_scale = 1.0,
#     normalize_audio = false,
# )

#NOTE: Functions
# with wave.open("test.wav", "wb") as wav_file:
#     voice.synthesize_wav("Hello! I am PAUL. I am gobsmacked to meet you.", wav_file)

#TODO: does not work
# play_wav("test.wav")

# for chunk in voice.synthesize("I am an alcoholic"):
#     set_audio_format(chunk.sample_rate, chunk.sample_width, chunk.sample_channels)
#     write_raw_data(chunk.audio_int16_bytes)

piper = subprocess.Popen(
    [
        "./piper",
        "--model", 
    ]
)
import subprocess

MODEL_PATH = "/home/shaboiken/Documents/GitHub/Clinic_PAUL/Raspberry PI/Voice_IO/en_GB-northern_english_male-medium.onnx"
SAMPLE_RATE = "22050"  # Check your .onnx.json to confirm

def stream_piper(text):
    # Start Piper process
    piper = subprocess.Popen(
        [
            "piper",
            "--model", MODEL_PATH,
            "--output-raw",
            "--sentence_silence", "0"
        ],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE
    )

    # Start ALSA playback
    aplay = subprocess.Popen(
        [
            "aplay",
            "-r", SAMPLE_RATE,
            "-f", "S16_LE",
            "-t", "raw",
            "-"
        ],
        stdin=piper.stdout
    )

    # Send text to Piper
    piper.stdin.write((text + "\n").encode("utf-8"))
    piper.stdin.flush()
    piper.stdin.close()

    # Wait for playback to finish
    aplay.wait()


if __name__ == "__main__":
    stream_piper("Hello! I am PAUL. I am gobsmacked to meet you.")
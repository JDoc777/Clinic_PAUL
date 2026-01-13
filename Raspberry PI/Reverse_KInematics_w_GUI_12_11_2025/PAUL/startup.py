# startup.py
import time
import testingUART
import Payload
import Melodies

def warmup_sequence(shared_data):
    Payload.set_text(shared_data, "PAUL            BOOTING...")
    time.sleep(1)

    Melodies.play("pokemon_center")
    Payload.set_text(shared_data, "Initializing    UART...")
    time.sleep(1)

    Payload.set_text(shared_data, "Starting        Sensors...")
    time.sleep(1)

    Payload.set_text(shared_data, "GO TIME!")
    Melodies.play("success")

    Payload.set_flags_byte(shared_data, 0b00101111)


def startup():
    shared_data, running_event, ser = testingUART.start_reader(
        start_writer=True,
        do_handshake=True
    )

    # Arduino reboots when serial port opens → give it time
    time.sleep(2.0)

    Melodies.init(shared_data)

    warmup_sequence(shared_data)

    return shared_data, running_event, ser

import threading
import queue
import time
from Payload import buzzer

# -----------------------------
# NOTE FREQUENCIES
# -----------------------------
NOTES = {
    "C7": 2090, "CS7": 2210, "D7": 2350, "DS7": 2490,
    "E7": 2640, "F7": 2800, "FS7": 2960, "G7": 3140,
    "GS7": 3320, "A7": 3520, "AS7": 3730, "B7": 3950,
    "REST": 0
}

# -----------------------------
# MELODIES (with default tempo)
# -----------------------------
MELODIES = {

    # ---------------------------------------------------
    # Mario Short Theme
    # ---------------------------------------------------
    "mario": {
        "tempo": 0.35,
        "notes": [
            ("E7", 8), ("E7", 8), ("REST", 8), ("E7", 8),
            ("REST", 8), ("C7", 8), ("E7", 8), ("G7", 4),
            ("REST", 4), ("G7", 4), ("REST", 4),

            ("C7", 4), ("REST", 8), ("G7", 8), ("REST", 4),
            ("E7", 4), ("REST", 4), ("A7", 4), ("REST", 8),
            ("B7", 8), ("REST", 8), ("AS7", 8), ("A7", 8),
            ("G7", 8), ("E7", 8), ("G7", 8), ("A7", 4),

            ("F7", 8), ("F7", 8), ("F7", 8), ("D7", 8),
            ("F7", 8), ("A7", 8), ("C7", 4)
        ]
    },

    # ---------------------------------------------------
    # Full Mario Theme
    # ---------------------------------------------------
    "mario_full": {
        "tempo": 0.30,
        "notes": [
            ("E7", 8), ("E7", 8), ("REST", 8), ("E7", 8), ("REST", 8),
            ("C7", 8), ("E7", 8), ("G7", 4), ("REST", 4),
            ("G7", 8), ("REST", 4),

            ("C7", 4), ("G7", 8), ("REST", 4), ("E7", 4),
            ("A7", 4), ("B7", 4), ("AS7", 8), ("A7", 4),
            ("G7", 8), ("E7", 8), ("G7", 8), ("A7", 4), ("F7", 8), ("G7", 8),

            ("REST", 8), ("E7", 4), ("C7", 8), ("D7", 8), ("B7", 4),

            ("C7", 4), ("G7", 8), ("REST", 4), ("E7", 4),
            ("A7", 4), ("B7", 4), ("AS7", 8), ("A7", 4),
            ("G7", 8), ("E7", 8), ("G7", 8), ("A7", 4), ("F7", 8), ("G7", 8),

            ("REST", 8), ("E7", 4), ("C7", 8), ("D7", 8), ("B7", 4),

            ("REST", 4), ("G7", 8), ("FS7", 8), ("F7", 8), ("DS7", 8), ("E7", 4),
            ("REST", 8), ("GS7", 8), ("A7", 8), ("C7", 8),
            ("REST", 8), ("A7", 8), ("C7", 8), ("D7", 8),

            ("F7", 8), ("A7", 8), ("C7", 4)
        ]
    },

    # ---------------------------------------------------
    # Star Wars Theme
    # ---------------------------------------------------
    "starwars": {
        "tempo": 0.50,
        "notes": [
            ("AS7", 8), ("AS7", 8), ("AS7", 8),
            ("F7", 2), ("C7", 2),
            ("AS7", 8), ("A7", 8), ("G7", 8), ("F7", 2), ("C7", 4),

            ("AS7", 8), ("A7", 8), ("G7", 8), ("F7", 2), ("C7", 4),
            ("AS7", 8), ("A7", 8), ("AS7", 8), ("G7", 2),
            ("C7", 8), ("C7", 8), ("C7", 8),

            ("F7", 2), ("C7", 2),
            ("AS7", 8), ("A7", 8), ("G7", 8), ("F7", 2), ("C7", 4),

            ("AS7", 8), ("A7", 8), ("G7", 8), ("F7", 2), ("C7", 4),

            ("AS7", 8), ("A7", 8), ("AS7", 8), ("G7", 2),
            ("C7", 8), ("C7", 16),

            ("D7", 4), ("D7", 8), ("AS7", 8), ("A7", 8), ("G7", 8), ("F7", 8),

            ("F7", 8), ("G7", 8), ("A7", 8), ("G7", 4), ("D7", 8), ("E7", 4),
            ("C7", 8), ("C7", 16),

            ("D7", 4), ("D7", 8), ("AS7", 8), ("A7", 8), ("G7", 8), ("F7", 8),

            ("C7", 4), ("G7", 8), ("G7", 16), ("REST", 2), ("C7", 8)
        ]
    },

    # ---------------------------------------------------
    # Happy Birthday (from your file)
    # ---------------------------------------------------
    "birthday": {
        "tempo": 0.50,
        "notes": [
            ("C7", 4), ("C7", 8),
            ("D7", 4), ("C7", 4), ("F7", 4),
            ("E7", 2),

            ("C7", 4), ("C7", 8),
            ("D7", 4), ("C7", 4), ("G7", 4),
            ("F7", 2),

            ("C7", 4), ("C7", 8),
            ("C7", 4), ("A7", 4), ("F7", 4),
            ("E7", 4), ("D7", 4),

            ("AS7", 4), ("AS7", 8),
            ("A7", 4), ("F7", 4), ("G7", 4),
            ("F7", 2)
        ]
    },

    # ---------------------------------------------------
    # Small sound effects
    # ---------------------------------------------------
    "coin":      {"tempo": 0.15, "notes": [("E7",16),("G7",16)]},
    "jump":      {"tempo": 0.15, "notes": [("C7",16),("E7",16),("G7",8)]},
    "1up":       {"tempo": 0.12, "notes": [("E7",16),("G7",16),("E7",16),("C7",16),("D7",16),("G7",16)]},

    "game_over": {
        "tempo": 0.40,
        "notes": [
            ("C7", 8), ("G7", 8), ("E7", 8), ("A7", 8), ("B7", 8),
            ("A7", 8), ("GS7", 8), ("AS7", 8), ("G7", 8)
        ]
    },

    "pipe":         {"tempo": 0.25, "notes": [("E7",8),("C7",8),("G7",8),("REST",8),("G7",8),("C7",8)]},
    "success":      {"tempo": 0.10, "notes": [("C7",16),("E7",16),("G7",16)]},
    "error":        {"tempo": 0.20, "notes": [("C7",4),("REST",8),("C7",4)]},
    "alert":        {"tempo": 0.20, "notes": [("A7",8),("REST",8),("A7",8),("REST",8),("A7",8)]},

    "power_up":     {"tempo": 0.12, "notes": [("C7",16),("D7",16),("E7",16),("F7",16),("G7",8)]},
    "power_down":   {"tempo": 0.12, "notes": [("G7",16),("F7",16),("E7",16),("D7",16),("C7",8)]},

    "notification": {"tempo": 0.12, "notes": [("C7",16),("REST",16),("G7",8)]},
    "siren":        {"tempo": 0.20, "notes": [("C7",8),("E7",8),("C7",8),("E7",8),("C7",8),("E7",8)]},
    "click":        {"tempo": 0.05, "notes": [("C7",32),("REST",32)]},

    "low_battery":  {"tempo": 0.40, "notes": [
        ("G7",16),("F7",16),("E7",16),("D7",16),("C7",8)
    ]},
    "pokemon_center": {
        "tempo": 0.35,
        "notes": [
            ("C7", 8), ("E7", 8), ("G7", 8), ("C7", 4),
            ("B7", 8), ("G7", 8), ("E7", 8), ("C7", 4),
            ("A7", 8), ("CS7", 8), ("E7", 8), ("A7", 4),
            ("G7", 8), ("E7", 8), ("CS7", 8), ("A7", 4)
        ]
    },

    "pokemon_levelup": {
        "tempo": 0.20,
        "notes": [
            ("C7", 16), ("D7", 16), ("E7", 16), ("G7", 8), 
            ("C7", 4)
        ]
    },

    "pokemon_heal": {
        "tempo": 0.22,
        "notes": [
            ("C7", 16), ("E7", 16), ("G7", 16),
            ("C7", 16), ("E7", 16), ("G7", 8),
            ("C7", 4)
        ]
    },

    "pokemon_battle_intro": {
        "tempo": 0.25,
        "notes": [
            ("E7", 8), ("F7", 8), ("G7", 4),
            ("E7", 8), ("C7", 4),
            ("G7", 2)
        ]
    },

    "pokemon_item": {
        "tempo": 0.18,
        "notes": [
            ("A7", 16), ("C7", 16), ("E7", 8)
        ]
    },

    "karma_chameleon": {
    "tempo": 0.68,
    "notes": [
        # "KAR-ma KAR-ma KAR-ma KAR-ma"
        ("G7", 8), ("A7", 8), ("G7", 8), ("E7", 8),
        ("G7", 8), ("A7", 8), ("G7", 8), ("E7", 8),

        # "KAR-ma CHAME-leon"
        ("G7", 8), ("A7", 8), ("B7", 4),
        ("A7", 8), ("G7", 8), ("E7", 4),

        # "You come and go"
        ("G7", 4), ("A7", 8), ("G7", 8),
        ("E7", 4), ("G7", 4),

        # "You come and go"
        ("A7", 4), ("B7", 4), ("A7", 8), ("G7", 8),

        # end tag (soft)
        ("E7", 4), ("REST", 4)
    ]
    }


}


# -------------------------------------------------------
# MELODY MANAGER
# -------------------------------------------------------
class MelodyManager:
    def __init__(self, shared_data):
        self.shared_data = shared_data
        self.q = queue.Queue()
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.currently_playing = False

    def start(self):
        self.thread.start()

    def request(self, name, tempo_override=None):
        """Queue a melody by name, optional tempo override."""
        if name not in MELODIES:
            print(f"[Melodies] Unknown melody: {name}")
            return

        default_tempo = MELODIES[name]["tempo"]
        tempo = tempo_override if tempo_override is not None else default_tempo

        self.q.put((name, tempo))

    def play_once(self, melody_name, tempo):
        data = MELODIES[melody_name]
        notes = data["notes"]

        for note, beats in notes:
            freq = NOTES[note]
            duration = tempo * (4 / beats)

            if freq == 0:
                ip = 0; dp = 0
            else:
                ip = freq // 1000
                dp = (freq % 1000) // 10

            buzzer(self.shared_data, ip, dp)
            time.sleep(duration * 0.80)

            buzzer(self.shared_data, 0, 0)
            time.sleep(duration * 0.20)

        buzzer(self.shared_data, 0, 0)

    def loop(self):
        while True:
            name, tempo = self.q.get()
            self.currently_playing = True
            try:
                self.play_once(name, tempo)
            except Exception as e:
                print("[MelodyManager] Error:", e)
            self.currently_playing = False


# -------------------------------------------------------
# GLOBAL SINGLETON ACCESS
# -------------------------------------------------------
_manager = None

def init(shared_data):
    """Call from run_all.py after shared_data exists."""
    global _manager
    _manager = MelodyManager(shared_data)
    _manager.start()

def play(name, tempo=None):
    """Public function to request a melody."""
    if _manager:
        _manager.request(name, tempo_override=tempo)
    else:
        print("[Melodies] ERROR: MelodyManager not initialized")

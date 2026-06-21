import serial
import threading
import pygame
import time

# ==========================================
# CONFIGURATION
# ==========================================
COM_PORT = 'COM7'   #  XIAO'S COM PORT
BAUD_RATE = 500000  # Matches the ultra-fast hub baud rate

VOLUME_MAP = {1: 0.20, 2: 0.40, 3: 0.60, 4: 0.80, 5: 0.90, 6: 1.00}
DRUM_MAP = {1: "crash", 2: "snare", 3: "tom1", 4: "tom2", 5: "ride", 6: "hihat", 7: "floor_tom"}

# ==========================================
# PYGAME SOUND GENERATION 
# ==========================================
pygame.mixer.pre_init(frequency=44100, size=-16, channels=2, buffer=256)
pygame.init()
pygame.mixer.set_num_channels(32) 

drum_samples = {}
for drum_id, name in DRUM_MAP.items():
    try:
        drum_samples[drum_id] = pygame.mixer.Sound(f"{name}.wav")
    except FileNotFoundError:
        print(f"Warning: '{name}.wav' not found.")
        drum_samples[drum_id] = pygame.mixer.Sound(buffer=b'\x00\x7F' * 1000)

print(f"Audio Engine Ready. Listening to Receiver Hub on {COM_PORT} at {BAUD_RATE} baud...")

# ==========================================
# HIGH-SPEED SERIAL LISTENER
# ==========================================
def serial_listener():
    try:
        # 🚨 Removed timeout=0.1. Natively blocks until a full line is received.
        ser = serial.Serial(COM_PORT, BAUD_RATE)
    except Exception as e:
        print(f"Failed to open {COM_PORT}: {e}")
        return

    while True:
        try:
            # readline() will wait forever until it gets a clean \n character
            # No more chopped or fragmented packets!
            raw_line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if raw_line.startswith("H,"):
                parts = raw_line.split(",")
                if len(parts) == 4:
                    stick_id = int(parts[1])
                    drum_id = int(parts[2])
                    velocity_zone = int(parts[3])

                    if drum_id in drum_samples and velocity_zone in VOLUME_MAP:
                        target_channel = pygame.mixer.find_channel()
                        if target_channel:
                            volume = VOLUME_MAP[velocity_zone]
                            
                            # Stereo Panning
                            if stick_id == 0: 
                                target_channel.set_volume(volume * 0.9, volume * 0.4)
                            else: 
                                target_channel.set_volume(volume * 0.4, volume * 0.9)

                            target_channel.play(drum_samples[drum_id])
                            side_str = "LEFT" if stick_id == 0 else "RIGHT"
                            print(f"[{side_str}] {DRUM_MAP[drum_id].upper()} | Vol: {int(volume*100)}%")
        except Exception as e:
            print(f"Serial Error: {e}")
            time.sleep(1)

            
if __name__ == "__main__":
    net_thread = threading.Thread(target=serial_listener, daemon=True)
    net_thread.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nClosing Air Drum System.")
        pygame.quit()
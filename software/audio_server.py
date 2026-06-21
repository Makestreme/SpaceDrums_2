import socket
import threading
import pygame
import time

# ==========================================
# CONFIGURATION & AUDIO MAPS
# ==========================================
UDP_PORT = 5005

# Map Velocity Zones (1-6) to Pygame channel volumes (0.0 to 1.0)
VOLUME_MAP = {
    1: 0.20,  # Ghost Note
    2: 0.40,  # Soft
    3: 0.60,  # Medium-Soft
    4: 0.80,  # Medium-Hard
    5: 0.90,  # Hard
    6: 1.00   # Smash Accent
}

# Map IDs from ESP32 Spatial Grid
DRUM_MAP = {
    1: "crash",
    2: "snare",
    3: "tom1",
    4: "tom2",
    5: "ride",
    6: "hihat",
    7: "floor_tom"
}

# ==========================================
# PYGAME SOUND GENERATION OPTIMIZATION
# ==========================================
# CRITICAL: Buffer size set to 256 samples forces sub-10ms hardware engine latency
pygame.mixer.pre_init(frequency=44100, size=-16, channels=2, buffer=256)
pygame.init()
pygame.mixer.set_num_channels(32) # Allow 32 overlapping polyphonic sounds

# Load Audio Files into Memory Cache
drum_samples = {}
for drum_id, name in DRUM_MAP.items():
    try:
        # Expects snare.wav, crash.wav, etc. in same directory
        drum_samples[drum_id] = pygame.mixer.Sound(f"{name}.wav")
    except FileNotFoundError:
        print(f"Warning: Audio asset '{name}.wav' not found. Using placeholder synthesizer click.")
        drum_samples[drum_id] = pygame.mixer.Sound(buffer=b'\x00\x7F' * 1000)

print("Audio Engine Ready and Cached. Awaiting Wireless Sticks...")

# ==========================================
# MULTITHREADED UDP NETWORK CONTROLLER
# ==========================================
def network_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', UDP_PORT))

    active_sticks = set()

    while True:
        data, addr = sock.recvfrom(1024)
        
        # Handle Auto-Discovery Handshake Protocol
        if b"AIRDRUM_" in data and b"_SEEK" in data:
            stick_identity = data.decode('utf-8')
            if addr[0] not in active_sticks:
                print(f"Detected New Hardware Stick: {stick_identity} at IP {addr[0]}")
                active_sticks.add(addr[0])
            
            # Fire acknowledgment packet back to stick to establish connection lock
            sock.sendto(b"AIRDRUM_HOST_ACK", addr)
            continue

        # Parse fast 3-byte hit payloads
        if len(data) == 3:
            stick_id = data[0]
            drum_id = data[1]
            velocity_zone = data[2]

            # Security verification on indexes
            if drum_id in drum_samples and velocity_zone in VOLUME_MAP:
                # Find an open audio hardware channel dynamically
                target_channel = pygame.mixer.find_channel()
                if target_channel:
                    volume = VOLUME_MAP[velocity_zone]
                    
                    # Optional spatial separation: pan left stick left, right stick right
                    if stick_id == 0:   # Left Stick
                        target_channel.set_volume(volume * 0.9, volume * 0.4)
                    else:               # Right Stick
                        target_channel.set_volume(volume * 0.4, volume * 0.9)

                    # Trigger playback instantly
                    target_channel.play(drum_samples[drum_id])
                    
                    side_str = "LEFT" if stick_id == 0 else "RIGHT"
                    print(f"[{side_str} STICK] Hit {DRUM_MAP[drum_id].upper()} | Zone {velocity_zone} (Vol: {int(volume*100)}%)")

if __name__ == "__main__":
    net_thread = threading.Thread(target=network_listener, daemon=True)
    net_thread.start()

    # Keep main script thread alive gracefully
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nClosing Air Drum Host Receiver System.")
        pygame.quit()
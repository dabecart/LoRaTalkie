import serial
import wave
import time

# Configuration
SERIAL_PORT = 'COM6'  # Replace with your actual serial port (e.g., '/dev/ttyUSB0' on Linux)
BAUD_RATE = 460800    # Match your device's baud rate
SAMPLE_RATE = 3200*2  # Samples per second
BITS_PER_SAMPLE = 8   # 8-bit audio
CHANNELS = 1          # Mono
DURATION = 10         # Recording duration in seconds
OUTPUT_WAV = 'output.wav'

def record_audio_from_serial():
    # Calculate the number of bytes to read
    total_samples = SAMPLE_RATE * DURATION

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Recording audio for {DURATION} seconds...")
            audio_data = bytearray()

            while len(audio_data) < total_samples:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)
                    audio_data.extend(data)

            print(f"Captured {len(audio_data)} bytes.")



            # Save to .wav file
            with wave.open(OUTPUT_WAV, 'wb') as wf:
                wf.setnchannels(CHANNELS)
                wf.setsampwidth(BITS_PER_SAMPLE // 8)
                wf.setframerate(SAMPLE_RATE)
                wf.writeframes(audio_data[:total_samples])

            print(f"Audio saved to '{OUTPUT_WAV}'.")

    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == '__main__':
    record_audio_from_serial()
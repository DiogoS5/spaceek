import serial
import csv
import time
import numpy as np

# Adjust the serial port as needed
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
CSV_FILE = 'mlx90641_frames.csv'

def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Reading from {SERIAL_PORT} and saving to {CSV_FILE}...")

    with open(CSV_FILE, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)

        try:
            while True:
                line = ser.readline().decode('ascii', errors='ignore').strip()
                line = line.replace('\\n', '').rstrip(',')

                parts = line.split(',')

                if len(parts) == 192:
                    try:
                        temps = [float(x) for x in parts]
                        writer.writerow(temps)
                        print("Frame saved.")
                    except ValueError:
                        print("Skipping invalid numeric data.")
                else:
                    print(f"Invalid frame size: {len(parts)} elements")

        except KeyboardInterrupt:
            print("Stopped by user.")
        finally:
            ser.close()

if __name__ == "__main__":
    main()

import time

import board
import busio
import getmac
from adafruit_rfm9x import RFM9x
from digitalio import DigitalInOut

# Unique id
unique_id = "Host" + getmac.get_mac_address(interface="eth0").replace(":", "")[-6:].upper()  # Last 3 bytes as unique ID
print(f"Raspberry Pi Unique ID: {unique_id}\n")

# LoRa Configuration
RADIO_FREQ_MHZ = 915.0  # Frequency for LoRa 

# Setup SPI bus and LoRa module
spi = busio.SPI(clock=board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = DigitalInOut(board.CE1)  # Chip select pin
reset = DigitalInOut(board.D25)  # Reset pin

# Initialze RFM radio
rfm9x = RFM9x(spi, cs, reset, RADIO_FREQ_MHZ)

# Set up the LoRa settings
rfm9x.tx_power = 23  # Set transmit power (in dBm)
rfm9x.enable_crc = True  # Enable CRC checks

def send_lora_message():
    message = f"{unique_id}: Hello from Raspberry Pi!"
    print(f"\nSending message: {message}\n")
    rfm9x.send(bytes(message, "utf-8"))


while True:
    try:
        # Send the message
        send_lora_message()

        # Attempt to receive a packet
        packet = rfm9x.receive(timeout=5.0, with_header=True)

        if packet is None:
            # No packet received, retrying
            print("Received nothing! Listening again...")

        else:
            # Received a packet
            packet_text = str(packet, "ascii")  # Convert to ASCII if it's a text message
            print(f"Received (ASCII): {packet_text}")

            # # Print the raw bytes
            # print(f"Received (raw bytes): {packet}")

            # Get and display the RSSI (signal strength) of the last received message
            rssi = rfm9x.last_rssi
            print(f"Received signal strength: {rssi} dB")

        # Delay before the next cycle to avoid rapid loops (optional)
        time.sleep(1)  # A short sleep to manage loop timing

    except Exception as e:
        # If an error occurs (e.g., communication issues), log it and continue
        print(f"Error: {e}")
        time.sleep(1)  # Wait a moment before retrying in case of error

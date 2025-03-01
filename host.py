import struct
import time

import RPi.GPIO as GPIO
import spidev
from getmac import get_mac_address

# Unique ID based on MAC address
unique_id = "Host" + get_mac_address(interface="eth0").replace(":", "")[-6:].upper()
print(f"Raspberry Pi Unique ID: {unique_id}\n")

# LoRa Configuration
RADIO_FREQ_MHZ = 915.0
SPI_BUS = 0
SPI_DEVICE = 1
CS_PIN = 7  # Chip select (GPIO07, CE1)
RESET_PIN = 25  # Reset pin

# Register
REG_FIFO = 0x00
REG_OP_MODE = 0x01
REG_FRF_MSB = 0x06
REG_FRF_MID = 0x07
REG_FRF_LSB = 0x08
REG_PA_CONFIG = 0x09
REG_FIFO_ADDR_PTR = 0x0D
REG_FIFO_TX_BASE_ADDR = 0x0E
REG_FIFO_RX_BASE_ADDR = 0x0F
REG_IRQ_FLAGS = 0x12
REG_RX_NB_BYTES = 0x13
REG_MODEM_CONFIG_1 = 0x1D
REG_MODEM_CONFIG_2 = 0x1E
REG_PREAMBLE_MSB = 0x20
REG_PREAMBLE_LSB = 0x21
REG_PAYLOAD_LENGTH = 0x22
REG_MODEM_CONFIG_3 = 0x26
REG_PA_DAC = 0x4D

# Modes
MODE_LONG_RANGE_MODE = 0x80
MODE_SLEEP = 0x00
MODE_TX = 0x03
MODE_RX_CONTINUOUS = 0x05

# IRQ masks
IRQ_TX_DONE_MASK = 0x08
IRQ_RX_DONE_MASK = 0x40

# Initialize SPI
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = 10000000  # 10 MHz SPI speed

# GPIO Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT)
GPIO.setup(RESET_PIN, GPIO.OUT)

# Reset LoRa module
def reset_lora():
    GPIO.output(RESET_PIN, 0)
    time.sleep(0.01)
    GPIO.output(RESET_PIN, 1)
    time.sleep(0.01)

# Function to send SPI data
def spi_write(register, value):
    GPIO.output(CS_PIN, 0)  # Select LoRa module
    spi.xfer2([register | 0x80, value])  # Write operation (bit 7 high)
    GPIO.output(CS_PIN, 1)  # Deselect LoRa module

# Function to read SPI data
def spi_read(register):
    GPIO.output(CS_PIN, 0)
    value = spi.xfer2([register & 0x7F, 0x00])  # Read operation (bit 7 low)
    GPIO.output(CS_PIN, 1)
    return value[1]  # Return data byte

# Initialize LoRa module
def init_lora():
    reset_lora()
    spi_write(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP)  # Set LoRa mode, sleep mode
    # Set frequency (example: 915 MHz)
    freq = int((915000000 / 32e6) * (2**19))
    spi_write(REG_FRF_MSB, (freq >> 16) & 0xFF)  # RegFrfMsb
    spi_write(REG_FRF_MID, (freq >> 8) & 0xFF)   # RegFrfMid
    spi_write(REG_FRF_LSB, freq & 0xFF)          # RegFrfLsb

    # Set output power to max (17 dBm)
    spi_write(REG_PA_CONFIG, 0x8F)  # RegPaConfig

    # Set spreading factor (SF7 for fast transmission)
    spi_write(REG_MODEM_CONFIG_1, 0x72)  # RegModemConfig1 (BW=125kHz, Coding Rate=4/5)
    spi_write(REG_MODEM_CONFIG_2, 0x74)  # RegModemConfig2 (SF=7, CRC on)

    # Set preamble length
    spi_write(REG_PREAMBLE_MSB, 0x00)  # RegPreambleMsb
    spi_write(REG_PREAMBLE_LSB, 0x08)  # RegPreambleLsb (8 symbols)

    # Enable FIFO TX base address
    spi_write(REG_FIFO_TX_BASE_ADDR, 0x00)  # Set FIFO TX base address
    spi_write(REG_FIFO_RX_BASE_ADDR, 0x00)  # Set FIFO RX base address

    spi_write(REG_MODEM_CONFIG_3, 0x04)  # Modem Config 3 (Low Data Rate Optimize OFF)
    # spi_write(REG_PA_DAC, 0x00)  # No Auto-AGC
    print("LoRa module initialized.")

# LoRa Send Packet
def send_lora_message():
    timestamp = time.time_ns()  # Nanosecond precision
    time1 = timestamp # Start of packet transmission

    message = f"{unique_id}: Hello from Raspberry Pi!"
    packet = struct.pack(">Q", timestamp) + message.encode('utf-8')
    print(f"\nSending message: {message}")
    
    # Send LoRa packet (Write to FIFO)
    spi_write(REG_FIFO_ADDR_PTR, 0x00)  # Set FIFO pointer to base
    for byte in packet:
        spi_write(REG_FIFO, byte)  # Write data to FIFO
    
    spi_write(REG_PAYLOAD_LENGTH, len(packet))  # Set payload length
    spi_write(REG_OP_MODE, MODE_TX)  # Start transmission

    # Timeout Mechanism
    start_time = time.time()
    while (spi_read(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0:  # Wait for packet sent
        if time.time() - start_time > 2:  # Timeout after 2 seconds
            print("ERROR: Send timeout!")
            return
        time.sleep(0.01)

    spi_write(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK)  # Clear TX Done flag

    time2 = time.time_ns() # End of packet transmission
    send_time = time2 - time1 # Time wasted sending packet
    return send_time

# LoRa Receive Packet
def receive_lora_message(send_time):
    spi_write(REG_OP_MODE, MODE_RX_CONTINUOUS)  # Set to continuous receive mode

    start_time = time.time()
    while (spi_read(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) == 0:  # Wait for valid packet
        if time.time() - start_time > 5:  # Timeout after 5 seconds
            print("ERROR: Receive timeout!")
            return
        time.sleep(0.01)

    time1 = time.time_ns() # Start of packet reception
    spi_write(REG_IRQ_FLAGS, IRQ_RX_DONE_MASK)  # Clear interrupt flag
    packet_length = spi_read(REG_RX_NB_BYTES)  # Get received packet length

    packet = []
    spi_write(REG_FIFO_ADDR_PTR, 0x00)  # Set FIFO read pointer
    for _ in range(packet_length):
        packet.append(spi_read(REG_FIFO))  # Read bytes
    time2 = time.time_ns() # End of packet reception

    receive_time = time2 - time1 # Time wasted processing packet
    timestamp = time2 # Get current timestamp

    received_packet = bytes(packet)
    received_timestamp = int(received_packet[:19]) # Extract timestamp

    # print(f"Received a packet!(bytes): {received_packet}")
    print(f"Received a packet!: {received_packet[19:].decode('utf-8')}")


    # ToF Calculation
    rtt_ns = timestamp - received_timestamp - send_time - receive_time
    tof_s = rtt_ns / (2 * 1e9)  # Convert ns to seconds, divide by 2 for one-way
    distance_m = tof_s * 299792458  # Speed of light

    print(f"Round-trip time: {rtt_ns} ns")
    print(f"Estimated distance: {distance_m:.3f} meters")


if __name__ == "__main__":
    # Initialize LoRa Module
    init_lora()

    # Main Loop
    while True:
        send_time = send_lora_message()
        receive_lora_message(send_time)
        time.sleep(1)  # Prevent flooding LoRa module

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

# Registers
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

def reset_lora():
    GPIO.output(RESET_PIN, 0)
    time.sleep(0.01)
    GPIO.output(RESET_PIN, 1)
    time.sleep(0.01)

def spi_write(register, value):
    GPIO.output(CS_PIN, 0)
    spi.xfer2([register | 0x80, value])
    GPIO.output(CS_PIN, 1)

def spi_read(register):
    GPIO.output(CS_PIN, 0)
    value = spi.xfer2([register & 0x7F, 0x00])
    GPIO.output(CS_PIN, 1)
    return value[1]

def init_lora():
    reset_lora()
    spi_write(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP)

    freq = int((RADIO_FREQ_MHZ * 1_000_000) / (32_000_000) * (2**19))
    spi_write(REG_FRF_MSB, (freq >> 16) & 0xFF)
    spi_write(REG_FRF_MID, (freq >> 8) & 0xFF)
    spi_write(REG_FRF_LSB, freq & 0xFF)

    # Set output power to max (17 dBm)
    spi_write(REG_PA_CONFIG, 0x8F)
    # Set spreading factor (SF7 for fast transmission)
    spi_write(REG_MODEM_CONFIG_1, 0x72)
    spi_write(REG_MODEM_CONFIG_2, 0x74)

    spi_write(REG_MODEM_CONFIG_3, 0x04)

    # Set preamble length
    spi_write(REG_PREAMBLE_MSB, 0x00)
    spi_write(REG_PREAMBLE_LSB, 0x08)
    
    # Enable FIFO TX base address
    spi_write(REG_FIFO_TX_BASE_ADDR, 0x00)
    spi_write(REG_FIFO_RX_BASE_ADDR, 0x00)

    print("LoRa module initialized.")

def send_lora_message():
    timestamp = int(time.time() * 1_000_000)

    message = f"{unique_id}: Hello from Raspberry Pi!"
    packet = struct.pack(">Q", timestamp) + message.encode('utf-8')

    print(f"\nSending message: {message}")
    print(f"Timestamp: {timestamp}")
    print(f"Packet length: {len(packet)} bytes")

    spi_write(REG_FIFO_ADDR_PTR, 0x00) # Set FIFO pointer to base
    for byte in packet:
        spi_write(REG_FIFO, byte) # Write data to FIFO

    spi_write(REG_PAYLOAD_LENGTH, len(packet)) # Set payload length
    spi_write(REG_OP_MODE, MODE_TX) # Start transmission

    start_time = time.time()
    while (spi_read(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0:
        if time.time() - start_time > 2:
            print("ERROR: Send timeout!")
            return None
        time.sleep(0.005)

    spi_write(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK) # Clear TX done flag

    return timestamp

def calculate(distances_array):
    """
    Calculate average and standard deviation.
    """
    average_distance = sum(distances_array) / len(distances_array)
    print(f"Average distance: {average_distance:.3f} meters")
    std_dev = (sum((x - average_distance) ** 2 for x in distances_array) / len(distances_array)) ** 0.5
    print(f"Standard deviation: {std_dev:.3f} meters")
    return (average_distance, std_dev)

def calculate_distance(receive_timestamp, received_timestamp, send_time_diff):
    """
    Calculate the distance based on the time of flight
    and the speed of light (3e8 m/s)
    """
    HARDWARE_DELAY = 98522.0 # µs
    rtt_ns = abs(((receive_timestamp - received_timestamp) - send_time_diff) - HARDWARE_DELAY)
    tof_s = rtt_ns / (2 * 1_000_000) # µs -> s, divide by 2 for one-way
    distance_m = tof_s * 299_792_458 / 1_000 # m -> km
    print(f"Time of flight: {tof_s:.6f} seconds")
    print(f"Distance: {distance_m:.3f} km")

    return distance_m

def receive_lora_message(send_time_diff):
    spi_write(REG_OP_MODE, MODE_RX_CONTINUOUS) # Set to RX mode

    start_time = time.time()
    while (spi_read(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) == 0:
        if time.time() - start_time > 2:
            print("ERROR: Receive timeout!")
            return
        time.sleep(0.005)

    receive_timestamp = time.time() * 1_000_000
    spi_write(REG_IRQ_FLAGS, IRQ_RX_DONE_MASK) # Clear RX done flag

    packet_length = spi_read(REG_RX_NB_BYTES)

    packet = []
    spi_write(REG_FIFO_ADDR_PTR, 0x00) # Set FIFO pointer to base
    for _ in range(packet_length):
        packet.append(spi_read(REG_FIFO))

    packet_bytes = bytes(packet)
    print(f"Received packet: {packet_bytes}")
    print(f"Received length: {len(packet_bytes)}")

    try:
        packet_split = packet_bytes.split(b"-")
        received_timestamp = int(packet_split[0].decode('utf-8'))
        message = packet_split[1].decode('utf-8')
    except (struct.error, UnicodeDecodeError):
        print("ERROR: Failed to decode packet.")
        return
    except ValueError:
        return

    print(f"\nReceived message: {message}")
    print(f"Received timestamp (from sender): {received_timestamp}")
    print(f"Reception timestamp: {receive_timestamp}")

    print("\nCalculating Time of Flight (ToF)...")
    distance_m = calculate_distance(receive_timestamp, received_timestamp, send_time_diff)
    return distance_m

def main():
    init_lora()
    DISTANCES_BUFFER_SIZE = 20 # Number of distances to average per measurement
    CALCULATIONS_BUFFER_SIZE = 10 # Number of measurements to average
    distances_buffer = [] # Buffer for distances
    measurements_buffer = [] # Buffer for average and standard deviations
    while True:
        t1 = time.time() * 1_000_000
        send_lora_message()
        t2 = time.time() * 1_000_000
        send_time_diff = t2 - t1

        # Receive the message and calculate distance
        distance_m = receive_lora_message(send_time_diff)
        if distance_m:
            # Append distance to the buffer
            distances_buffer.append(distance_m)
            # Calculate average distance and std deviation and append to buffer
            if len(distances_buffer) == DISTANCES_BUFFER_SIZE:
                print("\n#########################\n")
                print("Calculating average and standard deviation...")
                measurements_buffer.append(calculate(distances_buffer))
                # Reset the distances buffer
                distances_buffer = []
                print("\n#########################\n")


        # Retrieve the best distance when buffer is full
        if len(measurements_buffer) == CALCULATIONS_BUFFER_SIZE:
            print("\nRetrieving best distance...")
            # Sort list of tuples by standard deviation and get the lowest
            measurements_buffer.sort(key=lambda x: x[1])
            print(f"Measurements buffer: {measurements_buffer}")
            print(f"Distance: {measurements_buffer[0][0]:.3f} meters")

            exit(1)

        print("-----------------------")
        time.sleep(1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        GPIO.cleanup()
        spi.close()

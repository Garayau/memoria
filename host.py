import statistics
import struct
import time
from collections import namedtuple

import RPi.GPIO as GPIO
import spidev
from getmac import get_mac_address

# --- 1. CONFIGURACIÓN GLOBAL Y CONSTANTES ---

# Unique ID (para identificar este host en la red LoRa)
# Puedes usar la MAC address o un ID fijo
unique_id = get_mac_address(interface="eth0") # O "wlan0" si usas Wi-Fi
if not unique_id:
    unique_id = "RPI_HOST_001" # Fallback si no se obtiene la MAC

# Configuración LoRa (SX127x series - LoRa E32-915T20D)
# NOTA: Los registros están en hexadecimal, los valores de bits son decimales en python
REG_FIFO            = 0x00
REG_OP_MODE         = 0x01
REG_FRF_MSB         = 0x06
REG_FRF_MID         = 0x07
REG_FRF_LSB         = 0x08
REG_PA_CONFIG       = 0x09
REG_LNA             = 0x0C
REG_FIFO_ADDR_PTR   = 0x0D
REG_FIFO_TX_BASE_ADDR = 0x0E
REG_FIFO_RX_BASE_ADDR = 0x0F
REG_FIFO_RX_CURRENT_ADDR = 0x10
REG_IRQ_FLAGS_MASK  = 0x11
REG_IRQ_FLAGS       = 0x12
REG_RX_NB_BYTES     = 0x13
REG_PKT_RSSI_VALUE  = 0x1A
REG_MODEM_CONFIG_1  = 0x1D
REG_MODEM_CONFIG_2  = 0x1E
REG_SYMB_TIMEOUT_LSB= 0x1F
REG_PREAMBLE_MSB    = 0x20
REG_PREAMBLE_LSB    = 0x21
REG_PAYLOAD_LENGTH  = 0x22
REG_MAX_PAYLOAD_LENGTH = 0x23
REG_HOP_PERIOD      = 0x24
REG_DIO_MAPPING_1   = 0x40
REG_VERSION         = 0x42

# Modos de Operación (Reg 0x01)
MODE_SLEEP          = 0x00
MODE_STDBY          = 0x01
MODE_TX             = 0x03
MODE_RX_CONTINUOUS  = 0x05
MODE_LORA_SLEEP     = 0x80 # Modo LoRa con Sleep

# Máscaras de IRQ (Reg 0x12)
IRQ_RX_TIMEOUT_MASK = 0x80
IRQ_RX_DONE_MASK    = 0x40
IRQ_CRC_ERROR_MASK  = 0x20
IRQ_VALID_HEADER_MASK = 0x10
IRQ_TX_DONE_MASK    = 0x08
IRQ_CAD_DONE_MASK   = 0x04
IRQ_FHSS_CHANGE_CHANNEL_MASK = 0x02
IRQ_CAD_DETECTED_MASK = 0x01

# --- NUEVAS CONSTANTES PARA MEDICIÓN DE DISTANCIAS ---
DISTANCE_BUFFER_SIZE = 15 # Numero de valores TOF para promediar por medicion
MEASURE_BUFFER_SIZE = 5 # Número de mediciones promediadas (de avg/std dev de TOF)
NUM_CALIBRATION_SAMPLES = 5 # Número de las primeras muestras para estimar el hardware_delay base
OUTLIER_STD_DEV_FACTOR = 3.0 # Factor de desviación estándar para detección de outliers (3.0 es común)
MIN_ACCEPTABLE_RTT_NS = 100_000.0 # Umbral mínimo razonable para RTT (100us)
MAX_ACCEPTABLE_RTT_NS = 50_000_000.0 # Umbral máximo razonable para RTT (50ms)

# Buffers para almacenar los valores de TOF y las mediciones agregadas
tof_buffer = [] # Almacena TOFs (ya corregidos por offset) para promediar
measurements_buffer = [] # Almacena AvgStdDevResult de TOFs para promediar (mediciones compuestas)

# Variable para el hardware_delay calibrado dinámicamente
base_rtt_offset_ns = 0.0
initial_calibration_done = False # Bandera para controlar la fase de calibración

_counter = 0 # Variable global para el contador de mensajes enviados

# Estructura para almacenar el resultado del promedio y desviación estándar (como en C)
AvgStdDevResult = namedtuple("AvgStdDevResult", ["average", "std_deviation"])

# --- 2. CONFIGURACIÓN SPI Y GPIO ---

# GPIOs para LoRa (ejemplo para Raspberry Pi)
# Ajusta estos pines a tu configuración real
PIN_CS   = 8  # SPI Chip Select (CE0)
PIN_RST  = 17 # Reset LoRa
PIN_DIO0 = 24 # DIO0 (IRQ para RX_DONE/TX_DONE)

# Configuración SPI
spi = spidev.SpiDev()
spi.open(0, 0) # SPI bus 0, device 0 (CS0)
spi.max_speed_hz = 5000000 # 5 MHz

# Configuración GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_CS, GPIO.OUT)
GPIO.setup(PIN_RST, GPIO.OUT)
GPIO.setup(PIN_DIO0, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # DIO0 como entrada

# --- 3. FUNCIONES BÁSICAS DE LORA Y SPI ---

def spi_write(address, value):
    GPIO.output(PIN_CS, GPIO.LOW)
    spi.xfer2([address | 0x80, value]) # Escribir (MSB alto)
    GPIO.output(PIN_CS, GPIO.HIGH)

def spi_read(address):
    GPIO.output(PIN_CS, GPIO.LOW)
    response = spi.xfer2([address & 0x7F, 0x00]) # Leer (MSB bajo)
    GPIO.output(PIN_CS, GPIO.HIGH)
    return response[1]

def reset_lora():
    GPIO.output(PIN_RST, GPIO.LOW)
    time.sleep(0.01)
    GPIO.output(PIN_RST, GPIO.HIGH)
    time.sleep(0.01)

def init_lora():
    reset_lora()
    
    # Poner en modo Sleep antes de configurar
    spi_write(REG_OP_MODE, MODE_LORA_SLEEP) # LoRa + Sleep
    time.sleep(0.01)

    # Establecer frecuencia (915 MHz)
    # FRF = F_rf * 2^19 / F_xo
    # F_rf = 915 MHz, F_xo = 32 MHz
    # 915,000,000 * 524288 / 32,000,000 = 15000000 (aproximado)
    frf = int(915000000 / (32000000 / 524288.0))
    spi_write(REG_FRF_MSB, (frf >> 16) & 0xFF)
    spi_write(REG_FRF_MID, (frf >> 8) & 0xFF)
    spi_write(REG_FRF_LSB, frf & 0xFF)

    # Configuración de potencia de PA (PA_BOOST) y LNA
    spi_write(REG_PA_CONFIG, 0xFF) # Max Power (PA_BOOST)
    spi_write(REG_LNA, 0x23) # LNA gain highest

    # Configuraciones LoRa (ejemplo de valores comunes)
    # Bandwidth: 125KHz (0x70), Coding Rate: 4/5 (0x02), Implicit Header: Disable (0x00) -> 0x72
    spi_write(REG_MODEM_CONFIG_1, 0x72) 
    # Spreading Factor: 7 (0x70), TX Continuous: Disable (0x00), RX Timeout MSB (0x00) -> 0x70
    spi_write(REG_MODEM_CONFIG_2, 0x74) # SF7, CRC Enable
    spi_write(REG_SYMB_TIMEOUT_LSB, 0x64) # Timeout 100
    spi_write(REG_PREAMBLE_MSB, 0x00) # Preamble 8 bytes
    spi_write(REG_PREAMBLE_LSB, 0x08)
    spi_write(REG_DIO_MAPPING_1, 0x00) # DIO0 para TxDone/RxDone
    spi_write(REG_FIFO_TX_BASE_ADDR, 0x00)
    spi_write(REG_FIFO_RX_BASE_ADDR, 0x00)

    # Poner en modo Standby para limpiar IRQ flags y FIFO
    spi_write(REG_OP_MODE, MODE_STDBY)
    time.sleep(0.01) # Pequeño delay para asegurar el modo

    # Limpiar todas las banderas de interrupción
    spi_write(REG_IRQ_FLAGS, 0xFF)

    # print("LoRa initialized.")

# --- 4. FUNCIONES DE COMUNICACIÓN (ADAPTADAS) ---

def send_lora_message():
    """
    Envía un mensaje LoRa con el timestamp inicial del Host.
    Retorna el timestamp inicial que fue enviado.
    """
    global _counter
    # timestamp_initial es el 'ts_en_paquete' original del Host
    timestamp_initial = time.time_ns() 

    message = f"{unique_id}: Message No. {_counter}"
    _counter += 1

    # Empaquetamos SOLO un uint64_t (el timestamp inicial) y luego el mensaje de texto.
    # El C Receptor espera solo este timestamp en el primer paquete.
    packet_data = struct.pack("<Q", timestamp_initial) + message.encode('utf-8')

    # print(f"\nSending message: {message}")
    # print(f"Host Initial Timestamp (Python): {timestamp_initial} ns")
    # print(f"Packet length: {len(packet_data)} bytes")

    spi_write(REG_FIFO_ADDR_PTR, 0x00) # Puntero FIFO al inicio
    for byte in packet_data:
        spi_write(REG_FIFO, byte)

    spi_write(REG_PAYLOAD_LENGTH, len(packet_data))
    spi_write(REG_OP_MODE, MODE_TX)

    start_time = time.time()
    # Esperar por la bandera de TX_DONE o timeout
    while (spi_read(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0:
        if time.time() - start_time > 2: # Timeout de 2 segundos
            # print("ERROR: Send timeout!")
            # Intentar limpiar la bandera de TX_DONE en caso de timeout para evitar quedar atascado
            spi_write(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK) 
            return None
        time.sleep(0.005)

    spi_write(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK) # Limpiar la bandera de TX_DONE
    spi_write(REG_OP_MODE, MODE_STDBY) # Volver a Standby después de TX
    return timestamp_initial # Retorna el timestamp inicial enviado

def receive_single_lora_packet(timeout_s=3):
    """
    Espera y recibe un único paquete LoRa.
    Retorna (timestamp_recepcion_local_host, valor_uint64_de_paquete) o (None, None) en timeout/error.
    """
    spi_write(REG_OP_MODE, MODE_RX_CONTINUOUS) # Entrar en modo RX_CONTINUOUS
    start_time = time.time()
    
    # Esperar por la bandera de RX_DONE o timeout
    while (spi_read(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) == 0:
        if time.time() - start_time > timeout_s:
            print(f"ERROR: Receive timeout after {timeout_s} seconds!")
            # Limpiar IRQ_RX_DONE en caso de timeout
            spi_write(REG_IRQ_FLAGS, IRQ_RX_DONE_MASK) 
            spi_write(REG_OP_MODE, MODE_STDBY) # Volver a Standby
            return None, None
        time.sleep(0.005)

    receive_timestamp_local = time.time_ns() # Timestamp local de la Raspberry Pi al recibir
    spi_write(REG_IRQ_FLAGS, IRQ_RX_DONE_MASK | IRQ_VALID_HEADER_MASK | IRQ_CRC_ERROR_MASK) # Limpiar banderas RX

    packet_length = spi_read(REG_RX_NB_BYTES)
    # print(f"Raw packet length reported: {packet_length} bytes") # Depuración

    # Obtener el puntero al inicio del FIFO para el paquete recibido
    current_fifo_addr = spi_read(REG_FIFO_RX_CURRENT_ADDR)
    spi_write(REG_FIFO_ADDR_PTR, current_fifo_addr)

    packet = []
    for _ in range(packet_length):
        packet.append(spi_read(REG_FIFO))

    packet_bytes = bytes(packet)
    spi_write(REG_OP_MODE, MODE_STDBY) # Volver a Standby después de RX

    if len(packet_bytes) < 8:
        print(f"ERROR: Received packet too short ({len(packet_bytes)} bytes). Expected at least 8 bytes.")
        return None, None

    # Desempaquetar el primer uint64_t (8 bytes) del paquete
    try:
        val_from_packet = struct.unpack("<Q", packet_bytes[0:8])[0]
        message_content = packet_bytes[8:].decode('utf-8')
    except struct.error as e:
        print(f"ERROR: Failed to unpack packet header: {e}")
        return None, None
    except UnicodeDecodeError as e:
        print(f"ERROR: Failed to decode message part: {e}")
        # En caso de error de decodificación, aún podemos retornar el valor numérico
        return receive_timestamp_local, val_from_packet

    # print(f"  Packet Value: {val_from_packet} ns")
    # print(f"  Message Content: '{message_content}'")
    # print(f"  Host Local Reception Timestamp: {receive_timestamp_local} ns")

    return receive_timestamp_local, val_from_packet

# --- 5. FUNCIONES DE CÁLCULO DE DISTANCIA (ADAPTADAS) ---

def calculate_avg_stddev(data):
    """
    Calcula el promedio y la desviación estándar de una lista de datos.
    """
    if not data:
        return AvgStdDevResult(average=0.0, std_deviation=0.0)

    avg = statistics.mean(data)
    
    if len(data) > 1:
        std_dev = statistics.stdev(data)
    else:
        std_dev = 0.0

    return AvgStdDevResult(average=avg, std_deviation=std_dev)

def calculate_distance(
    host_initial_tx_timestamp, # El timestamp que el Host envió originalmente (T_H_TX_initial)
    host_tx_delay,             # El tiempo que al Host le tomó enviar su primer mensaje (D_H_TX)
    ts_reception_local_packet1, # Timestamp del Host al recibir el 1er paquete del Receptor (T_H_RX1)
    val_from_packet1,           # Valor del 1er paquete del Receptor (T_H_TX_initial + D_R_Processing)
    ts_reception_local_packet2, # Timestamp del Host al recibir el 2do paquete del Receptor (T_H_RX2)
    val_from_packet2            # Valor del 2do paquete del Receptor (D_R_TX1)
):
    """
    Calcula el RTT y el TOF basándose en los tiempos y delays de ambos paquetes,
    adaptándose al comportamiento de dos paquetes del ESP32 Receptor.
    """
    # La fórmula de RTT se adapta para replicar la lógica del C Host, usando
    # los valores de ambos paquetes recibidos y el delay de envío del Host.
    # RTT = T_H_RX2 - (T_H_TX_initial + D_R_Processing) - D_H_TX - D_R_TX1
    #       (ts_recepcion_local_packet2 - val_from_packet1 - host_tx_delay - val_from_packet2)
    rtt_ns = ts_reception_local_packet2 - val_from_packet1 - host_tx_delay - val_from_packet2
    print(f"\nRTT Calculado: {rtt_ns} ns ({rtt_ns / 1_000_000:.3f} ms)")
    # Filtro básico de outliers extremos antes de usar el RTT
    if rtt_ns < MIN_ACCEPTABLE_RTT_NS or rtt_ns > MAX_ACCEPTABLE_RTT_NS:
        print(f"ADVERTENCIA: RTT {rtt_ns} ns ({rtt_ns / 1_000_000:.3f} ms) fuera de rango aceptable. Descartando.")
        return None # Retornar None para indicar descarte

    # Calibración dinámica del base_rtt_offset_ns (HARDWARE_DELAY)
    global base_rtt_offset_ns, initial_calibration_done, tof_buffer

    if not initial_calibration_done:
        if len(tof_buffer) < NUM_CALIBRATION_SAMPLES:
            tof_buffer.append(float(rtt_ns)) # Almacenamos el RTT completo para la calibración
            print(f"Calibrando... Muestra {len(tof_buffer)}/{NUM_CALIBRATION_SAMPLES}")
            if len(tof_buffer) == NUM_CALIBRATION_SAMPLES:
                base_rtt_offset_ns = statistics.mean(tof_buffer)
                initial_calibration_done = True
                tof_buffer = [] # Resetear el buffer para usarlo para TOFs reales
                print("\n--- CALIBRACIÓN INICIAL COMPLETA ---")
                print(f"Base RTT Offset (Hardware Delay) Calibrado: {base_rtt_offset_ns:.3f} ns ({base_rtt_offset_ns / 1_000_000:.3f} ms)")
                print("-----------------------------------\n")
            return None # Retornar None durante la calibración
    
    # Calcular TOF si la calibración está hecha
    tof = rtt_ns - base_rtt_offset_ns
    
    # Asegurarse de que el TOF sea positivo antes de usarlo
    if tof < 0:
        print(f"ADVERTENCIA: TOF calculado es negativo ({tof:.3f} ns). Descartando.")
        return None

    print(f"\nCalculated RTT: {rtt_ns} ns ({rtt_ns / 1_000_000:.3f} ms)")
    print(f"Calibrated TOF (total): {tof:.3f} ns")
    
    # TOF es el tiempo de vuelo de ida y vuelta. La distancia es un solo sentido.
    distance_m = (tof / 1_000_000_000.0) * 299_792_458.0 / 2.0 

    # Abrir archivo de log para registro
    with open("log.txt", "a") as log_file:
        log_file.write(f"RTT: {rtt_ns} ns, TOF: {tof:.3f} ns, Dist: {distance_m:.3f} m\n")
        
    print(f"Distance: {distance_m:.3f} meters")

    # Si la calibración está hecha, se añade el TOF calculado (después del offset) al buffer de TOFs.
    if initial_calibration_done:
        if len(tof_buffer) < DISTANCE_BUFFER_SIZE:
            tof_buffer.append(tof) # Añadir el TOF calculado (después de offset)
            if len(tof_buffer) == DISTANCE_BUFFER_SIZE:
                print("\n#########################\n")
                print(f"Calculando promedio y desviación estándar para {len(tof_buffer)} valores de TOF...")
                
                # Filtrado de outliers por desviación estándar
                current_avg_stddev = calculate_avg_stddev(tof_buffer)
                
                # Solo filtrar si hay suficientes datos y una desviación estándar significativa
                if current_avg_stddev.std_deviation > 0 and len(tof_buffer) > 1:
                    filtered_tof_buffer = [
                        val for val in tof_buffer 
                        if abs(val - current_avg_stddev.average) <= OUTLIER_STD_DEV_FACTOR * current_avg_stddev.std_deviation
                    ]
                else:
                    filtered_tof_buffer = list(tof_buffer) # No hay outliers que filtrar

                if len(tof_buffer) != len(filtered_tof_buffer):
                    print(f"  Descartados {len(tof_buffer) - len(filtered_tof_buffer)} outliers.")
                
                # Si el filtrado elimina todos los puntos, no se agrega nada al buffer de mediciones
                if not filtered_tof_buffer:
                    print("ADVERTENCIA: Todos los puntos fueron descartados como outliers en este lote. No se agrega medición.")
                else:
                    final_tof_result = calculate_avg_stddev(filtered_tof_buffer)
                    
                    global measurements_buffer
                    if len(measurements_buffer) < MEASURE_BUFFER_SIZE:
                        measurements_buffer.append(final_tof_result)
                    else:
                        # Si el buffer de mediciones está lleno, podemos reemplazar la peor o descartar
                        # Aquí, simplemente reemplazamos la más antigua (FIFO)
                        measurements_buffer.pop(0) 
                        measurements_buffer.append(final_tof_result)

                tof_buffer.clear() # Limpiar el buffer para la próxima serie
                print("#########################\n")

    return distance_m # calculate_distance ahora retorna la distancia


# --- 6. FUNCIÓN PRINCIPAL ---

def main():
    try:
        init_lora()
        global tof_buffer, measurements_buffer, initial_calibration_done

        host_send_delay_current = 0 # Variable para almacenar el delay de envío del Host

        # Limpiar el archivo de log al inicio
        with open("log.txt", "w") as log_file:
            log_file.write(f"--- LoRa RTT Log (Host ID: {unique_id})---\n")

        while True:
            # 1. Host envía el mensaje inicial y mide su propio delay de envío
            t1_send = time.time_ns()
            host_initial_tx_timestamp = send_lora_message() 
            t2_send = time.time_ns()
            
            if host_initial_tx_timestamp is None:
                # print("Failed to send initial packet. Retrying cycle.")
                time.sleep(1)
                continue

            host_send_delay_current = t2_send - t1_send 
            # print(f"Host Measured TX Delay: {host_send_delay_current} ns")

            # 2. Host espera y recibe el PRIMER paquete del Receptor
            # print("\nWaiting for Receptor's FIRST response packet (Value: HostInitialTS + ProcessingDelay)...")
            ts_reception_local_packet1, val_from_packet1 = receive_single_lora_packet(timeout_s=3)

            if ts_reception_local_packet1 is None:
                # print("Failed to receive FIRST packet from Receptor. Retrying cycle.")
                time.sleep(1) # Pausa antes de reintentar
                continue # Volver al inicio del bucle

            # 3. Host espera y recibe el SEGUNDO paquete del Receptor
            # print("Waiting for Receptor's SECOND response packet (Value: ReceptorTxDelay_for_FirstPacket)...")
            # Un timeout más corto para el segundo paquete, ya que debería llegar muy rápido.
            ts_reception_local_packet2, val_from_packet2 = receive_single_lora_packet(timeout_s=1) 

            if ts_reception_local_packet2 is None:
                # print("Failed to receive SECOND packet from Receptor. Retrying cycle.")
                time.sleep(1)
                continue

            # 4. Calcular la distancia usando todos los valores
            distance_m = calculate_distance(
                host_initial_tx_timestamp,
                host_send_delay_current,
                ts_reception_local_packet1, # Este valor no se usa directamente en el RTT, pero se pasa para consistencia/depuración
                val_from_packet1,
                ts_reception_local_packet2,
                val_from_packet2
            )

            # Lógica para procesar el buffer de mediciones compuestas (si se llenó en calculate_distance)
            if initial_calibration_done and len(measurements_buffer) == MEASURE_BUFFER_SIZE:
                print("\n--- AGREGANDO MEDICIONES ---")
                
                # Ordenar por desviación estándar para encontrar la "mejor" medición
                measurements_buffer.sort(key=lambda x: x.std_deviation)
                
                print("Buffer de Mediciones Agregadas (Promedio TOF, Desviación Estándar TOF):")
                for m in measurements_buffer:
                    print(f"    {{Promedio: {m.average:.3f} ns, Desv. Est.: {m.std_deviation:.3f} ns}}")
                
                best_avg_tof = measurements_buffer[0].average
                best_std_dev_tof = measurements_buffer[0].std_deviation
                best_distance_m = (best_avg_tof / 1_000_000_000.0) * 299_792_458.0 / 2.0

                print(f"\nDistancia Estimada (promedio de {MEASURE_BUFFER_SIZE} lotes, menor desv. est.): {best_distance_m:.3f} m")
                print(f"  (Promedio TOF del Lote: {best_avg_tof:.3f} ns, Desv. Est. TOF del Lote: {best_std_dev_tof:.3f} ns)\n")
                
                # Opcional: escribir la mejor distancia agregada al log
                with open("log.txt", "a") as log_file:
                    log_file.write(f"AGREGADO - Dist: {best_distance_m:.3f} m (Avg TOF: {best_avg_tof:.3f} ns, Std Dev TOF: {best_std_dev_tof:.3f} ns)\n")

                measurements_buffer.clear() # Limpiar el buffer de mediciones después de procesar

            print("-----------------------")
            time.sleep(1) # Pausa entre ciclos completos de medición

    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        spi.close()
        GPIO.cleanup()
        print("SPI closed, GPIO cleaned up.")

if __name__ == "__main__":
    main()
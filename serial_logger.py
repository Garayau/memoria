import os
import re
import time

import serial

# --- CONFIGURACIÓN ---
SERIAL_PORT = '/dev/ttyUSB0'  # CAMBIA ESTO al puerto serial de tu ESP32 (ej. 'COM3' o '/dev/ttyUSB0')
BAUD_RATE = 115200            # Velocidad de baudios de tu ESP32
OUTPUT_DIR = 'lora_cycle_logs' # Directorio para guardar los archivos CSV

# Nombres de archivos CSV separados
RTT_CYCLE_CSV = os.path.join(OUTPUT_DIR, 'rtt_cycle_log.csv')
TOF_CYCLE_CSV = os.path.join(OUTPUT_DIR, 'tof_cycle_log.csv')
RSSI_CYCLE_CSV = os.path.join(OUTPUT_DIR, 'rssi_cycle_log.csv')

# Encabezados para los archivos CSV (AHORA INCLUYEN LOS PROPORCIONADOS POR EL USUARIO)
# Cada archivo tendrá el Cycle_ID y una columna para los datos combinados.
# Se añade "Cycle_ID," al inicio de los encabezados proporcionados por el ESP32.
RTT_FILE_HEADER = "Cycle_ID,[RTT] (para offset), [RTT] (para tof)"
TOF_FILE_HEADER = "Cycle_ID,offset_ns, [tof_buffer], [prom, std_dev] (iniciales), [prom, std_dev] (despues de filtrado), distancia"
RSSI_FILE_HEADER = "Cycle_ID, rssi/snr"

# Buffers para acumular el contenido de RTT_DATA y TOF_RAW para el ciclo actual
current_rtt_data_parts = []
current_tof_raw_parts = []
current_rssi_data_parts = []

current_cycle_id = 0 # ID del ciclo actual, se incrementa con cada '#'

# Manejadores de archivos
rtt_file_handle = None
tof_file_handle = None
rssi_file_handle = None

def initialize_csv_files():
    """Crea los archivos CSV con sus encabezados si no existen y los abre en modo de añadir."""
    global rtt_file_handle, tof_file_handle, rssi_file_handle
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        print(f"Directorio '{OUTPUT_DIR}' creado.")

    # Archivo para RTT
    if not os.path.exists(RTT_CYCLE_CSV):
        print(f"Creando archivo RTT: {RTT_CYCLE_CSV}")
        rtt_file_handle = open(RTT_CYCLE_CSV, 'w', newline='')
        rtt_file_handle.write(RTT_FILE_HEADER + '\n')
    else:
        rtt_file_handle = open(RTT_CYCLE_CSV, 'a', newline='')
    
    # Archivo para TOF
    if not os.path.exists(TOF_CYCLE_CSV):
        print(f"Creando archivo TOF: {TOF_CYCLE_CSV}")
        tof_file_handle = open(TOF_CYCLE_CSV, 'w', newline='')
        tof_file_handle.write(TOF_FILE_HEADER + '\n')
    else:
        tof_file_handle = open(TOF_CYCLE_CSV, 'a', newline='')

    # Archivo para RSSI
    if not os.path.exists(RSSI_CYCLE_CSV):
        print(f"Creando archivo RSSI: {RSSI_CYCLE_CSV}")
        rssi_file_handle = open(RSSI_CYCLE_CSV, 'w', newline='')
        rssi_file_handle.write(RSSI_FILE_HEADER + '\n')
    else:
        rssi_file_handle = open(RSSI_CYCLE_CSV, 'a', newline='')
    
    print("Archivos CSV inicializados. Esperando datos del ESP32...")

def close_csv_files():
    """Cierra todos los archivos CSV abiertos."""
    global rtt_file_handle, tof_file_handle, rssi_file_handle
    print("Cerrando archivos CSV...")
    if rtt_file_handle:
        rtt_file_handle.close()
    if tof_file_handle:
        tof_file_handle.close()
    if rssi_file_handle:
        rssi_file_handle.close()

def write_cycle_data_to_csv_files():
    """Toma los datos acumulados del ciclo actual y los escribe como una sola fila en sus respectivos CSVs."""
    global current_rtt_data_parts, current_tof_raw_parts, current_rssi_data_parts, rtt_file_handle, tof_file_handle, current_cycle_id

    # --- Escribir datos RTT ---
    if current_rtt_data_parts:
        # Unir todas las partes de RTT_DATA en una sola cadena
        cleaned_rtt_parts = [re.sub(r"^RTT_DATA:", "", part).strip() for part in current_rtt_data_parts]
        rtt_combined_content = "".join(cleaned_rtt_parts)
        escaped_rtt_content = rtt_combined_content.replace('"', '""')
        rtt_file_handle.write(f"{current_cycle_id},\"{escaped_rtt_content}\"\n")
        rtt_file_handle.flush()

    # --- Escribir datos TOF ---
    if current_tof_raw_parts:
        # Unir todas las partes de TOF_RAW en una sola cadena
        cleaned_tof_parts = [re.sub(r"^TOF_RAW:", "", part).strip() for part in current_tof_raw_parts]
        tof_combined_content = "".join(cleaned_tof_parts)
        escaped_tof_content = tof_combined_content.replace('"', '""')
        tof_file_handle.write(f"{current_cycle_id},\"{escaped_tof_content}\"\n")
        tof_file_handle.flush()

    # --- Escribir datos RSSI ---
    if current_rssi_data_parts:
        # Unir todas las partes de RSSI_DATA en una sola cadena
        cleaned_rssi_parts = [re.sub(r"^RSSI_DATA:", "", part).strip() for part in current_rssi_data_parts]
        rssi_combined_content = "".join(cleaned_rssi_parts)
        escaped_rssi_content = rssi_combined_content.replace('"', '""')
        rssi_file_handle.write(f"{current_cycle_id},\"{escaped_rssi_content}\"\n")
        rssi_file_handle.flush()

    # Limpiar los buffers para el próximo ciclo
    current_rtt_data_parts = []
    current_tof_raw_parts = []
    current_rssi_data_parts = []

def main():
    global current_cycle_id, current_rtt_data_parts, current_tof_raw_parts, current_rssi_data_parts

    initialize_csv_files()
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Conectado al puerto serial {SERIAL_PORT} a {BAUD_RATE} baudios.")
        print("Presiona Ctrl+C para detener el registro.")

        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()

            if line:
                if line == '#':
                    # Fin de un ciclo, escribir los datos acumulados en ambos archivos
                    write_cycle_data_to_csv_files()
                    current_cycle_id += 1 # Preparar para el próximo ciclo
                    print(f"--- Datos del Ciclo {current_cycle_id - 1} escritos. Inicio de Ciclo {current_cycle_id} ---")
                elif line.startswith("RTT_HEADER:"):
                    # Imprime los encabezados del ESP32 en la consola, pero no al CSV de datos
                    # porque los encabezados ya están definidos en el script Python.
                    print(f"[ESP32 HEADER]: {line}")
                elif line.startswith("TOF_HEADER:"):
                    # Imprime los encabezados del ESP32 en la consola, pero no al CSV de datos
                    print(f"[ESP32 HEADER]: {line}")
                elif line.startswith("RTT_DATA:"):
                    current_rtt_data_parts.append(line)
                elif line.startswith("TOF_RAW:"):
                    current_tof_raw_parts.append(line)
                elif line.startswith("RSSI_DATA:"):
                    pass  # No procesar RSSI_DATA aquí, se maneja en la sección de RSSI
                else:
                    # Imprime otras líneas de depuración del ESP32 en la consola
                    print(f"[ESP32]: {line}")

            # Seccion de RSSI
            if line:
                if line.startswith("RSSI_DATA:"):
                    current_rssi_data_parts.append(line)
                elif line.startswith("rssi_15"):
                    write_cycle_data_to_csv_files()
                    print(f"--- Datos RSSI/SNR del Ciclo {current_cycle_id - 1} escritos ---")

    except serial.SerialException as e:
        print(f"Error de puerto serial: {e}")
        print(f"Asegúrate de que {SERIAL_PORT} es el puerto correcto y no está en uso.")
    except KeyboardInterrupt:
        print("\nRegistro detenido por el usuario.")
    finally:
        # Asegúrate de escribir cualquier dato restante en los buffers al salir (si no hubo un '#' final)
        if current_rtt_data_parts or current_tof_raw_parts or current_rssi_data_parts:
            write_cycle_data_to_csv_files() 
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Puerto serial cerrado.")
        close_csv_files()

if __name__ == "__main__":
    main()


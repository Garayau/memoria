#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include <esp_log.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "rom/gpio.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "xtensa/core-macros.h"
#include <math.h>
#include <stddef.h>
#include <float.h>

#include "LoRa.h" // Asegúrate de que esta librería LoRa sea compatible con ESP32

#define PIN_NUM_CLK  	18
#define PIN_NUM_MISO 	19
#define PIN_NUM_MOSI 	23
#define PIN_NUM_CS   	4
#define PIN_NUM_DIO		26
#define RESET_PIN  		12

#define	FLASH_PIN			2 // Pin para indicar actividad (opcional)

#define UNIQUE_ID_MAX_LEN 20  // Longitud máxima para la cadena de ID

char unique_id[UNIQUE_ID_MAX_LEN];  // Variable global para almacenar el ID único
int _counter = 0; // Contador de mensajes enviados

// Define si este ESP32 es el "Host" (el que inicia la comunicación)
// Esto se determinará por el último byte de la MAC.
bool is_host = true;
bool continue_sending = false; // Variable para controlar si el Host puede seguir enviando mensajes

// Constantes para medir distancias2
#define DISTANCE_BUFFER_SIZE 15 // Numero de valores TOF para promediar por medicion
#define MEASURE_BUFFER_SIZE 3 // Número de mediciones promediadas
#define NUM_CALIBRATION_SAMPLES 5 // Número de las primeras muestras para estimar el hardware_delay base
#define OUTLIER_STD_DEV_FACTOR 3.0 // Factor de desviación estándar para detección de outliers (3.0 es común)
#define MIN_ACCEPTABLE_RTT_NS 100.0 // Umbral mínimo razonable para RTT (100ns), para descartar valores extremadamente bajos/corruptos
#define MAX_ACCEPTABLE_RTT_NS 5000000.0 // Umbral máximo razonable para RTT (5ms), para descartar valores extremadamente altos/corruptos

// Buffers para almacenar los valores de TOF (Tiempo de Vuelo)
double tof_buffer[DISTANCE_BUFFER_SIZE]; 
size_t tof_buffer_idx = 0; // Indice actual para tof_buffer

// Estructura para almacenar el resultado del promedio y desviación estándar
typedef struct {
    double average;
    double std_deviation;
} AvgStdDevResult;

// Buffer para almacenar los resultados de las mediciones
AvgStdDevResult measurements_buffer[MEASURE_BUFFER_SIZE];
size_t measurements_buffer_idx = 0; // Índice actual para measurements_buffer

// Tiempo máximo de espera en microsegundos para recibir un mensaje antes de intentar enviar
#define LORA_RECEIVE_TIMEOUT_US 3000000 // 3 segundos

// Almacena el timestamp del momento justo después del envío de un mensaje del Host
uint64_t host_send_delay = 0;
// Almacena el timestamp del momento justo después del envío de un mensaje del Receptor
uint64_t receptor_send_delay = 0;

// Variable para el hardware_delay calibrado dinámicamente
double base_rtt_offset_ns = 0.0;
bool initial_calibration_done = false;

// Funciones Auxiliares

// Función para calcular el promedio y la desviación estándar de un array de datos
AvgStdDevResult calculate_avg_stddev(const double data[], size_t size) {
    AvgStdDevResult result;
    result.average = 0.0;
    result.std_deviation = 0.0;

    if (size == 0) {
        return result; // Si el tamaño es 0, retornar resultado vacío
    }

    double sum = 0.0;
    for (size_t i = 0; i < size; i++) {
        sum += data[i];
    }
    result.average = sum / size; // Calcular el promedio

    double sq_sum = 0.0;
    for (size_t i = 0; i < size; i++) {
        sq_sum += (data[i] - result.average) * (data[i] - result.average);
    }
    // Calcular la desviación estándar: (size - 1) para una muestra si size > 1, si no 1.0
    result.std_deviation = sqrt(sq_sum / (size > 1 ? (double)(size - 1) : 1.0)); 

    return result;
}

// Función de comparación para ordenar los resultados por desviación estándar
int compare_avg_stddev_results(const void *a, const void *b) {
    const AvgStdDevResult *result_a = (const AvgStdDevResult *)a;
    const AvgStdDevResult *result_b = (const AvgStdDevResult *)b;

    if (result_a->std_deviation < result_b->std_deviation) {
        return -1; // a comes before b
    } else if (result_a->std_deviation > result_b->std_deviation) {
        return 1;  // b comes before a
    } else {
        return 0;  // they are equal
    }
}

// Función para generar un ID único basado en la MAC del ESP32 y determinar el rol
void generate_unique_id() {
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);

    snprintf(unique_id, sizeof(unique_id), "ESP32-%02X%02X%02X", mac[3], mac[4], mac[5]);
    ESP_LOGI("UID", "ESP32 Unique ID: %s", unique_id);

    if (mac[5] == 0x94) { // Si el último byte de la MAC es 0x94, este ESP32 es el Host
        is_host = true;
    } else {
        is_host = false;
    }

    ESP_LOGI("ROLE", "Este ESP32 es el %s.", is_host ? "HOST" : "RECEPTOR");
}

// Configuración del reloj del ESP32
#ifndef CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ
#define CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ 240  // fallback
#endif

// Función para obtener el tiempo en nanosegundos
uint64_t get_nanotime() {
    uint32_t cycles = XTHAL_GET_CCOUNT();
    return (uint64_t)cycles * 1000 / CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
}

// Función para enviar un mensaje LoRa
void writeMessage( LoRa* lora, uint64_t timestamp_payload )
{
    char msg_content[100]; // Contenido del mensaje de texto
    sprintf( msg_content, "%s: Mensaje No. [%d]", unique_id, _counter++);

    lora->beginPacket(false);

    // 1. Enviar el timestamp binario (8 bytes) al inicio del payload
    lora->write((uint8_t*)&timestamp_payload, sizeof(uint64_t));

    // 2. Enviar el contenido del mensaje de texto
    lora->write( (uint8_t*) msg_content, (size_t) strlen(msg_content) );
    lora->endPacket(false);

    printf("--> [TX] Enviando TS: %llu, Contenido: '%s'\n", timestamp_payload, msg_content);
}

// Función de retardo
void delay( int msec )
{
    vTaskDelay( msec / portTICK_PERIOD_MS);
}

uint64_t send_message_and_get_delay(LoRa* lora, uint64_t timestamp_payload) {
    // Esta función envía un mensaje LoRa con un timestamp y retorna el tiempo que tomó enviar el mensaje.
    // El timestamp_payload es el tiempo en microsegundos que se envía como parte del mensaje.
    uint64_t ts_before_send = get_nanotime(); // Momento de envío
    writeMessage(lora, timestamp_payload); // Envía el mensaje con el timestamp
    uint64_t ts_after_send = get_nanotime(); // Momento justo después del envío
    return ts_after_send - ts_before_send; // Retorna el tiempo que tomó enviar el mensaje
}

// Función para manejar el mensaje recibido
int handle_message(LoRa* lora, uint64_t *ts_en_paquete, uint64_t *ts_recepcion_local=nullptr) {
    char msg_content[100] = {0};
    
    int packetSize = lora->handleDataReceived(msg_content, ts_en_paquete);
    lora->setDataReceived(false);
    
    if (packetSize == 0) { // handleDataReceived devuelve 0 si hay error o paquete corrupto
        printf("Paquete inválido o error de CRC.\n");
        return 0;
    }
    
    // printf("\n<-- [RX] Mensaje Recibido: '%s'\n", msg_content);
    // printf("    Timestamp en Paquete: %llu ns\n", *ts_en_paquete);
    if (ts_recepcion_local != nullptr) {
        // printf("    Timestamp Recepción Local (RX_DONE): %llu ns\n", *ts_recepcion_local);
    }
    // printf("    RSSI: %d dBm, SNR: %d dB\n", lora->getPacketRssi(), lora->getPacketSnr());
    
    return 1;
}

// Función Principal

extern "C" void app_main();
extern "C" void lora_task( void *);

void lora_task( void* param )
{
    LoRa lora( PIN_NUM_MOSI, PIN_NUM_MISO, PIN_NUM_CLK, PIN_NUM_CS, RESET_PIN, PIN_NUM_DIO, 17 );

    generate_unique_id(); // Generar ID y determinar rol HOST/RECEPTOR

    if (is_host) {
        delay(1000); // Espera un segundo antes de enviar el siguiente mensaje
        printf("Soy el HOST. Iniciando comunicación...\n");
        host_send_delay = send_message_and_get_delay(&lora, get_nanotime()); // Envía el primer mensaje con su propio timestamp
        printf("(HOST) Time it takes to send (first message): %llu ns\n", host_send_delay);
        // delay(10); // Pequeña pausa para que el receptor tenga tiempo de ponerse en modo RX
    }

    for ( ;; ) // Bucle infinito
    {
        lora.receive(0); // Pone LoRa en modo de recepción
        printf("Esperando mensajes... (Timeout: %d segundos)\n", LORA_RECEIVE_TIMEOUT_US / 1000000);
        
        bool message_received = false;
        uint64_t receive_start_time = esp_timer_get_time(); // Tiempo de inicio de la espera RX

        while ((esp_timer_get_time() - receive_start_time) < LORA_RECEIVE_TIMEOUT_US)
        {
            if (lora.getDataReceived())
            {
                uint64_t ts_recepcion_local = get_nanotime(); // Timestamp al recibir la interrupción
                uint64_t ts_en_paquete = 0; // El timestamp que viene en el paquete

                // Manejar el mensaje recibido
                // Si handle_message devuelve 0, significa error o paquete corrupto
                if (handle_message(&lora, &ts_en_paquete, &ts_recepcion_local) == 0) {
                    continue;
                }
                // Lógica y cálculo de Latencia/RTT

                if (is_host) {
                    // HOST recibe un mensaje y espera el segundo mensaje
                    receive_start_time = esp_timer_get_time(); // Tiempo de inicio de la espera RX
                    while ((esp_timer_get_time() - receive_start_time) < LORA_RECEIVE_TIMEOUT_US)
                    {
                        if (lora.getDataReceived())
                        {
                            uint64_t receptor_send_delay = 0; // Timestamp que viene en el paquete

                            // Manejar el mensaje recibido
                            // Si handle_message devuelve 0, significa error o paquete corrupto
                            if (handle_message(&lora, &receptor_send_delay) == 0) {
                                break;
                            }
                            // HOST recibe un mensaje, calcula el RTT y la distancia
                            uint64_t rtt = ts_recepcion_local - ts_en_paquete - host_send_delay - receptor_send_delay;
                            if (rtt < MIN_ACCEPTABLE_RTT_NS || rtt > MAX_ACCEPTABLE_RTT_NS) {
                                printf("ADVERTENCIA: RTT %llu ns (%.3f ms) fuera de rango aceptable. Descartando.\n", rtt, (double)rtt / 1000000.0);
                                break; 
                            }

                            printf("==================================================\n");
                            printf("  HOST: ¡Respuesta Recibida! RTT: %llu ns (%.3f ms)\n", rtt, (float)rtt / 1000000.0);
                            printf("==================================================\n");

                            // Calibración dinámica de base_rtt_offset_ns
                            if (!initial_calibration_done && tof_buffer_idx < NUM_CALIBRATION_SAMPLES) {
                                tof_buffer[tof_buffer_idx++] = (double)rtt; // Guardamos el RTT completo para calibración
                                if (tof_buffer_idx == NUM_CALIBRATION_SAMPLES) {
                                    AvgStdDevResult calibration_result = calculate_avg_stddev(tof_buffer, NUM_CALIBRATION_SAMPLES);
                                    base_rtt_offset_ns = calibration_result.average;
                                    initial_calibration_done = true;
                                    tof_buffer_idx = 0; // Resetear buffer para TOFs reales
                                    printf("\n--- CALIBRACIÓN INICIAL COMPLETA ---\n");
                                    printf("Base RTT Offset (Hardware Delay) Calibrado: %.3f ns (%.3f ms)\n", base_rtt_offset_ns, base_rtt_offset_ns / 1000000.0);
                                    printf("-----------------------------------\n\n");
                                }
                                break; 
                            }

                             // Si la calibración inicial está hecha, procedemos con el cálculo de TOF
                            if (initial_calibration_done) {
                                double tof = (double)rtt - base_rtt_offset_ns; // Se resta el offset
                                
                                // Asegurarse de que el TOF sea positivo y que el buffer no esté lleno
                                if (tof > 0 && tof_buffer_idx < DISTANCE_BUFFER_SIZE) { 
                                    tof_buffer[tof_buffer_idx++] = tof;

                                    // Cuando el buffer de TOF esté lleno
                                    if (tof_buffer_idx == DISTANCE_BUFFER_SIZE) {
                                        printf("\n#########################\n");
                                        printf("Calculando promedio y desviación estándar para %zu valores de TOF...\n", tof_buffer_idx);
                                        
                                        // Filtrado de outliers por desviación estándar
                                        double filtered_tof_buffer[DISTANCE_BUFFER_SIZE];
                                        size_t filtered_count = 0;
                                        AvgStdDevResult current_avg_stddev = calculate_avg_stddev(tof_buffer, tof_buffer_idx);

                                        for (size_t i = 0; i < tof_buffer_idx; i++) {
                                            if (fabs(tof_buffer[i] - current_avg_stddev.average) <= OUTLIER_STD_DEV_FACTOR * current_avg_stddev.std_deviation) {
                                                filtered_tof_buffer[filtered_count++] = tof_buffer[i];
                                            } else {
                                                printf("  Descartando outlier TOF: %.3f ns (afuera de %f std. dev.)\n", tof_buffer[i], OUTLIER_STD_DEV_FACTOR);
                                            }
                                        }

                                        // Recalcular promedio y desviación estándar con los valores filtrados
                                        AvgStdDevResult final_tof_result = calculate_avg_stddev(filtered_tof_buffer, filtered_count);
                                        
                                        // Añadir el resultado al buffer de mediciones (código existente)
                                        if (measurements_buffer_idx < MEASURE_BUFFER_SIZE) {
                                            measurements_buffer[measurements_buffer_idx++] = final_tof_result;
                                        } else {
                                            printf("ADVERTENCIA: measurements_buffer está lleno, descartando nuevo promedio/desviación estándar.\n");
                                        }
                                        
                                        tof_buffer_idx = 0; 
                                        printf("#########################\n\n");
                                    }
                                }
                            }

                            // Cuando el buffer de mediciones promediadas esté lleno, seleccionar la mejor distancia
                            if (measurements_buffer_idx == MEASURE_BUFFER_SIZE) {
                                printf("\nObteniendo la mejor distancia de %zu mediciones de TOF promediadas...\n", measurements_buffer_idx);
                                // Ordenar el arreglo de estructuras por la desviación estándar del TOF (usando qsort)
                                qsort(measurements_buffer, MEASURE_BUFFER_SIZE, sizeof(AvgStdDevResult), compare_avg_stddev_results);
                                
                                printf("Buffer de mediciones (Promedio TOF, Desv. Est. TOF):\n");
                                for (size_t i = 0; i < MEASURE_BUFFER_SIZE; i++) {
                                    printf("    {Prom: %.9f ns, Desv. Est.: %.9f ns}\n", measurements_buffer[i].average, measurements_buffer[i].std_deviation);
                                }
                                printf("\n");

                                // El mejor promedio de TOF es el que tiene la menor desviación estándar
                                double best_avg_tof = measurements_buffer[0].average;
                                // Convertir este promedio de TOF a distancia
                                // double best_distance_m = (best_avg_tof / 1000000000.0) * 299792458.0; // Velocidad de la luz en m/s
                                double best_distance_m = (best_avg_tof / 1000000000.0) * 299792458.0 / 2.0; 

                                printf("Mejor distancia estimada (Menor Desv. Est. de TOF): %.3f m (Promedio TOF: %.9f ns, Desv. Est. TOF: %.9f ns)\n", 
                                       best_distance_m, measurements_buffer[0].average, measurements_buffer[0].std_deviation);
                                
                                // Limpiar el índice del buffer de mediciones para la próxima serie de promedios
                                measurements_buffer_idx = 0; 
                                printf("\n");
                            }

                                continue_sending = true; // Indica que el HOST puede continuar enviando mensajes
                                message_received = true;
                                break; // Salir del bucle de espera, ya se recibió y procesó un mensaje
                            }
                        }
                        
                    } else { 
                    // RECEPTOR recibe un mensaje del HOST, lo procesa y responde.
                    uint64_t timestamp_antes_envio_receptor = get_nanotime(); // Timestamp antes de enviar la respuesta

                    // Calcular el tiempo que el RECEPTOR tardó en procesar el mensaje
                    uint64_t processing_delay_receptor = timestamp_antes_envio_receptor - ts_recepcion_local;
                    printf("  RECEPTOR: Retardo de Procesamiento: %llu ns\n", processing_delay_receptor);

                    // El timestamp para enviar de vuelta al HOST es:
                    // el timestamp original del HOST (recibido en el paquete) + el tiempo de procesamiento del RECEPTOR
                    uint64_t timestamp_para_host = ts_en_paquete + processing_delay_receptor;
                    printf("  RECEPTOR: Enviando respuesta con TS ajustado: %llu ns\n", timestamp_para_host);

                    receptor_send_delay = send_message_and_get_delay(&lora, timestamp_para_host); // Envía el mensaje con el timestamp ajustado
                    printf("(RECEPTOR) Time it takes to send: %llu ns\n", receptor_send_delay);

                    delay(10); // Pequeña pausa

                    send_message_and_get_delay(&lora, receptor_send_delay);
                }
                message_received = true;
                break; // Salir del bucle de espera, ya se recibió y procesó un mensaje
            }
            // delay(10);
        }

        if (continue_sending && is_host) {
            delay(1000); // Espera un segundo antes de enviar el siguiente mensaje
            // Si el HOST puede seguir enviando mensajes, envía un nuevo mensaje
            host_send_delay = send_message_and_get_delay(&lora, get_nanotime()); // Envía un nuevo mensaje con su propio timestamp
            printf("(HOST) Time it takes to send (continue): %llu ns\n", host_send_delay);
            continue_sending = false; // Reinicia la variable para el próximo ciclo
            // delay(10); // Pequeña pausa para no saturar el bus SPI
            continue;
        }

        // Si no se recibió ningún mensaje dentro del timeout
        if (!message_received) {
            if (is_host) {
                // El HOST no recibe un mensaje, envía un nuevo mensaje para reiniciar la cadena.
                printf("HOST: Timeout de recepción. Enviando un nuevo mensaje para reiniciar la cadena...\n");
                host_send_delay = send_message_and_get_delay(&lora, get_nanotime()); // Envía un nuevo mensaje con su propio timestamp
                printf("(HOST) Time it takes to send (restart): %llu ns\n", host_send_delay);
                // delay(10);
                continue;
            } else {
                // El RECEPTOR no debe iniciar la comunicación a menos que reciba un mensaje.
                // Si hay timeout, simplemente vuelve a esperar.
                printf("RECEPTOR: Timeout de recepción. Volviendo a esperar...\n");
                continue;
            }
        }
    }
}

// Función principal de la aplicación
extern "C" void app_main()
{
    xTaskCreate(lora_task, "lora_task", 10000, NULL, 1, NULL);
}
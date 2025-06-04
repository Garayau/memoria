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

#include "LoRa.h" // Asegúrate de que esta librería LoRa sea compatible con ESP32

#define PIN_NUM_CLK  	18
#define PIN_NUM_MISO 	19
#define PIN_NUM_MOSI 	23
#define PIN_NUM_CS   	4
#define PIN_NUM_DIO		26
#define RESET_PIN  		15

#define	FLASH_PIN			2 // Pin para indicar actividad (opcional)

#define UNIQUE_ID_MAX_LEN 20  // Longitud máxima para la cadena de ID

char unique_id[UNIQUE_ID_MAX_LEN];  // Variable global para almacenar el ID único
int _counter = 0; // Contador de mensajes enviados

// Define si este ESP32 es el "Host" (el que inicia la comunicación)
// Esto se determinará por el último byte de la MAC.
bool is_host = true;
bool continue_sending = false; // Variable para controlar si el Host puede seguir enviando mensajes

// Tiempo máximo de espera en microsegundos para recibir un mensaje antes de intentar enviar
#define LORA_RECEIVE_TIMEOUT_US 3000000 // 3 segundos

// Almacena el timestamp del momento justo después del envío de un mensaje del Host
uint64_t host_send_delay = 0;
// Almacena el timestamp del momento justo después del envío de un mensaje del Receptor
uint64_t receptor_send_delay = 0;


// --- Funciones Auxiliares ---

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
void handle_message(uint64_t *ts_en_paquete, uint64_t *ts_recepcion_local=nullptr) {
    char msg_content[100] = {0};
    
    int packetSize = lora.handleDataReceived(msg_content, ts_en_paquete);
    lora.setDataReceived(false);
    
    if (packetSize == 0) { // handleDataReceived devuelve 0 si hay error o paquete corrupto
        printf("Paquete inválido o error de CRC.\n");
        continue;
    }
    
    printf("\n<-- [RX] Mensaje Recibido: '%s'\n", msg_content);
    printf("    Timestamp en Paquete: %llu ns\n", *ts_en_paquete);
    if (ts_recepcion_local != nullptr) {
        printf("    Timestamp Recepción Local (RX_DONE): %llu ns\n", *ts_recepcion_local);
    }
    printf("    RSSI: %d dBm, SNR: %d dB\n", lora.getPacketRssi(), lora.getPacketSnr());
}

// --- Función Principal ---

extern "C" void app_main();
extern "C" void lora_task( void *);

void lora_task( void* param )
{
	LoRa lora( PIN_NUM_MOSI, PIN_NUM_MISO, PIN_NUM_CLK, PIN_NUM_CS, RESET_PIN, PIN_NUM_DIO, 17 );

    generate_unique_id(); // Generar ID y determinar rol HOST/RECEPTOR

    if (is_host) {
        printf("Soy el HOST. Iniciando comunicación...\n");
        host_send_delay = send_message_and_get_delay(&lora, get_nanotime()); // Envía el primer mensaje con su propio timestamp
        printf("(HOST) Time it takes to send (first message): %llu ns\n", host_send_delay);
        delay(10); // Pequeña pausa para que el receptor tenga tiempo de ponerse en modo RX
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
                uint64_t timestamp_recepcion_local = get_nanotime(); // Timestamp al recibir la interrupción
                uint64_t ts_en_paquete = 0; // El timestamp que viene en el paquete

                // Manejar el mensaje recibido
                handle_message(&ts_en_paquete, &timestamp_recepcion_local);
                
                // --- Lógica de Roles y Cálculo de Latencia/RTT ---

                if (is_host) {
                    // HOST recibe un mensaje y espera el segundo mensaje
                    receive_start_time = esp_timer_get_time(); // Tiempo de inicio de la espera RX
                    while ((esp_timer_get_time() - receive_start_time) < LORA_RECEIVE_TIMEOUT_US)
                    {
                        if (lora.getDataReceived())
                        {
                            uint64_t receptor_send_delay = 0; // Timestamp que viene en el paquete

                            // Manejar el mensaje recibido 
                            handle_message(&receptor_send_delay);

                            // HOST recibe un mensaje, calcula el RTT y la distancia
                            uint64_t rtt = timestamp_recepcion_local - ts_en_paquete - host_send_delay - receptor_send_delay;
                            double hardware_delay = 0; // Retardo estimado en nanosegundos
                            double tof = (rtt - hardware_delay) / (2.0 * 1000000000.0); // Tiempo de vuelo en segundos
                            double distance_m = tof * 299792458.0;
                            printf("==================================================\n");
                            printf("  HOST: ¡Respuesta Recibida! RTT: %llu ns (%.3f ms)\n", rtt, (float)rtt / 1000000.0);
                            printf("==================================================\n");
                            printf("  HOST: Distancia estimada: %f m\n", distance_m);
                            printf("==================================================\n");

                            continue_sending = true; // Indica que el HOST puede continuar enviando mensajes
                            message_received = true;
                            break; // Salir del bucle de espera, ya se recibió y procesó un mensaje

                        }
                    }

                } else { 
                    // RECEPTOR recibe un mensaje del HOST, lo procesa y responde.
                    uint64_t timestamp_antes_envio_receptor = get_nanotime(); // Timestamp antes de enviar la respuesta

                    // Calcular el tiempo que el RECEPTOR tardó en procesar el mensaje
                    uint64_t processing_delay_receptor = timestamp_antes_envio_receptor - timestamp_recepcion_local;
                    printf("  RECEPTOR: Retardo de Procesamiento: %llu ns\n", processing_delay_receptor);

                    // El timestamp para enviar de vuelta al HOST es:
                    // el timestamp original del HOST (recibido en el paquete) + el tiempo de procesamiento del RECEPTOR
                    uint64_t timestamp_para_host = timestamp_en_paquete_recibido + processing_delay_receptor;
                    printf("  RECEPTOR: Enviando respuesta con TS ajustado: %llu ns\n", timestamp_para_host);

                    receptor_send_delay = send_message_and_get_delay(&lora, timestamp_para_host); // Envía el mensaje con el timestamp ajustado
                    printf("(RECEPTOR) Time it takes to send: %llu ns\n", receptor_send_delay);

                    delay(10); // Pequeña pausa

                    send_message_and_get_delay(&lora, receptor_send_delay);
                }
                message_received = true;
                break; // Salir del bucle de espera, ya se recibió y procesó un mensaje
            }
            delay(10); // Pequeña pausa para no bloquear la CPU
        }

        if (continue_sending && is_host) {
            delay(1000); // Espera un segundo antes de enviar el siguiente mensaje
            
            // Si el HOST puede seguir enviando mensajes, envía un nuevo mensaje
            host_send_delay = send_message_and_get_delay(&lora, get_nanotime()); // Envía un nuevo mensaje con su propio timestamp
            printf("(HOST) Time it takes to send (continue): %llu ns\n", host_send_delay);
            continue_sending = false; // Reinicia la variable para el próximo ciclo
            delay(10); // Pequeña pausa para no saturar el bus SPI
            continue;
        }

        // Si no se recibió ningún mensaje dentro del timeout
        if (!message_received) {
            if (is_host) {
                // El HOST no recibe un mensaje, envía un nuevo mensaje para reiniciar la cadena.
                printf("HOST: Timeout de recepción. Enviando un nuevo mensaje para reiniciar la cadena...\n");
                host_send_delay = send_message_and_get_delay(&lora, get_nanotime()); // Envía un nuevo mensaje con su propio timestamp
                printf("(HOST) Time it takes to send (restart): %llu ns\n", host_send_delay);
                delay(10);
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
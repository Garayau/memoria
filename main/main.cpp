#ifndef ESP_INTR_FLAG_DEFAULT
#define ESP_INTR_FLAG_DEFAULT 0
#endif
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_flash.h"
#include <inttypes.h>
#include "LoRa.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"



// Definiciones de pines para el ESP32
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   4
#define RESET_PIN    12
#define PIN_NUM_DIO  26 // DIO0

// Potencia de transmisión
#define TX_POWER 17

static const char *TAG = "MAIN";

// Function to get time in nanoseconds (using esp_timer_get_time)
uint64_t get_time_ns() {
    return esp_timer_get_time() * 1000;
}

// Forward declaration for printLoRaConfig
void printLoRaConfig(LoRa& lora_obj, const char* context);

extern "C" void app_main(void)
{
    // Es crucial instalar el servicio ISR para GPIO antes de añadir manejadores
    esp_err_t ret = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) { // ESP_ERR_INVALID_STATE means it's already installed
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        return;
    }
    
    // Primero inicializamos el objeto LoRa y sus pines SPI/GPIO
    LoRa lora(PIN_NUM_MOSI, PIN_NUM_MISO, PIN_NUM_CLK, PIN_NUM_CS, RESET_PIN, PIN_NUM_DIO, TX_POWER);

    // --- NUEVO FLUJO DE INICIALIZACIÓN CRÍTICO Y ROBUSTO ---
    lora.initialize(TX_POWER); // Realiza el setup básico de hardware y un primer check de versión

    printf("MAIN: Iniciando proceso de configuración LoRa robusta...\n");
    int config_attempts = 0;
    const int MAX_CONFIG_ATTEMPTS = 10; // Aumentar intentos para mayor robustez
    bool config_ok = false;

    while (config_attempts < MAX_CONFIG_ATTEMPTS && !config_ok) {
        printf("MAIN: Intento de configuración LoRa #%d...\n", config_attempts + 1);
        lora.resetRegisters(); // Forzar una reconfiguración completa

        // Leer los valores de los registros críticos después del reset
        uint8_t reg_modem_config_1_val = lora.getRegisterValue(REG_MODEM_CONFIG_1);
        uint8_t reg_modem_config_2_val = lora.getRegisterValue(REG_MODEM_CONFIG_2);
        long current_freq = lora.getFrequencyValue();
        
        printf("  Verificación: REG_MODEM_CONFIG_1: 0x%02x (Esperado: 0x82)\n", reg_modem_config_1_val);
        printf("  Verificación: REG_MODEM_CONFIG_2: 0x%02x (Esperado: 0x94)\n", reg_modem_config_2_val);
        printf("  Verificación: Frecuencia: %ld (Esperado: 915000000)\n", current_freq);

        if (reg_modem_config_1_val == 0x82 && 
            reg_modem_config_2_val == 0x94 &&
            current_freq == 915000000) { // Verificar también la frecuencia
            config_ok = true;
            printf("MAIN: Configuración LoRa exitosa después de %d intentos.\n", config_attempts + 1);
        } else {
            printf("MAIN: Configuración LoRa fallida. Reintentando...\n");
            vTaskDelay(pdMS_TO_TICKS(100)); // Pequeño retraso antes del siguiente intento
        }
        config_attempts++;
    }

    if (!config_ok) {
        printf("ERROR: MAIN: Fallo al configurar el módulo LoRa después de %d intentos. Revisar hardware y pines.\n", MAX_CONFIG_ATTEMPTS);
        while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); } // Detener ejecución
    }
    printLoRaConfig(lora, "After Mandatory Final LoRa Configuration Loop");
    // --- FIN FLUJO DE INICIALIZACIÓN CRÍTICO Y ROBUSTO ---

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %" PRIu32 " CPU cores, WiFi%s%s, ",
           (uint32_t)chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %" PRIu32 ", ", (uint32_t)chip_info.revision);
    
    uint32_t raw_flash_size;
    esp_flash_get_size(NULL, &raw_flash_size);
    uint32_t flash_size_mb = raw_flash_size / (1024 * 1024);
    
    printf("%" PRIu32 "MB %s flash\n", flash_size_mb,
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    printf("I (%" PRIu32 ") UID: ESP32 Unique ID: ESP32-%02X%02X%02X%02X%02X%02X\n",
           esp_log_timestamp(), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    bool is_host = false;
    // Asignación de rol basada en una MAC específica (ej: MAC del HOST)
    if (mac[3] == 0x10 && mac[4] == 0xE1 && mac[5] == 0xCC) {
        is_host = true;
        printf("I (%" PRIu32 ") ROLE: Este ESP32 es el HOST.\n", esp_log_timestamp());
    } else {
        printf("I (%" PRIu32 ") ROLE: Este ESP32 es el RECEPTOR.\n", esp_log_timestamp());
    }

    printLoRaConfig(lora, "After Role Determination (Should be correct)");

    if (is_host) {
        printf("Soy el HOST. Iniciando comunicación...\n");
        int message_count = 0;
        while (1) {
            uint64_t start_time_tx = get_time_ns();
            char tx_msg_buffer[100];
            uint64_t tx_timestamp = get_time_ns();

            memcpy(tx_msg_buffer, &tx_timestamp, sizeof(uint64_t));
            snprintf(tx_msg_buffer + sizeof(uint64_t), sizeof(tx_msg_buffer) - sizeof(uint64_t),
                     "ESP32-%02X%02X%02X%02X%02X%02X: Mensaje No. [%d]",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], message_count);

            printf("--> [TX] Enviando TS: %" PRIu64 ", Contenido: '%s'\n", tx_timestamp, tx_msg_buffer + sizeof(uint64_t));
            
            lora.beginPacket(false);
            lora.write((uint8_t*)tx_msg_buffer, strlen(tx_msg_buffer + sizeof(uint64_t)) + sizeof(uint64_t));
            lora.endPacket(false); // endPacket ahora llama a resetRegisters()

            printf("(HOST) Time it takes to send (first message): %" PRIu64 " ns\n", get_time_ns() - start_time_tx);
            
            printf("Esperando mensajes... (Timeout: 3 segundos)\n");

            printLoRaConfig(lora, "After TX and Internal Reset");

            uint64_t start_time_rx_host = get_time_ns();
            char rx_msg_buffer_host[100];
            uint64_t rx_timestamp_host = 0;
            lora.receive(0); // Llama a receive() para poner en modo RX

            while (!lora.getDataReceived() && (get_time_ns() - start_time_rx_host) < (3 * 1000 * 1000 * 1000ULL)) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            if (lora.getDataReceived()) {
                lora.setDataReceived(false);
                int packetSize = lora.parsePacket(0); // Esto debería dejar el módulo en IDLE
                if (packetSize) {
                    printf("(HOST) Mensaje recibido! RSSI: %d, SNR: %d\n", lora.getPacketRssi(), lora.getPacketSnr());
                    uint64_t processing_start_time = get_time_ns();
                    lora.handleDataReceived(rx_msg_buffer_host, &rx_timestamp_host); // handleDataReceived ahora llama a resetRegisters
                    uint64_t processing_end_time = get_time_ns();
                    uint64_t processing_latency = processing_end_time - processing_start_time;
                    printf("  HOST: Retardo de Procesamiento: %" PRIu64 " ns\n", processing_latency);

                    uint64_t current_time_host = get_time_ns();
                    int64_t latency = current_time_host - rx_timestamp_host;
                    printf("HOST: Mensaje recibido: '%s', TS original: %" PRIu64 ", TS actual: %" PRIu64 ", Latencia: %" PRId64 " ns\n",
                           rx_msg_buffer_host, rx_timestamp_host, current_time_host, latency);
                }
            } else {
                printf("HOST: Timeout de recepción. Enviando un nuevo mensaje para reiniciar la cadena...\n");
            }

            message_count++;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    } else { // RECEPTOR
        printf("Soy el RECEPTOR. Esperando mensajes...\n");
        
        int response_count = 0;
        while (1) {
            printf("Esperando mensajes... (Timeout: 3 segundos)\n");
            lora.receive(0); // Pone el módulo en modo RX_CONTINUOUS

            uint64_t start_time_rx_client = get_time_ns();
            char rx_msg_buffer_client[100];
            uint64_t rx_timestamp_client = 0;

            while (!lora.getDataReceived() && (get_time_ns() - start_time_rx_client) < (3 * 1000 * 1000 * 1000ULL)) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            if (lora.getDataReceived()) {
                lora.setDataReceived(false);
                printf("RECEPTOR: Mensaje recibido. Procesando...\n");
                int packetSize = lora.parsePacket(0); // Esto debería dejar el módulo en IDLE
                if (packetSize) {
                    uint64_t processing_start_time = get_time_ns();
                    lora.handleDataReceived(rx_msg_buffer_client, &rx_timestamp_client); // handleDataReceived ahora llama a resetRegisters
                    uint64_t processing_end_time = get_time_ns();
                    uint64_t processing_latency = processing_end_time - processing_start_time;
                    printf("  RECEPTOR: Retardo de Procesamiento: %" PRIu64 " ns\n", processing_latency);

                    uint64_t adjusted_tx_timestamp = rx_timestamp_client + processing_latency;
                    printf("  RECEPTOR: Enviando respuesta con TS ajustado: %" PRIu64 " ns\n", adjusted_tx_timestamp);

                    uint64_t start_time_tx_client = get_time_ns();
                    char tx_response_buffer[100];
                    
                    memcpy(tx_response_buffer, &adjusted_tx_timestamp, sizeof(uint64_t));
                    snprintf(tx_response_buffer + sizeof(uint64_t), sizeof(tx_response_buffer) - sizeof(uint64_t),
                             "ESP32-%02X%02X%02X%02X%02X%02X: Mensaje No. [%d]",
                             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], response_count);

                    lora.beginPacket(false); // Pone en IDLE, luego en TX
                    lora.write((uint8_t*)tx_response_buffer, strlen(tx_response_buffer + sizeof(uint64_t)) + sizeof(uint64_t));
                    lora.endPacket(false); // endPacket ahora llama a resetRegisters()

                    printf("(RECEPTOR) Time it takes to send (first response): %" PRIu64 " ns\n", get_time_ns() - start_time_tx_client);

                    response_count++;
                }
            } else {
                printf("RECEPTOR: Timeout de recepción. Reintentando...\n");
            }

            printLoRaConfig(lora, "Before Next Action"); // Verifica el estado antes de la siguiente iteración
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// Function for printing LoRa config (now takes LoRa object by reference)
void printLoRaConfig(LoRa& lora_obj, const char* context) {
    printf("LoRa Config Check (%s):\n", context);
    printf("  REG_MODEM_CONFIG_1: 0x%02x\n", lora_obj.getRegisterValue(REG_MODEM_CONFIG_1));
    printf("  REG_MODEM_CONFIG_2: 0x%02x\n", lora_obj.getRegisterValue(REG_MODEM_CONFIG_2));
    printf("  REG_PA_CONFIG: 0x%02x\n", lora_obj.getRegisterValue(REG_PA_CONFIG));
    printf("  REG_PA_DAC: 0x%02x\n", lora_obj.getRegisterValue(REG_PA_DAC));
    printf("  REG_FRF_MSB: 0x%02x, MID: 0x%02x, LSB: 0x%02x (Freq: %ld)\n",
           lora_obj.getRegisterValue(REG_FRF_MSB),
           lora_obj.getRegisterValue(REG_FRF_MID),
           lora_obj.getRegisterValue(REG_FRF_LSB),
           lora_obj.getFrequencyValue());
}

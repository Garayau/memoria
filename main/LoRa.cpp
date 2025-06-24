#include <stdio.h>
#include "sdkconfig.h"
#include "LoRa.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"  // Para esp_rom_delay_us
#include "driver/gpio.h"
#include "driver/spi_master.h"

static const char *TAG = "LoRa"; 

// Constructor: Inicializa miembros y configura SPI/GPIO
LoRa::LoRa( int mosi, int miso, int clk, int cs, int reset, int dio, int power ) :
    _spi(nullptr),                 // Inicializado a nullptr
    _packetIndex(0),
    _implicitHeaderMode(0),
    _dataReceived(false),
    _frequency(0),                 // Inicializa _frequency a un valor conocido
    _power(power)                  // Inicializa _power con el valor pasado al constructor
{
    initializeSPI( mosi, miso, clk, cs );
    initializeReset( reset );
    initializeDIO( dio );
    // initialize(power); // Eliminado: initialize() se llamará desde main.cpp
}

long LoRa::getFrequencyValue()
{
    return _frequency;
}

uint8_t LoRa::getRegisterValue(uint8_t reg)
{
    return readRegister(reg);
}

// Función para reconfigurar el módulo LoRa de forma robusta
void LoRa::resetRegisters() {
    printf("DEBUG: LoRa::resetRegisters() - Reconfigurando LoRa...\n");
    idle(); // Poner en modo standby para poder cambiar registros
    delay(200); // Retardo después del idle para estabilidad

    // Realiza TODAS las configuraciones aquí
    setFrequency(915E6); // Asegurarse de usar la frecuencia base
    setTxPower(_power, PA_BOOST); // Usa _power (establecida en el constructor) y PA_BOOST
    setCRC(true); // Habilita CRC
    setSignalBandwidth(250E3); // Ancho de banda de 250 kHz
    setSpreadingFactor(9); // Spreading Factor de 9

    printf("DEBUG: LoRa::resetRegisters() - Reconfiguración completa.\n");
}


void LoRa::initializeSPI( int mosi, int miso, int clk, int cs )
{
    esp_err_t ret;

    spi_bus_config_t buscfg;
    memset( &buscfg, 0, sizeof(spi_bus_config_t) );

    buscfg.mosi_io_num = mosi;
    buscfg.miso_io_num = miso;
    buscfg.sclk_io_num = clk;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = MAX_PKT_LENGTH + 1; // Tamaño máximo de transferencia
    buscfg.flags = 0;
    buscfg.intr_flags = 0;


    spi_device_interface_config_t devcfg;
    memset( &devcfg, 0, sizeof(spi_device_interface_config_t) );

    devcfg.address_bits = 8; // Modo de 8 bits para la dirección del registro
    devcfg.mode = 0; // SPI Mode 0
    devcfg.clock_speed_hz = 10 * 1000 * 1000; // 10 MHz
    devcfg.spics_io_num = cs; // Pin CS
    devcfg.flags = 0;
    devcfg.queue_size = 1;


    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO); // Usar SPI_DMA_CH_AUTO para asignación automática de DMA
    ESP_ERROR_CHECK(ret);
    printf("Bus Init: %d\n", ret);

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &_spi);
    ESP_ERROR_CHECK(ret);
    printf("Add device: %d\n", ret);

}

void LoRa::initializeReset( int reset )
{
    gpio_num_t r = (gpio_num_t) reset;

    gpio_reset_pin(r);
    gpio_set_direction(r, GPIO_MODE_OUTPUT);

    gpio_set_level(r, 0);
    delay(50); // Mantenemos el reset bajo por 50ms (vTaskDelay)
    gpio_set_level(r, 1);
    delay(50); // Esperamos 50ms después de liberar el reset (vTaskDelay)
    esp_rom_delay_us(200000); // Retardo adicional en microsegundos después de liberar el reset
}

extern "C" {
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    LoRa *s = (LoRa*) arg;
    s->setDataReceived(true);
}
}

void LoRa::initializeDIO( int dio )
{
    gpio_config_t io_conf;
    gpio_num_t pin = (gpio_num_t) dio;

    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << pin );
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;

    gpio_config(&io_conf);

    gpio_isr_handler_add( pin, gpio_isr_handler, (void*) this );
}

// Esta función realiza una inicialización mínima y una primera lectura de versión.
// La configuración LoRa robusta se hace en resetRegisters().
void LoRa::initialize( int power )
{
    // put in sleep mode
    sleep();
    esp_rom_delay_us(500000); // Añadido: Retardo significativo después de sleep para estabilización

    // print version
    uint8_t version = readRegister(REG_VERSION);
    printf( "Version: [%d]\n", version );

    // ** COMPROBACIÓN CRÍTICA DE SPI **
    if (version == 0x00 || version == 0xFF) { // 0x00 o 0xFF son valores comunes para SPI fallido
        printf("ERROR: Módulo LoRa no encontrado o comunicación SPI fallida! Versión leída: 0x%02x. ¡Revisa el cableado!\n", version);
        while(1) { vTaskDelay(pdMS_TO_TICKS(100)); } // Detiene la ejecución para depuración
    }
    // La versión esperada para chips SX127x es 0x12 (18 en decimal)
    if (version != 0x12) {
        printf("ADVERTENCIA: Versión de módulo LoRa inesperada 0x%02x. Se esperaba 0x12. Continuando de todas formas.\n", version);
    }
    // ** FIN DE COMPROBACIÓN **

    // set base addresses (esto es seguro de hacer aquí)
    writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
    writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

    // set LNA boost (seguro aquí)
    writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

    // set auto AGC (seguro aquí)
    writeRegister(REG_MODEM_CONFIG_3, 0x04);

    // put in standby mode
    idle();
    delay(500); // Retardo después del idle inicial en initialize()
    
    printf("LoRa Config Check (End of Initialize - Preliminary):\n");
    printf("  REG_MODEM_CONFIG_1: 0x%02x\n", readRegister(REG_MODEM_CONFIG_1));
    printf("  REG_MODEM_CONFIG_2: 0x%02x\n", readRegister(REG_MODEM_CONFIG_2));
    printf("  REG_PA_CONFIG: 0x%02x\n", readRegister(REG_PA_CONFIG));
    printf("  REG_PA_DAC: 0x%02x\n", readRegister(REG_PA_DAC));
    printf("  REG_FRF_MSB: 0x%02x, MID: 0x%02x, LSB: 0x%02x (Freq: %ld)\n",
    readRegister(REG_FRF_MSB), readRegister(REG_FRF_MID), readRegister(REG_FRF_LSB), _frequency);

    // Las configuraciones de Frecuencia, Potencia, CRC, BW, SF se harán en resetRegisters()
    // después de esta inicialización en main.
}

void LoRa::sleep()
{
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void LoRa::idle()
{
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LoRa::setFrequency(long frequency)
{
    idle(); // Asegura que el módulo esté en STDBY antes de configurar
    printf("DEBUG Freq: Current OP_MODE before setting freq: 0x%02x\n", readRegister(REG_OP_MODE));
    _frequency = frequency; // CRÍTICO: Guarda la frecuencia en el miembro de la clase

    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
    printf("DEBUG Freq: Input Freq=%ld, FRF_Val=0x%llx\n", frequency, frf);

    writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
    esp_rom_delay_us(200); // Retardo de 200 microsegundos
    printf("DEBUG Freq: Wrote FRF_MSB=0x%02x, ReadBack=0x%02x\n", (uint8_t)(frf >> 16), readRegister(REG_FRF_MSB));

    writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
    esp_rom_delay_us(200); // Retardo de 200 microsegundos
    printf("DEBUG Freq: Wrote FRF_MID=0x%02x, ReadBack=0x%02x\n", (uint8_t)(frf >> 8), readRegister(REG_FRF_MID));

    writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
    esp_rom_delay_us(200); // Retardo de 200 microsegundos
    printf("DEBUG Freq: Wrote FRF_LSB=0x%02x, ReadBack=0x%02x\n", (uint8_t)(frf >> 0), readRegister(REG_FRF_LSB));
}

void LoRa::setSignalBandwidth(long bw) {
    idle(); // Asegura que esté en STDBY antes de configurar
    printf("DEBUG BW: OP_MODE before write REG_MODEM_CONFIG_1: 0x%02x\n", readRegister(REG_OP_MODE));
    uint8_t bwCode = 0;

    if (bw <= 7.8E3) {
        bwCode = 0x0;
    } else if (bw <= 10.4E3) {
        bwCode = 0x1;
    } else if (bw <= 15.6E3) {
        bwCode = 0x2;
    } else if (bw <= 20.8E3) {
        bwCode = 0x3;
    } else if (bw <= 31.25E3) {
        bwCode = 0x4;
    } else if (bw <= 41.7E3) {
        bwCode = 0x5;
    } else if (bw <= 62.5E3) {
        bwCode = 0x6;
    } else if (bw <= 125E3) {
        bwCode = 0x7;
    } else if (bw <= 250E3) {
        bwCode = 0x8; // Código correcto para 250 kHz
    } else {
        bwCode = 0x9;
    }

    uint8_t modemConfig1 = readRegister(REG_MODEM_CONFIG_1);
    modemConfig1 &= 0x0F; // Limpia los bits del BW (bits 7-4) manteniendo los 4 bits bajos
    modemConfig1 |= (bwCode << 4); // Establece el nuevo BW

    writeRegister(REG_MODEM_CONFIG_1, modemConfig1);
    esp_rom_delay_us(200);
    printf("DEBUG BW Set: Input SBW=%ld, BW_Code=%d, Wrote REG_MODEM_CONFIG_1=0x%02x, ReadBack=0x%02x (OP_MODE after write: 0x%02x)\n", bw, bwCode, modemConfig1, readRegister(REG_MODEM_CONFIG_1), readRegister(REG_OP_MODE));
}

void LoRa::setSpreadingFactor(int sf) {
    idle(); // Asegura que esté en STDBY antes de configurar
    printf("DEBUG SF: OP_MODE before write REG_MODEM_CONFIG_2: 0x%02x\n", readRegister(REG_OP_MODE));
    if (sf < 6) {
        sf = 6;
    } else if (sf > 12) {
        sf = 12;
    }

    uint8_t modemConfig2 = readRegister(REG_MODEM_CONFIG_2);
    // Mantenemos los bits [3:0] (que incluyen el bit de CRC y SymbTimeout)
    uint8_t lower_bits = modemConfig2 & 0x0F;

    // Limpiamos los bits [7:4] del SF actual y añadimos el nuevo SF
    uint8_t newSF_val = (sf << 4);
    modemConfig2 = newSF_val | lower_bits;

    writeRegister(REG_MODEM_CONFIG_2, modemConfig2);
    esp_rom_delay_us(200);
    printf("DEBUG SF Set: Input SF=%d, Wrote REG_MODEM_CONFIG_2=0x%02x, ReadBack=0x%02x (OP_MODE after write: 0x%02x)\n", sf, modemConfig2, readRegister(REG_MODEM_CONFIG_2), readRegister(REG_OP_MODE));
}


void LoRa::setSyncWord(int sw)
{
    writeRegister(REG_SYNC_WORD, sw);
}

void LoRa::setCRC( bool crc )
{
    idle(); // Asegura que esté en STDBY antes de configurar
    printf("DEBUG CRC: OP_MODE before write REG_MODEM_CONFIG_2: 0x%02x\n", readRegister(REG_OP_MODE));
    if ( crc )
        writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
    else
        writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
    esp_rom_delay_us(200); // Retardo después de la escritura del CRC
    printf("DEBUG CRC Set: New REG_MODEM_CONFIG_2=0x%02x, ReadBack=0x%02x (OP_MODE after write: 0x%02x)\n", readRegister(REG_MODEM_CONFIG_2), readRegister(REG_MODEM_CONFIG_2), readRegister(REG_OP_MODE));
}

void LoRa::setOCP(uint8_t mA)
{
    uint8_t ocpTrim = 27;

    if (mA <= 120) ocpTrim = (mA - 45) / 5;
    else if (mA <=240) ocpTrim = (mA + 30) / 10;

    writeRegister(REG_LR_OCP, 0x20 | (0x1F & ocpTrim));
}

void LoRa::setTxPower(int8_t level, int8_t outputPin)
{
    idle(); // Asegura que esté en STDBY antes de configurar
    printf("DEBUG Power: Current OP_MODE before setting power: 0x%02x\n", readRegister(REG_OP_MODE));
    _power = level; 

    if (PA_OUTPUT_RFO_PIN == outputPin)
    {
        printf( "Setting Power to %d using RFO Pin\n", level );
        if (level < 0) level = 0;
        else if (level > 14) level = 14;

        writeRegister(REG_PA_CONFIG, 0x70 | level);
    }
    else // Using PA_BOOST pin
    {
        printf( "Setting Power to %d using Boost Pin\n", level );

        if (level > 17)
        {
            if (level > 20) level = 20; // Max 20dBm

            level -= 3; // Subtract 3 for 20dBm boost (Semtech errata)
            writeRegister(REG_PA_DAC, 0x87); // Enable high power PA_DAC for +20dBm
            setOCP(140); // Set OCP for higher current
        }
        else
        {
            if (level < 2) level = 2; // Min 2dBm
            writeRegister(REG_PA_DAC, 0x84); // Disable high power PA_DAC
            setOCP(100); // Set OCP for lower current
        }
        int paConfig = PA_BOOST | (level - 2); // Calculate PA_CONFIG value

        printf( "RegPAConfig: [%02x]\n", paConfig);
        printf( "RegPADAC: [%02x]\n", readRegister(REG_PA_DAC)); // Read back PADAC for verification

        writeRegister(REG_PA_CONFIG, paConfig );
    }

    esp_rom_delay_us(200); // Retardo después de la escritura crítica de PA_CONFIG
    printf("DEBUG TXP: Wrote PA_CONFIG = 0x%02x, Read back = 0x%02x\n", readRegister(REG_PA_CONFIG), readRegister(REG_PA_CONFIG));
    esp_rom_delay_us(200); // Retardo después de la escritura crítica de PA_DAC
    printf("DEBUG TXP: Wrote PA_DAC = 0x%02x, Read back = 0x%02x\n", readRegister(REG_PA_DAC), readRegister(REG_PA_DAC));
}


void LoRa::explicitHeaderMode()
{
  _implicitHeaderMode = 0;
  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void LoRa::implicitHeaderMode()
{
  _implicitHeaderMode = 1;
  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

int LoRa::beginPacket(int implicitHeader)
{
  idle(); // put in standby mode
  if (implicitHeader) {
    implicitHeaderMode();
  } else {
    explicitHeaderMode();
  }
  // reset FIFO address and payload length
  writeRegister(REG_FIFO_ADDR_PTR, 0);
  writeRegister(REG_PAYLOAD_LENGTH, 0);

  return 1;
}

int LoRa::endPacket(bool async)
{
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX); // put in TX mode

  if (async) {
    delay(1);
  } else {
    while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0); // wait for TX done
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK); // clear IRQ's
    resetRegisters(); // Volver a habilitado
  }

  return 1;
}

size_t LoRa::write(const uint8_t *buffer, size_t size)
{
  int currentLength = readRegister(REG_PAYLOAD_LENGTH);

  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }

  // write data
  for (size_t i = 0; i < size; i++) {
    writeRegister(REG_FIFO, buffer[i]);
  }

  // update length
  writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

  return size;
}

int LoRa::available()
{
    return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int LoRa::read()
{
    if ( !available() )
        return -1;

    _packetIndex++;

    return readRegister(REG_FIFO);
}

void LoRa::receive(int size)
{
    // Asegurarse de que el módulo esté en IDLE antes de cambiar a RX
    idle(); 

    if (size > 0)
    {
        implicitHeaderMode();
        writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    }
    else
        explicitHeaderMode();
    
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

int LoRa::getPacketRssi() {
    int16_t rssi = readRegister(REG_PKT_RSSI_VALUE);
    return rssi - (_frequency < 525E6 ? 164 : 157);
}

int LoRa::getPacketSnr() {
    return (int8_t)readRegister(0x19) / 4;
}

int LoRa::handleDataReceived( char *msg, uint64_t *timestamp_rx )
{
    int irqFlags = readRegister(REG_IRQ_FLAGS);
    writeRegister(REG_IRQ_FLAGS, irqFlags); // Limpia banderas de interrupción

    esp_rom_delay_us(100); // 100 microsegundos de retardo para estabilización

    if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
    {
        int packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);

        _packetIndex = 0; // Reset packet index
        writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

        if (packetLength < (int)sizeof(uint64_t)) {
            ESP_LOGW(TAG, "Paquete muy pequeño (%d bytes) para contener timestamp.", packetLength); // Usar TAG
            for (int i = 0; i < packetLength; i++) { read(); } // Descartar bytes restantes
            *msg = '\0';
            *timestamp_rx = 0;
            writeRegister(REG_FIFO_ADDR_PTR, 0);
            resetRegisters(); // Volver a habilitado
            return 0;
        }

        uint8_t timestamp_bytes[sizeof(uint64_t)];
        for (int i = 0; i < sizeof(uint64_t); i++) {
            timestamp_bytes[i] = read();
        }
        memcpy(timestamp_rx, timestamp_bytes, sizeof(uint64_t)); // Copiar bytes al timestamp
        
        int message_start_index = sizeof(uint64_t);
        int bytes_to_read_for_msg = packetLength - message_start_index;

        if (bytes_to_read_for_msg >= 0 && bytes_to_read_for_msg < 100) { // Asegurar no desbordar el buffer 'msg'
            for (int i = 0; i < bytes_to_read_for_msg; i++) {
                *msg++ = read();
            }
            *msg = '\0'; // Null-terminate the string
        } else {
            ESP_LOGW(TAG, "Problema con la longitud del contenido del mensaje: %d bytes. Paquete: %d, TS: %d", bytes_to_read_for_msg, packetLength, (int)sizeof(uint64_t)); // Usar TAG
            for (int i = 0; i < bytes_to_read_for_msg; i++) { read(); } // Descartar bytes restantes
            *msg = '\0';
            resetRegisters(); // Volver a habilitado
            return 0;
        }
        
        writeRegister(REG_FIFO_ADDR_PTR, 0); // Reset FIFO pointer
        resetRegisters(); // Volver a habilitado
        return packetLength;
    }
    resetRegisters(); // Volver a habilitado
    return 0; // No se recibió un paquete válido
}

int LoRa::parsePacket(int size)
{
    int packetLength = 0;

    // parsePacket no debería cambiar el modo. receive() es el que lo hace.
    // if (size > 0)
    // {
    //     implicitHeaderMode();
    //     writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    // }
    // else
    //     explicitHeaderMode();

    int irqFlags = readRegister(REG_IRQ_FLAGS);
    writeRegister(REG_IRQ_FLAGS, irqFlags); // clear IRQ's

    if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0)
    {
        _packetIndex = 0;

        if (_implicitHeaderMode) // Usar el modo que ya estaba configurado
            packetLength = readRegister(REG_PAYLOAD_LENGTH);
        else
            packetLength = readRegister(REG_RX_NB_BYTES);

        writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

        idle(); // put in standby mode after parsing
    }
    else if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE))
    {
        // Esto solo es necesario si se quiere volver a single RX mode después de un fallo
        // o si se estaba en otro modo antes de parsePacket. Para RX_CONTINUOUS no es ideal.
        // writeRegister(REG_FIFO_ADDR_PTR, 0); // reset FIFO address
        // writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE); // put in single RX mode
        // Mejor dejar la gestión del modo a receive() en el bucle principal
    }

    return packetLength;
}

void LoRa::delay( int msec )
{
    vTaskDelay( msec / portTICK_PERIOD_MS);
}

// ** FUNCIÓN writeRegister **
void LoRa::writeRegister( uint8_t reg, uint8_t data )
{
    reg = reg | 0x80; // MSB alto para escritura

    spi_transaction_t transaction;
    memset( &transaction, 0, sizeof(spi_transaction_t) );

    transaction.length = 8; // Longitud en bits de los datos a transmitir (1 byte)
    transaction.rxlength = 0; // No se esperan datos de vuelta para una escritura
    transaction.addr = reg; // La dirección del registro y el bit de escritura
    transaction.flags = SPI_TRANS_USE_TXDATA; // Usar el campo tx_data para datos cortos

    memcpy(transaction.tx_data, &data, 1); // Copiar 1 byte de datos en el buffer tx_data

    esp_err_t err = spi_device_polling_transmit(_spi, &transaction);

    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error escribiendo registro SPI 0x%02x con 0x%02x: %s", reg & 0x7f, data, esp_err_to_name(err));
}

// ** FUNCIÓN readRegister **
uint8_t LoRa::readRegister( uint8_t reg )
{
    uint8_t result = 0; // Inicializar resultado a 0
    uint8_t tx_dummy_buffer[1] = {0x00}; // Byte dummy para generar ciclos de reloj en MOSI para la lectura

    spi_transaction_t transaction;
    memset( &transaction, 0, sizeof(spi_transaction_t) );

    // IMPORTANTE: Para la lectura, 'length' es el número de bits a transmitir
    // durante la fase de datos. Necesitamos enviar 8 bits dummy para leer 8 bits.
    transaction.length = 8; // Generar 8 pulsos de reloj para la fase de datos (para RX)
    transaction.rxlength = 8; // Esperar recibir 8 bits de datos

    // La dirección del registro LoRa para lectura (el MSB debe ser 0)
    transaction.addr = reg & 0x7f;

    // Usar tx_buffer y rx_buffer para enviar datos dummy y recibir datos reales
    transaction.tx_buffer = tx_dummy_buffer; // Enviar byte dummy (todos ceros)
    transaction.rx_buffer = &result;         // Recibir los datos reales en 'result'

    // No es necesario SPI_TRANS_USE_TXDATA o SPI_TRANS_USE_RXDATA cuando se usan tx_buffer/rx_buffer
    transaction.flags = 0; // Limpiar cualquier bandera existente para usar buffers directamente

    esp_err_t err = spi_device_polling_transmit( _spi, &transaction);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo registro SPI 0x%02x: %s", reg, esp_err_to_name(err));
    }
    
    return result;
}

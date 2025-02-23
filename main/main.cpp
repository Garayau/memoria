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

#include "LoRa.h"

#define PIN_NUM_CLK  	18
#define PIN_NUM_MISO 	19
#define PIN_NUM_MOSI 	23
#define PIN_NUM_CS   	4
#define PIN_NUM_DIO		26
#define RESET_PIN  		15

#define SENDER_RECEIVER_PIN	22
#define	FLASH_PIN			2

int _counter = 0;


void writeMessage( LoRa* lora )
{
	char buf[100];
	lora->beginPacket(false);

	//sprintf( buf, "123456789012345678901234567890123456789012345678901234567890: [%d]", _counter++);
	sprintf( buf, "Count: [%d]", _counter++);

	//lora->setTxPower(14,RF_PACONFIG_PASELECT_PABOOST);
	lora->write( (uint8_t*) buf, (size_t) strlen(buf) );
	lora->endPacket(false);

	char msg[200];
	sprintf( msg, "Sender\n%s", buf );
}

void readMessage( LoRa* lora )
{
	int packetSize = lora->parsePacket(0);

	if (packetSize)
	{
		char msg[200];
		sprintf( msg, "Receiver: %d", packetSize );

		for (int i = 0; i < packetSize; i++)
			printf( "%c", lora->read() );

		printf( "\n");
	}
}

void delay( int msec )
{
    vTaskDelay( msec / portTICK_PERIOD_MS);
}

extern "C" void app_main();
extern "C" void lora_task( void *);

//void app_main()
void lora_task( void* param )
{

	LoRa lora( PIN_NUM_MOSI, PIN_NUM_MISO, PIN_NUM_CLK, PIN_NUM_CS, RESET_PIN, PIN_NUM_DIO, 10 );
	lora.setTxPower( 17, PA_BOOST );
	// lora.setTxPower( 5, PA_OUTPUT_RFO_PIN );

	gpio_num_t fp = (gpio_num_t) FLASH_PIN;
	gpio_pad_select_gpio( fp );
	gpio_set_direction( fp , GPIO_MODE_OUTPUT);





	for ( ;; )
	{
        uint64_t startTime = esp_timer_get_time(); // Time in µs

        // Reception mode
        lora.receive(0);
        printf("Esperando mensajes...\n");

        // Wait message for 2 seconds
        while ((esp_timer_get_time() - startTime) < 2000000) // 1s = 1,000,000 µs
        {
            if ( lora.getDataReceived() )
            {   
                // Flash the LED
                for ( int i = 0 ; i < 3 ; i++)
                {
                    gpio_set_level( fp, 1);
                    delay(50);
                    gpio_set_level( fp, 0);
                    delay(50);
                }

                // Read message
                char buf[200] = {0};
                char msg[100] = {0};

                int packetSize = lora.handleDataReceived( msg );
                lora.setDataReceived( false );

                // Print the message with escape sequences for special characters
                sprintf(buf, "\n<%s>\n(%d) RSSI: %d\n", msg, packetSize, lora.getPacketRssi());

                printf( buf );

                // Send message
                writeMessage( &lora );
                printf("Sending\n");

                // Flash the LED
                for ( int i = 0 ; i < 3 ; i++)
                {
                    gpio_set_level( fp, 1);
                    delay(50);
                    gpio_set_level( fp, 0);
                    delay(50);
                }
                }
        }

		delay(10);

	}

}


void app_main()
{
	// Had to set the task size to 10k otherwise I would get various instabilities
	// Around 2k or less I would get the stack overflow warning but at 2048 it would
	// just crash in various random ways

	xTaskCreate(lora_task, "lora_task", 10000, NULL, 1, NULL);
}


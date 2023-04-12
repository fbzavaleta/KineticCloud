/*
 * Lib C
 */
#include <stdio.h>
#include <stdint.h>  
#include <string.h>

/*
 * FreeRTOS
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
/*
 * Drivers
 */
#include "driver/gpio.h"
#include "wifi.h"
#include "nvs_flash.h"

/*
 * logs
 */
#include "esp_log.h"


static const char * TAG = "MAIN-NAV: ";

void http_Socket ( void * pvParameter );
void http_SendReceive ( void * pvParameter );
#define DEGUG 1

typedef struct xData {
 	int sock; 
 	uint32_t val_teste;
} xSocket_t;

const char * msg_post = \

	"POST /update HTTP/1.1\n"
	"Host: api.thingspeak.com\n"
	"Connection: close\n"
	"X-THINGSPEAKAPIKEY: %s\n"
	"Content-Type: application/x-www-form-urlencoded\n"
	"content-length: ";


void http_Socket(void * pvParameter)
{
	int rc; 
	xSocket_t xSocket;
	
	for (;;)
	{
		int sock;

		open_socket(&sock, &rc);
		ESP_LOGI(TAG, "Status Socket: %d", rc);

		if (rc == -1)
		{
			ESP_LOGI(TAG, "error on Socket: %d", rc);
			for( int i = 1; i <= 5; ++i )
			{	
				ESP_LOGI(TAG, "timeout: %d", 5-i);
				vTaskDelay( 1000/portTICK_PERIOD_MS );
			}
			continue;			
		}

		xSocket.sock = sock;
		xSocket.val_teste = 10;

		xTaskCreate( http_SendReceive, "http_SendReceive", 10000, (void*)&(xSocket), 5, NULL );
	}
	vTaskDelete(NULL);
	
}

void http_SendReceive(void * pvParameter)
{
	int rec_offset = 0; 
	int total =	1*1024; 
	char post_string[200];
	const char *apikey = &API_WRITE_KEY[0];

	char *buffer = pvPortMalloc( total );
	if( buffer == NULL ) 
	{
		
		ESP_LOGI(TAG, "pvPortMalloc Error\r\n"); //alocate block of memory from the heap
		vTaskDelete(NULL); 	  
		return;
	 }
    xSocket_t * xSocket = (xSocket_t*) pvParameter;
	sprintf(post_string, msg_post, apikey);

	char databody[50];
  	sprintf( databody, "{%s&field1=%d}", API_WRITE_KEY, xSocket->val_teste);
	sprintf( buffer , "%s%d\r\n\r\n%s\r\n\r\n", post_string, strlen(databody), databody);
  
	int rc = send( xSocket->sock, buffer, strlen(buffer), 0 );

	ESP_LOGI(TAG, "HTTP Enviado? rc: %d", rc);
	
	for(;;)
	{
		ssize_t sizeRead = recv(xSocket->sock, buffer+rec_offset, total-rec_offset, 0);
		
		if ( sizeRead == -1 ) 
		{
			ESP_LOGI( TAG, "recv: %d", sizeRead );
		}

		if ( sizeRead == 0 ) 
		{
			break;
		}

		if( sizeRead > 0 ) 
		{	
			ESP_LOGI(TAG, "Socket: %d - Data read (size: %d) was: %.*s", xSocket->sock, sizeRead, sizeRead, buffer);
		   
		   rec_offset += sizeRead;
		 }

		vTaskDelay( 5/portTICK_PERIOD_MS );
	}
	
	rc = close(xSocket->sock);
	
	ESP_LOGI(TAG, "close: rc: %d", rc); 
	
	vPortFree( buffer );	

	vTaskDelete(NULL); 	
}

void app_main(void)
{
	esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) 
	{
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
	ESP_ERROR_CHECK(ret);

	wifi_config();
    if( ( xTaskCreate( http_Socket, "http_Socket", 2048, NULL, 5, NULL )) != pdTRUE )
	{
		ESP_LOGI( TAG, "error - nao foi possivel alocar http_Socket.\n" );	
		return;		
	}          

}
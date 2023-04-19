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
#include <driver/i2c.h>
#include "nvs_flash.h"

/*
 * logs
 */
#include "esp_log.h"

/**
 * MPU6050 definitions
*/
#define PIN_SDA 21
#define PIN_CLK 22
#define I2C_ADDRESS 0x68 // I2C address of MPU6050 without AD0

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1   0x6B

i2c_config_t conf;
i2c_cmd_handle_t cmd;

uint8_t data[14];
short accel_x;
short accel_y;
short accel_z;

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
		xSocket.val_teste = 10

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

void init_mpu6050()
{
	ESP_LOGD(TAG, ">>Init mpu6050");
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = PIN_SDA;
	conf.scl_io_num = PIN_CLK;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;	

	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));	

	vTaskDelay(200/portTICK_PERIOD_MS);    

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, 1);
	i2c_master_write_byte(cmd, 0, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	vTaskDelay(100/portTICK_PERIOD_MS);
}

void task_mpu6050() {

	for (;;)
	{
		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1));
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
		i2c_cmd_link_delete(cmd);
	
		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1));

		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data,   0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+1, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+2, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+3, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+4, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+5, 1));
	
		//i2c_master_read(cmd, data, sizeof(data), 1);
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
		i2c_cmd_link_delete(cmd);

		accel_x = (data[0] << 8) | data[1];
		accel_y = (data[2] << 8) | data[3];
		accel_z = (data[4] << 8) | data[5];	

		vTaskDelay(500/portTICK_PERIOD_MS);		
	}
	
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
	init_mpu6050();
    if( ( xTaskCreate( http_Socket, "http_Socket", 2048, NULL, 5, NULL )) != pdTRUE )
	{
		ESP_LOGI( TAG, "error - nao foi possivel alocar http_Socket.\n" );	
		return;		
	}   

	if( xTaskCreate( task_mpu6050, "task_mpu6050", 4048, NULL, 5, NULL ) != pdTRUE )
	{
		ESP_LOGI( TAG, "error - nao foi possivel alocar task_mpu6050.\n" );
		return;
	}	      

}
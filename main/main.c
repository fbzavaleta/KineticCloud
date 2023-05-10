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
#include "driver/ledc.h"
#include <driver/i2c.h>
#include "nvs_flash.h"
#include "ultrasonic.h"
#include "wifi.h"
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

//Ultrasonic
#define ECHO_GPIO   26
#define TRIGG_GPIO  25
#define CMD_MEASURE	300
#define MAX_DISTANCE_CM 500 // 5m max

QueueHandle_t XQuee_ultrasonic;

typedef struct {
	uint16_t command;
	uint32_t distance;
	TaskHandle_t taskHandle;
} CMD_t;

//PWM servo
#define pinServo 18
#define ServoMsMin 0.06
#define ServoMsMax 2.1
#define ServoMsAvg ((ServoMsMax-ServoMsMin)/2.0)

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
 	uint32_t x_accel; 
	uint32_t y_accel; 
	uint32_t z_accel; 	
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
		xSocket.x_accel = accel_x; 
		xSocket.y_accel = accel_y; 
		xSocket.z_accel = accel_z;		

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
  	sprintf( databody, "{%s&field1=%d&field2=%d&field3=%d}", API_WRITE_KEY, xSocket->x_accel, xSocket->y_accel, xSocket->z_accel);
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

		vTaskDelay( 50/portTICK_PERIOD_MS );
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


void config_servo()
{
	ledc_timer_config_t ledc_timer = {
			.speed_mode       = LEDC_LOW_SPEED_MODE,
			.timer_num        = LEDC_TIMER_0,
			.duty_resolution  = LEDC_TIMER_13_BIT,
			.freq_hz          = 50,  
			.clk_cfg          = LEDC_AUTO_CLK
	};
	ledc_timer_config(&ledc_timer);
	ledc_channel_config_t ledc_channel = {
			.speed_mode     = LEDC_LOW_SPEED_MODE,
			.channel        = LEDC_CHANNEL_0,
			.timer_sel      = LEDC_TIMER_0,
			.intr_type      = LEDC_INTR_DISABLE,
			.gpio_num       = pinServo,
			.duty           = 0,
			.hpoint         = 0
	};
	ledc_channel_config(&ledc_channel);  	
}
void servoDeg0() {
  int duty = (int)(100.0*(ServoMsMin/20.0)*81.91);
  printf("%fms, duty = %f%% -> %d\n",ServoMsMin, 100.0*(ServoMsMin/20.0), duty);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);  
  vTaskDelay( 2000/portTICK_PERIOD_MS ); 
  ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
}

void servoDeg90() {

  int duty = (int)(100.0*(ServoMsAvg/20.0)*81.91);
  printf("%fms, duty = %f%% -> %d\n",ServoMsAvg, 100.0*(ServoMsAvg/20.0), duty);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);  
  vTaskDelay( 2000/portTICK_PERIOD_MS ); 
  ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
}

void servoDeg180() {

  int duty = (int)(100.0*(ServoMsMax/20.0)*81.91);
  printf("%fms, duty = %f%% -> %d\n",ServoMsMax, 100.0*(ServoMsMax/20.0), duty);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);  
  vTaskDelay( 2000/portTICK_PERIOD_MS ); 
  ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
}

void manualServo() {

	for (;;)
	{
		printf("  0 degree:");
		servoDeg0();
		printf(" 90 degree:");
		servoDeg90();
		printf("180 degree:");
		servoDeg180();
		printf(" 90 degree:");
		servoDeg90();		
	}

}

void ultrasonic()
{
	CMD_t cmdBuf;
	cmdBuf.command = CMD_MEASURE;
	cmdBuf.taskHandle = xTaskGetCurrentTaskHandle();

	ultrasonic_sensor_t sensor = {
		.trigger_pin = TRIGG_GPIO,
		.echo_pin = ECHO_GPIO
	};

	ultrasonic_init(&sensor);    

	while (true) {
		uint32_t distance;
		esp_err_t res = ultrasonic_measure_cm(&sensor, MAX_DISTANCE_CM, &distance);
		if (res != ESP_OK) {
			printf("Error: ");
			switch (res) {
				case ESP_ERR_ULTRASONIC_PING:
					printf("Cannot ping (device is in invalid state)\n");
					break;
				case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
					printf("Ping timeout (no device found)\n");
					break;
				case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
					printf("Echo timeout (i.e. distance too big)\n");
					break;
				default:
					printf("%d\n", res);
			}
		} else {
			ESP_LOGI(TAG,"Send Distance: %d cm, %.02f m\n", distance, distance / 100.0);
			cmdBuf.distance = distance;
			xQueueSend(XQuee_ultrasonic, &cmdBuf, portMAX_DELAY);
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}    
}

void data_orquestator()
{
	CMD_t cmdBuf;
	uint8_t ascii[30];

		for(;;)
		{
			xQueueReceive( XQuee_ultrasonic, &cmdBuf, portMAX_DELAY ); 	

			strcpy((char*)ascii, "Ultrasonic DISTANCE");
			sprintf((char*)ascii, "%d cm",cmdBuf.distance );
			
			
			if( DEGUG ) 
			{
				ESP_LOGI(TAG,"\n\nDistance msg \ 
									value: %s", ascii);
			}

			vTaskDelay( 10/portTICK_PERIOD_MS );	
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

	//wifi_config();
	//init_mpu6050();
	//config_servo();
/*
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

	if( xTaskCreate( manualServo, "manualServo", 4048, NULL, 5, NULL ) != pdTRUE )
	{
		ESP_LOGI( TAG, "error - nao foi possivel alocar task_mpu6050.\n" );
		return;
	}	
*/
	if( (XQuee_ultrasonic = xQueueCreate( 10, sizeof(CMD_t)) ) == NULL )
	{
		ESP_LOGI( TAG, "error - nao foi possivel alocar XQuee_ultrasonic.\n" );
		return;
	} 

    if( ( xTaskCreate( ultrasonic, "ultrasonic", 2048, NULL, 5, NULL )) != pdTRUE )
	{
		ESP_LOGI( TAG, "error - nao foi possivel alocar ultrasonic.\n" );	
		return;		
	}   

    if( ( xTaskCreate( data_orquestator, "data_orquestator", 2048, NULL, 5, NULL )) != pdTRUE )
	{
		ESP_LOGI( TAG, "error - nao foi possivel alocar data_orquestator.\n" );	
		return;		
	}   

}
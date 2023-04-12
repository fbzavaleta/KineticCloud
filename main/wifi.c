#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "freertos/event_groups.h"
#include <string.h>


#include "wifi.h"
static const char * TAG = "MAIN-WIFI: ";

static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
		case SYSTEM_EVENT_STA_START:
			esp_wifi_connect();
			break;
		case SYSTEM_EVENT_STA_GOT_IP:
			ESP_LOGI(TAG, "got ip:%s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
			xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
			break;
		case SYSTEM_EVENT_STA_DISCONNECTED:
			esp_wifi_connect();
			xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
			break;
		default:
			break;
    }
    return ESP_OK;
}

/*
Configure the esp32 wifi mode 
*/
void wifi_init_sta( void )
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) ); // connect to network as a client
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

/*
This function should be configure the internet network connection
*/
void wifi_config()
{
    wifi_event_group = xEventGroupCreate();
    wifi_init_sta();
}

/*
In networking, a socket is an endpoint for communication between 
two processes over a network
*/
void open_socket(int * sock_var, int * status_var)
{
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); // IPv4, SOCK_STREAM(bytes stream), TCP
    struct sockaddr_in serverAddress;

    serverAddress.sin_family = AF_INET;
    inet_pton(AF_INET, SERVER_IP, &serverAddress.sin_addr.s_addr); //
    serverAddress.sin_port = htons(SERVER_PORT);

    * status_var =  connect(sock, (struct sockaddr *)&serverAddress, sizeof(struct sockaddr_in));
    * sock_var = sock;
}



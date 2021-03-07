/* WiFi Hello World

   Based on ESP-IDF WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/socket.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi hello world";

static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

#define EXAMPLE_ECHO_SERVER_PORT CONFIG_ESP_ECHO_SERVER_PORT
const int LISTEN_BACKLOG = 128;

void echo_server(void) {
  if (EXAMPLE_ECHO_SERVER_PORT == -1) {
    ESP_LOGI(TAG, "Echo server disabled");
    return;
  }

  int s = socket(AF_INET, SOCK_STREAM, 0);
  if (s == -1) {
    ESP_LOGE(TAG, "socket failed: %d", errno);
    return;
  }
  struct sockaddr_in servaddr;
  memset(&servaddr, 0, sizeof (servaddr));
  servaddr.sin_family      = AF_INET;
  servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  servaddr.sin_port        = htons(EXAMPLE_ECHO_SERVER_PORT);
  if (bind(s, (const struct sockaddr*)&servaddr, sizeof (servaddr)) == -1) {
    ESP_LOGE(TAG, "bind failed: %d", errno);
    return;
  }

  if (listen(s, LISTEN_BACKLOG) == -1) {
    ESP_LOGE(TAG, "listen failed: %d", errno);
    return;
  }

  for (;;) {
    struct sockaddr_in cliaddr;
    socklen_t cliaddr_len = sizeof (cliaddr);
    int clis = accept(s, (struct sockaddr*)&cliaddr, &cliaddr_len);
    if (clis == -1) {
      // TODO: Better error handling
      ESP_LOGE(TAG, "accept failed: %d", errno);
      close(s);
      return;
    }
    // TODO: select/epoll?
    char buf[2049];
    ssize_t n;
    while ( (n = read(clis, buf, sizeof (buf) - 1)) > 0) {
      buf[n] = '\0';
      ESP_LOGI(TAG, "Echo Got: %s", buf);
      n = write(clis, buf, n);
      if (n < 0) {
        break;
      }
    }
  }
  close(s);
}

#define EXAMPLE_ECHO_CONNECT_PORT CONFIG_ESP_ECHO_CONNECT_PORT
#define EXAMPLE_ECHO_CONNECT_ADDR CONFIG_ESP_ECHO_CONNECT_ADDR
#define EXAMPLE_ECHO_CONNECT_MSG  CONFIG_ESP_ECHO_CONNECT_MSG

void echo_client(void) {
  if (EXAMPLE_ECHO_CONNECT_PORT == -1) {
    ESP_LOGI(TAG, "Echo client disabled");
    return;
  }
  int s = socket(AF_INET, SOCK_STREAM, 0);
  if (s == -1) {
    ESP_LOGE(TAG, "socket failed: %d", errno);
    return;
  }
  struct sockaddr_in servaddr;
  memset(&servaddr, 0, sizeof (servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(EXAMPLE_ECHO_CONNECT_PORT);
  if (inet_aton(EXAMPLE_ECHO_CONNECT_ADDR, &servaddr.sin_addr) == 0) {
    ESP_LOGE(TAG, "inet_aton, invalid addr %s", EXAMPLE_ECHO_CONNECT_ADDR);
    return;
  }
  if (connect(s, (const struct sockaddr*)&servaddr, sizeof (servaddr)) == -1) {
    ESP_LOGE(TAG, "connect failed: %d", errno);
    close(s);
    return;
  }

  if (write(s, EXAMPLE_ECHO_CONNECT_MSG, strlen(EXAMPLE_ECHO_CONNECT_MSG)) < 0) {
    ESP_LOGE(TAG, "connect failed: %d", errno);
    close(s);
    return;
  }

  // TODO: read message

  close(s);
}

void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);

        //echo_client();
        echo_server();
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

void app_main(void) {
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
}

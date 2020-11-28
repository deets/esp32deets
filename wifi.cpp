#include "wifi.hh"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include <string>
#include <vector>
#include <sstream>
#include <string>

namespace {

const char *TAG = "wifi-sta";

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about one event
 * - are we connected to the AP with an IP? */
const int WIFI_CONNECTED_BIT = 1;

int s_retry_num = 0;

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
      //ESP_LOGI(TAG, "got ip: %s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        {
            if (s_retry_num < CONFIG_DEETS_WIFI_MAXIMUM_RETRY) {
                esp_wifi_connect();
                xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
                s_retry_num++;
                ESP_LOGI(TAG,"retry to connect to the AP");
            }
            ESP_LOGI(TAG,"connect to the AP fail\n");
            break;
        }
    default:
        break;
    }
    return ERR_OK;
}

void wifi_init_sta(const std::string& ssid, const std::string& password)
{
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    esp_err_t err = esp_event_loop_init(event_handler, NULL);
    ESP_ERROR_CHECK(err);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {};
    strncpy(reinterpret_cast<char*>(&wifi_config.sta.ssid[0]), ssid.c_str(), sizeof(wifi_config.sta.ssid));
    strncpy(reinterpret_cast<char*>(&wifi_config.sta.password[0]), password.c_str(), sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
             ssid.c_str(), password.c_str());
}

struct network_entry_t
{
  std::string ssid;
  std::string password;
};

std::vector<network_entry_t> parse_network_config(const char* config)
{
  std::vector<network_entry_t> res;
  std::string token;
  std::istringstream tokenStream(config);
  while(std::getline(tokenStream, token, ','))
  {
    ESP_LOGI(TAG, "found WIFI configuration entry: %s", token.c_str());
    std::istringstream part_stream(token);
    network_entry_t entry;
    std::getline(part_stream, entry.ssid, ':');
    std::getline(part_stream, entry.password, ':');
    res.push_back(entry);
  }
  return res;
}

} // end ns anon


void setup_wifi()
{
  ESP_LOGI(TAG, CONFIG_DEETS_WIFI_NETWORK_CONFIG);
  const auto network_config = parse_network_config(CONFIG_DEETS_WIFI_NETWORK_CONFIG);
  assert(network_config.size() == 1);
  wifi_init_sta(network_config[0].ssid, network_config[0].password);
}


bool wifi_connected()
{
  return xEventGroupGetBits(s_wifi_event_group) | WIFI_CONNECTED_BIT;
}

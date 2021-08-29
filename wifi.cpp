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

struct network_entry_t
{
  std::string ssid;
  std::string password;
};

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group = nullptr;

/* The event group allows multiple bits for each event, but we only care about one event
 * - are we connected to the AP with an IP? */
const int WIFI_CONNECTED_BIT = 1;

void event_handler(void* arg, esp_event_base_t event_base,
		   int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
      ESP_LOGE(TAG, "WIFI_EVENT, WIFI_EVENT_STA_START");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
      ESP_LOGE(TAG, "WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED");
      esp_wifi_connect();
    } else if (event_base == IP_EVENT)
    {
      switch(event_id)
      {
      case IP_EVENT_STA_GOT_IP:
	{
	  ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
	  ESP_LOGI(TAG, "IP_EVENT, IP_EVENT_STA_GOT_IP, got ip:" IPSTR, IP2STR(&event->ip_info.ip));
	  xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
	}
	break;
      case IP_EVENT_STA_LOST_IP:
	ESP_LOGI(TAG, "IP_EVENT, IP_EVENT_STA_LOST_IP");
	xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
	break;
      }
    }
}


void wifi_init_sta(std::vector<network_entry_t> preconfigured_networks)
{
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));
    esp_netif_init();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    if(preconfigured_networks.size())
    {
      wifi_config_t wifi_config = {};
      const auto ssid = preconfigured_networks[0].ssid;
      const auto password = preconfigured_networks[0].password;
      strncpy(reinterpret_cast<char*>(&wifi_config.sta.ssid[0]), ssid.c_str(), sizeof(wifi_config.sta.ssid));
      strncpy(reinterpret_cast<char*>(&wifi_config.sta.password[0]), password.c_str(), sizeof(wifi_config.sta.password));
      wifi_config.sta.pmf_cfg = { true, false };
      ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
	       ssid.c_str(), password.c_str());
      esp_wifi_set_mode(WIFI_MODE_STA);
      ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

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
  s_wifi_event_group = xEventGroupCreate();
  ESP_LOGI(TAG, "sdkconfig network config: %s", CONFIG_DEETS_WIFI_NETWORK_CONFIG);
  const auto network_config = parse_network_config(CONFIG_DEETS_WIFI_NETWORK_CONFIG);
  wifi_init_sta(network_config);
}


bool wifi_connected()
{
  return xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT;
}

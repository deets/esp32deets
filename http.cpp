// Copyright: 2021, Diez B. Roggisch, Berlin, all rights reserved

#include "http.hpp"

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include <esp_log.h>

#include <cstring>

namespace deets::http {

namespace {
#define TAG "http"


}

HTTPServer::HTTPServer() {

}

void HTTPServer::start()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&_server, &config) == ESP_OK)
    {
      for(auto& handler : _handlers)
      {
        httpd_register_uri_handler(_server, &handler.second.esp_handler);
      }
    }
    else {
      ESP_LOGE(TAG, "Error starting server!");
    }
}

void HTTPServer::register_handler(const char* path, httpd_method_t method, handler_callback_t callback)
{
  auto& mapping = _handlers[path];
  mapping.callback = std::move(callback);
  std::memset(&mapping.esp_handler, 0, sizeof(mapping.esp_handler));
  mapping.esp_handler.method = method;
  mapping.esp_handler.uri = path;
  mapping.esp_handler.handler = &HTTPServer::s_dispatch;
  mapping.esp_handler.user_ctx = this;
}

esp_err_t HTTPServer::s_dispatch(httpd_req_t *req)
{
  return static_cast<HTTPServer*>(req->user_ctx)->dispatch(req);;
}

esp_err_t HTTPServer::dispatch(httpd_req_t *req) {
  if(_handlers.count(req->uri))
  {
    auto& mapping = _handlers[req->uri];
    return mapping.callback(req);
  }
  return ESP_FAIL;
}

} // namespace deets::http

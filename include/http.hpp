// Copyright: 2021, Diez B. Roggisch, Berlin, all rights reserved

#pragma once

#include <esp_http_server.h>
#include <map>
#include <functional>

namespace deets::http {

class HTTPServer
{
  using handler_callback_t = std::function<esp_err_t(httpd_req_t *req)>;
  struct handler_mapping_t
  {
    httpd_uri_t esp_handler;
    handler_callback_t callback;
  };

public:
  HTTPServer();
  void register_handler(const char* path, httpd_method_t method, handler_callback_t callback);
  void start();

private:
  static esp_err_t s_dispatch(httpd_req_t *req);
  esp_err_t dispatch(httpd_req_t *req);

  httpd_handle_t _server = nullptr;
  std::map<std::string, handler_mapping_t> _handlers;
};

}

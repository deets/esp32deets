// Copyright: 2021, Diez B. Roggisch, Berlin, all rights reserved

#pragma once

#include "nlohmann/json.hpp"

#include <esp_http_server.h>

#include <map>
#include <functional>
#include <variant>

namespace deets::http {

using json = nlohmann::json;

class HTTPServer
{
  using handler_callback_t = std::variant<
    std::function<esp_err_t(httpd_req_t*)>,
    std::function<json(const json&)>
  >;

public:

  HTTPServer();
  void register_handler(const char* path, httpd_method_t method, handler_callback_t callback);
  void start();
  void set_cors(const std::string& origin, uint32_t timeout);

private:

  struct handler_mapping_t
  {
    httpd_uri_t esp_handler;
    handler_callback_t callback;
  };

  static esp_err_t s_dispatch(httpd_req_t *req);
  esp_err_t dispatch(httpd_req_t *req);

  void preflight(httpd_req_t*);

  httpd_handle_t _server = nullptr;
  std::map<std::string, handler_mapping_t> _handlers;
  std::optional<std::tuple<std::string, std::string>> _cors;
};

}

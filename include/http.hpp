// Copyright: 2021, Diez B. Roggisch, Berlin, all rights reserved

#pragma once

#include "nlohmann/json.hpp"

#include <esp_http_server.h>

#include <map>
#include <functional>
#include <variant>

namespace deets::http {

using json = nlohmann::json;

enum class ContentType
{
  text_html
};

class HTTPServer
{
  using handler_callback_t = std::variant<
    std::function<esp_err_t(httpd_req_t*)>,
    std::function<json(const json&)>
  >;

  struct handler_static_content_t
  {
    uint8_t *start;
    uint8_t *end;
    ContentType content_type;
  };

  using handler_definition_t = std::variant<handler_callback_t, handler_static_content_t>;

public:
  HTTPServer();
  void register_handler(const char* path, httpd_method_t method, handler_callback_t callback);
  void register_handler(const char* path, uint8_t* start, uint8_t* end, ContentType);
  void start();
  void set_cors(const std::string& origin, uint32_t timeout);

private:

  struct handler_mapping_t
  {
    httpd_uri_t esp_handler;
    handler_definition_t handler_definition;
  };

  static esp_err_t s_dispatch(httpd_req_t *req);
  esp_err_t dispatch(httpd_req_t *req);
  esp_err_t serve_callback(httpd_req_t *req, handler_callback_t& callback);
  esp_err_t serve_static_content(httpd_req_t *req, handler_static_content_t& static_content);

  void preflight(httpd_req_t*);
  std::optional<std::string> header_value(httpd_req_t *req, const std::string&);

  httpd_handle_t _server = nullptr;
  std::map<std::string, handler_mapping_t> _handlers;
  std::optional<std::tuple<std::string, std::string>> _cors;
};

}

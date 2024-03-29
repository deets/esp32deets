// Copyright: 2021, Diez B. Roggisch, Berlin, all rights reserved

#include "http.hpp"
#include "http_parser.h"
#include <optional>

//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include <esp_log.h>

#include <cstring>
#include <tuple>
#include <sstream>

//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

namespace deets::http {

namespace {

#define TAG "http"

const auto ACCESS_CONTROL_ALLOW_ORIGIN = "Access-Control-Allow-Origin";
const auto ACCESS_CONTROL_MAX_AGE = "Access-Control-Max-Age";
const auto ACCESS_CONTROL_ALLOW_METHODS = "Access-Control-Allow-Methods";
const auto ALL_METHODS = "PUT,POST,GET,OPTIONS";
const auto ACCESS_CONTROL_ALLOW_HEADERS = "Access-Control-Allow-Headers";

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
  auto& mapping = _handlers[{path, method}];
  mapping.handler_definition = std::move(callback);
  std::memset(&mapping.esp_handler, 0, sizeof(mapping.esp_handler));
  mapping.esp_handler.method = method;
  mapping.esp_handler.uri = path;
  mapping.esp_handler.handler = &HTTPServer::s_dispatch;
  mapping.esp_handler.user_ctx = this;
}

void HTTPServer::register_handler(const char *path, uint8_t *start,
                                  uint8_t *end, ContentType content_type) {
  auto& mapping = _handlers[{path, HTTP_GET}];
  const auto handler = handler_static_content_t { start, end, content_type };
  mapping.handler_definition = std::move(handler);
  std::memset(&mapping.esp_handler, 0, sizeof(mapping.esp_handler));
  mapping.esp_handler.method = HTTP_GET;
  mapping.esp_handler.uri = path;
  mapping.esp_handler.handler = &HTTPServer::s_dispatch;
  mapping.esp_handler.user_ctx = this;
}

esp_err_t HTTPServer::s_dispatch(httpd_req_t *req)
{
  return static_cast<HTTPServer*>(req->user_ctx)->dispatch(req);;
}

esp_err_t HTTPServer::dispatch(httpd_req_t *req) {
  const auto key = std::tuple<std::string, httpd_method_t>(req->uri, httpd_method_t(req->method));
  if(_handlers.count(key))
  {
    preflight(req);

    auto& mapping = _handlers[key];
    if(auto pcallback_val = std::get_if<handler_callback_t>(&mapping.handler_definition))
    {
      return serve_callback(req, *pcallback_val);
    }
    if(auto static_val = std::get_if<handler_static_content_t>(&mapping.handler_definition))
    {
      return serve_static_content(req, *static_val);
    }
  }
  return ESP_FAIL;
}

esp_err_t HTTPServer::serve_static_content(httpd_req_t *req, handler_static_content_t& static_content)
{
  switch(static_content.content_type)
  {
  case ContentType::text_html:
    httpd_resp_set_type(req, "text/html");
    break;
  }

  return httpd_resp_send(
    req,
    reinterpret_cast<const char*>(static_content.start),
    static_content.end - static_content.start
    );
}

esp_err_t HTTPServer::serve_callback(httpd_req_t *req, handler_callback_t& callback)
{
  if(auto pval = std::get_if<std::function<esp_err_t(httpd_req_t*)>>(&callback))
  {
    return (*pval)(req);
  }
  if(auto pval = std::get_if<std::function<json(const json&)>>(&callback))
  {
    json body;
    if(const auto content_type = header_value(req, "Content-Type"))
    {
      const auto content_type_value = *content_type;
      ESP_LOGD(TAG, "Content-type: %s, %i", content_type_value.c_str(), content_type_value.size());
      if(content_type_value == "application/json")
      {
	ESP_LOGD(TAG, "Got application/json, %i bytes", req->content_len);
	std::vector<char> buffer(req->content_len);
	httpd_req_recv(req, buffer.data(), buffer.size());
	body = body.parse(buffer.begin(), buffer.end());
      }
    }
    const auto result = (*pval)(body);
    const auto s = result.dump();
    httpd_resp_set_type(req, "text/json");
    httpd_resp_send(req, s.c_str(), s.size());
    return ESP_OK;
  }
  return ESP_FAIL;
}


std::optional<std::string> HTTPServer::header_value(httpd_req_t *req, const std::string &header)
{
  if(const auto len = httpd_req_get_hdr_value_len(req, header.c_str()))
  {
    ESP_LOGD(TAG, "found header, length: %i", len);
    std::vector<char> buffer(len + 1);
    httpd_req_get_hdr_value_str(req, header.c_str(), buffer.data(), buffer.size());
    ESP_LOGD(TAG, "value: %s", buffer.data());
    return std::string(buffer.data(), buffer.size() - 1);
  }
  ESP_LOGD(TAG, "no header %s", header.c_str());
  return std::nullopt;
}

void HTTPServer::set_cors(const std::string& origin, uint32_t timeout)
{
  std::stringstream ss;
  ss << timeout;
  _cors = std::make_tuple(origin, ss.str());
}

void HTTPServer::preflight(httpd_req_t *req)
{
  if(_cors)
  {
    const auto& [ origin, age ] = *_cors;
    httpd_resp_set_hdr(req, ACCESS_CONTROL_ALLOW_ORIGIN, origin.c_str());
    httpd_resp_set_hdr(req, ACCESS_CONTROL_MAX_AGE, age.c_str());
    httpd_resp_set_hdr(req, ACCESS_CONTROL_ALLOW_METHODS, ALL_METHODS);
    httpd_resp_set_hdr(req, ACCESS_CONTROL_ALLOW_HEADERS, "*");
  }
}

} // namespace deets::http

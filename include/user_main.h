#ifndef MAIN_HEADER
#define MAIN_HEADER

#define AP_CONNECTION_STATUS_LED_PIN GPIO_Pin_5
#define SERVER_AVAILABILITY_STATUS_LED_PIN GPIO_Pin_4

#ifndef true // needed only for Eclipse
   typedef unsigned char bool;
   #define true 1
   #define false 0
#endif

#define LONG_POLLING_REQUEST_ERROR_OCCURRED_FLAG   1
#define SERVER_IS_AVAILABLE_FLAG                   2
#define UPDATE_FIRMWARE_FLAG                       4
#define REQUEST_ERROR_OCCURRED_FLAG                8

#define REQUEST_IDLE_TIME_ON_ERROR (10000 / portTICK_RATE_MS) // 10 sec
#define REQUEST_MAX_DURATION_TIME (10000 / portTICK_RATE_MS) // 10 sec
#define LONG_POLLING_REQUEST_IDLE_TIME_ON_ERROR (10000 / portTICK_RATE_MS) // 10 sec
#define LONG_POLLING_REQUEST_MAX_DURATION_TIME (5.5 * 60 * 1000 / portTICK_RATE_MS) // 5.5 mins

char RESPONSE_SERVER_SENT_OK[] ICACHE_RODATA_ATTR = "\"statusCode\":\"OK\"";
char STATUS_INFO_POST_REQUEST[] ICACHE_RODATA_ATTR =
      "POST /server/esp8266/statusInfo HTTP/1.1\r\n"
      "Content-Length: <1>\r\n"
      "Host: <2>\r\n"
      "User-Agent: ESP8266\r\n"
      "Content-Type: application/json\r\n"
      "Accept: application/json\r\n\r\n"
      "<3>\r\n";
char STATUS_INFO_REQUEST_PAYLOAD[] ICACHE_RODATA_ATTR =
      "{\"gain\":\"<1>\","
      "\"deviceName\":\"<2>\","
      "\"errors\":\"<3>\","
      "\"buildTimestamp\":\"<4>\"}";
char ALARM_GET_REQUEST[] ICACHE_RODATA_ATTR =
      "GET /server/esp8266/alarm HTTP/1.1\r\n"
      "Host: <1>\r\n"
      "User-Agent: ESP8266\r\n"
      "Accept: application/json\r\n\r\n";
char UPDATE_FIRMWARE[] ICACHE_RODATA_ATTR = "\"updateFirmware\":true";
char FIRMWARE_UPDATE_GET_REQUEST[] ICACHE_RODATA_ATTR =
      "GET /esp8266_fota/<1> HTTP/1.1\r\n"
      "Host: <2>\r\n"
      "User-Agent: ESP8266\r\n"
      "Connection: close\r\n\r\n";

struct connection_user_data {
   bool response_received;
   char *request;
   char *response;
   void (*execute_on_succeed)(struct espconn *connection);
   void (*execute_on_error)(struct espconn *connection);
   xTaskHandle timeout_request_supervisor_task;
   portTickType request_max_duration_time;
};

void scan_access_point_task(void *pvParameters);
void send_long_polling_requests_task(void *pvParameters);
void autoconnect_task(void *pvParameters);
void successfull_connected_tcp_handler_callback(void *arg);
void successfull_disconnected_tcp_handler_callback();
void tcp_connection_error_handler_callback(void *arg, sint8 err);
void tcp_response_received_handler_callback(void *arg, char *pdata, unsigned short len);
void tcp_request_successfully_sent_handler_callback();
void tcp_request_successfully_written_into_buffer_handler_callback();
void long_polling_request_on_succeed_callback(struct espconn *connection);
void long_polling_request_on_error_callback(struct espconn *connection);
void long_polling_request_finish_action(struct espconn *connection);
void upgrade_firmware();
#endif

/**
 * Pins 4 and 5 on some ESP8266-07 are exchanged on silk screen!!!
 */

#include "esp_common.h"
#include "uart.h"
#include "gpio.h"
#include "esp_sta.h"
#include "esp_wifi.h"
#include "global_definitions.h"
#include "malloc_logger.h"
#include "upgrade.h"
#include "FREErtos/FREERTOS.h"
#include "device_settings.h"
#include "espconn.h"
#include "utils.h"
#include "lwip/sys.h"
#include "lwip/inet.h"
#include "user_main.h"

unsigned int milliseconds_counter_g;
int signal_strength_g;
unsigned short errors_counter_g;

LOCAL os_timer_t millisecons_time_serv_g;
LOCAL os_timer_t motion_detector_ignore_timer_g;
LOCAL os_timer_t pin_state_timers_g[16];
LOCAL os_timer_t ignore_alarms_timer_g;
LOCAL os_timer_t ignore_false_alarms_timer_g;
LOCAL os_timer_t recheck_false_alarm_timer_g;

struct _esp_tcp user_tcp;

unsigned char responses_index;
char *responses[10];
unsigned int general_flags;

xSemaphoreHandle status_request_semaphore_g;
xSemaphoreHandle requests_mutex_g;
xSemaphoreHandle buzzer_semaphore_g;

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
 *******************************************************************************/
uint32 user_rf_cal_sector_set(void) {
   flash_size_map size_map = system_get_flash_size_map();
   uint32 rf_cal_sec = 0;

   switch (size_map) {
      case FLASH_SIZE_4M_MAP_256_256:
         rf_cal_sec = 128 - 5;
         break;

      case FLASH_SIZE_8M_MAP_512_512:
         rf_cal_sec = 256 - 5;
         break;

      case FLASH_SIZE_16M_MAP_512_512:
      case FLASH_SIZE_16M_MAP_1024_1024:
         rf_cal_sec = 512 - 5;
         break;

      case FLASH_SIZE_32M_MAP_512_512:
      case FLASH_SIZE_32M_MAP_1024_1024:
         rf_cal_sec = 1024 - 5;
         break;

      default:
         rf_cal_sec = 0;
         break;
   }
   return rf_cal_sec;
}

LOCAL void milliseconds_counter() {
   milliseconds_counter_g++;
}

void start_100millisecons_counter() {
   os_timer_disarm(&millisecons_time_serv_g);
   os_timer_setfn(&millisecons_time_serv_g, (os_timer_func_t *) milliseconds_counter, NULL);
   os_timer_arm(&millisecons_time_serv_g, 1000 / MILLISECONDS_COUNTER_DIVIDER, 1); // 100 ms
}

void stop_milliseconds_counter() {
   os_timer_disarm(&millisecons_time_serv_g);
}

void stop_ignoring_motion_detector() {
   reset_flag(&general_flags, IGNORE_MOTION_DETECTOR_FLAG);

   #ifdef ALLOW_USE_PRINTF
   printf("\n Motion detector isn't ignored anymore. Time: %u\n", milliseconds_counter_g);
   #endif
}

void turn_motion_detector_on() {
   set_flag(&general_flags, IGNORE_MOTION_DETECTOR_FLAG);

   os_timer_disarm(&motion_detector_ignore_timer_g);
   os_timer_setfn(&motion_detector_ignore_timer_g, (os_timer_func_t *) stop_ignoring_motion_detector, NULL);
   os_timer_arm(&motion_detector_ignore_timer_g, IGNORE_MOTION_DETECTOR_TIMEOUT_AFTER_TURN_ON_SEC * 1000, false);

   pin_output_set(MOTION_DETECTOR_ENABLE_PIN);
}

void turn_motion_detector_off() {
   set_flag(&general_flags, IGNORE_MOTION_DETECTOR_FLAG);
   pin_output_reset(MOTION_DETECTOR_ENABLE_PIN);
}

// Callback function when AP scanning is completed
void get_ap_signal_strength(void *arg, STATUS status) {
   if (status == OK) {
      struct bss_info *got_bss_info = (struct bss_info *) arg;

      signal_strength_g = got_bss_info->rssi;
      //got_bss_info = got_bss_info->next.stqe_next;
   }
}

void scan_access_point_task(void *pvParameters) {
   long rescan_when_connected_task_delay = 10 * 60 * 1000 / portTICK_RATE_MS; // 10 mins
   long rescan_when_not_connected_task_delay = 10 * 1000 / portTICK_RATE_MS; // 10 secs

   for (;;) {
      STATION_STATUS status = wifi_station_get_connect_status();

      if (status == STATION_GOT_IP) {
         struct scan_config ap_scan_config;
         char *default_access_point_name = get_string_from_rom(ACCESS_POINT_NAME);

         ap_scan_config.ssid = default_access_point_name;
         wifi_station_scan(&ap_scan_config, get_ap_signal_strength);
         FREE(default_access_point_name);

         vTaskDelay(rescan_when_connected_task_delay);
      } else {
         vTaskDelay(rescan_when_not_connected_task_delay);
      }
   }
}

void autoconnect_task(void *pvParameters) {
   long task_delay = 10000 / portTICK_RATE_MS;

   for (;;) {
      STATION_STATUS status = wifi_station_get_connect_status();
      read_output_pin_state(AP_CONNECTION_STATUS_LED_PIN);
      if (status != STATION_GOT_IP && status != STATION_CONNECTING) {
         wifi_station_connect(); // Do not call this API in user_init
      }
      vTaskDelay(task_delay);
   }
}

void beep_task() {
   #ifdef ALLOW_USE_PRINTF
   printf("\n beep_task has been created. Time: %u\n", milliseconds_counter_g);
   #endif

   //vSemaphoreCreateBinary(buzzer_semaphore_g);

   pin_output_set(BUZZER_PIN);
   vTaskDelay(80 / portTICK_RATE_MS);
   pin_output_reset(BUZZER_PIN);

   /*vTaskDelay(500 / portTICK_RATE_MS);

   if (xSemaphoreTake(buzzer_semaphore_g, IGNORE_ALARMS_TIMEOUT_SEC * 1000 / portTICK_RATE_MS) == pdPASS) {
      pin_output_set(BUZZER_PIN);
      vTaskDelay(100 / portTICK_RATE_MS);
      pin_output_reset(BUZZER_PIN);
   }

   vSemaphoreDelete(buzzer_semaphore_g);*/
   buzzer_semaphore_g = NULL;
   vTaskDelete(NULL);
}

void successfull_connected_tcp_handler_callback(void *arg) {
   struct espconn *connection = arg;
   struct connection_user_data *user_data = connection->reserve;
   char *request = user_data->request;
   unsigned short request_length = strnlen(request, 0xFFFF);

   espconn_set_opt(connection, ESPCONN_REUSEADDR);
   // Keep-Alive timeout doesn't work yet
   //espconn_set_opt(connection, ESPCONN_KEEPALIVE); // ESPCONN_REUSEADDR |
   //uint32 espconn_keepidle_value = 5; // seconds
   //unsigned char keepalive_error_code = espconn_set_keepalive(connection, ESPCONN_KEEPIDLE, &espconn_keepidle_value);
   //uint32 espconn_keepintvl_value = 2; // seconds
   // If there is no response, retry ESPCONN_KEEPCNT times every ESPCONN_KEEPINTVL
   //keepalive_error_code |= espconn_set_keepalive(connection, ESPCONN_KEEPINTVL, &espconn_keepintvl_value);
   //uint32 espconn_keepcnt_value = 2; // count
   //keepalive_error_code |= espconn_set_keepalive(connection, ESPCONN_KEEPCNT, &espconn_keepcnt_value);

   int sent_status = espconn_send(connection, request, request_length);
   FREE(request);
   user_data->request = NULL;

   if (sent_status != 0) {
      void (*execute_on_error)(struct espconn *connection) = user_data->execute_on_error;
      if (execute_on_error) {
         execute_on_error(connection);
      }
   }
}

void successfull_disconnected_tcp_handler_callback(void *arg) {
   struct espconn *connection = arg;
   struct connection_user_data *user_data = connection->reserve;
   bool response_received = user_data->response_received;

   #ifdef ALLOW_USE_PRINTF
   printf("Disconnected callback beginning. Response %s received. Time: %u\n", response_received ? "has been" : "has not been", milliseconds_counter_g);
   #endif

   void (*execute_on_succeed)(struct espconn *connection) = user_data->execute_on_succeed;
   if (execute_on_succeed) {
      execute_on_succeed(connection);
   }
}

void tcp_connection_error_handler_callback(void *arg, sint8 err) {
   #ifdef ALLOW_USE_PRINTF
   printf("Connection error callback. Error code: %d. Time: %u\n", err, milliseconds_counter_g);
   #endif

   struct espconn *connection = arg;
   struct connection_user_data *user_data = connection->reserve;
   void (*execute_on_error)(struct espconn *connection) = user_data->execute_on_error;
   if (execute_on_error) {
      execute_on_error(connection);
   }
}

void tcp_response_received_handler_callback(void *arg, char *pdata, unsigned short len) {
   struct espconn *connection = arg;
   struct connection_user_data *user_data = connection->reserve;
   bool response_received = user_data->response_received;

   if (!response_received) {
      char *server_sent = get_string_from_rom(RESPONSE_SERVER_SENT_OK);

      if (strstr(pdata, server_sent)) {
         user_data->response_received = true;

         char *response = MALLOC(len, __LINE__);

         memcpy(response, pdata, len);
         user_data->response = response;

         #ifdef ALLOW_USE_PRINTF
         printf("Response received: %sTime: %u\n", pdata, milliseconds_counter_g);
         #endif
      }
      FREE(server_sent);
   }

   // Don't call this API in any espconn callback. If needed, please use system task to trigger espconn_disconnect.
   //espconn_disconnect(connection);
}

void tcp_request_successfully_sent_handler_callback() {
   //printf("Request sent callback\n");
}

void tcp_request_successfully_written_into_buffer_handler_callback() {
   //printf("Request written into buffer callback\n");
}

void status_request_on_error_callback(struct espconn *connection) {
   #ifdef ALLOW_USE_PRINTF
   printf("status_request_on_error_callback. Time: %u\n", milliseconds_counter_g);
   #endif

   errors_counter_g++;
   pin_output_reset(SERVER_AVAILABILITY_STATUS_LED_PIN);
   set_flag(&general_flags, REQUEST_ERROR_OCCURRED_FLAG);
   xSemaphoreHandle semaphores_to_give[] = {status_request_semaphore_g, requests_mutex_g, NULL};
   request_finish_action(connection, semaphores_to_give);
}

void general_request_on_error_callback(struct espconn *connection) {
   #ifdef ALLOW_USE_PRINTF
   printf("general_request_on_error_callback. Time: %u\n", milliseconds_counter_g);
   #endif

   errors_counter_g++;
   pin_output_reset(SERVER_AVAILABILITY_STATUS_LED_PIN);
   set_flag(&general_flags, REQUEST_ERROR_OCCURRED_FLAG);
   xSemaphoreHandle semaphores_to_give[] = {requests_mutex_g, NULL};
   request_finish_action(connection, semaphores_to_give);
}

void check_for_update_firmware(char *response) {
   char *update_firmware_json_element = get_string_from_rom(UPDATE_FIRMWARE);

   if (strstr(response, update_firmware_json_element) != NULL) {
      set_flag(&general_flags, UPDATE_FIRMWARE_FLAG);
   }
   FREE(update_firmware_json_element);
}

void check_for_manually_ignoring_alarms(char *response) {
   char *manually_ignore_alarms_json_element = get_string_from_rom(MANUALLY_IGNORE_ALARMS);

   if (strstr(response, manually_ignore_alarms_json_element) != NULL) {
      set_flag(&general_flags, MANUALLY_IGNORE_ALARMS_FLAG);
   } else {
      reset_flag(&general_flags, MANUALLY_IGNORE_ALARMS_FLAG);
   }
   FREE(manually_ignore_alarms_json_element);
}

void status_request_on_succeed_callback(struct espconn *connection) {
   #ifdef ALLOW_USE_PRINTF
   printf("status_request_on_succeed_callback, Time: %u\n", milliseconds_counter_g);
   #endif

   struct connection_user_data *user_data = connection->reserve;

   if (!user_data->response_received && user_data->execute_on_error != NULL) {
      user_data->execute_on_error(connection);
      return;
   }

   check_for_update_firmware(user_data->response);
   check_for_manually_ignoring_alarms(user_data->response);

   pin_output_set(SERVER_AVAILABILITY_STATUS_LED_PIN);
   xSemaphoreHandle semaphores_to_give[] = {requests_mutex_g, NULL};
   request_finish_action(connection, semaphores_to_give);

   xTaskCreate(activate_status_requests_task_task, "activate_status_requests_task_task", 200, NULL, 2, NULL);

   if (read_flag(general_flags, UPDATE_FIRMWARE_FLAG)) {
      upgrade_firmware();
   }
}

void general_request_on_succeed_callback(struct espconn *connection) {
   #ifdef ALLOW_USE_PRINTF
   printf("general_request_on_succeed_callback. Time: %u\n", milliseconds_counter_g);
   #endif

   struct connection_user_data *user_data = connection->reserve;
   xTaskHandle parent_task = user_data->parent_task;

   if (!user_data->response_received && user_data->execute_on_error != NULL) {
      user_data->execute_on_error(connection);
      return;
   }

   check_for_update_firmware(user_data->response);

   pin_output_set(SERVER_AVAILABILITY_STATUS_LED_PIN);

   if (parent_task != NULL) {
      #ifdef ALLOW_USE_PRINTF
      printf("parent task is to be deleted...\n");
      #endif

      vTaskDelete(parent_task);
   }

   if (buzzer_semaphore_g != NULL) {
      xSemaphoreGive(buzzer_semaphore_g);
   }

   xSemaphoreHandle semaphores_to_give[] = {requests_mutex_g, NULL};
   request_finish_action(connection, semaphores_to_give);
}

void request_finish_action(struct espconn *connection, xSemaphoreHandle semaphores_to_give[]) {
   struct connection_user_data *user_data = connection->reserve;

   if (user_data->request != NULL) {
      FREE(user_data->request);
      user_data->request = NULL;
   }
   if (user_data->response != NULL) {
      FREE(user_data->response);
      user_data->response = NULL;
   }

   if (user_data->timeout_request_supervisor_task != NULL) {
      #ifdef ALLOW_USE_PRINTF
      printf("\n timeout_request_supervisor_task still exists\n");
      #endif

      vTaskDelete(user_data->timeout_request_supervisor_task);
   }

   FREE(user_data);
   sint8 error_code = espconn_delete(connection);

   if (error_code != 0) {
      #ifdef ALLOW_USE_PRINTF
      printf("\n ERROR! Connection still is in progress\n");
      #endif
   }

   FREE(connection);

   if (semaphores_to_give != NULL) {
      unsigned char i;
      for (i = 0; semaphores_to_give[i] != NULL; i++) {
         xSemaphoreHandle semaphore_to_give = semaphores_to_give[i];
         xSemaphoreGive(semaphore_to_give);
      }
   }

   #if defined(ALLOW_USE_PRINTF) && defined(USE_MALLOC_LOGGER)
   printf("\n Elements amount in malloc logger list: %u\n", get_malloc_logger_list_elements_amount());
   print_not_empty_elements_lines();
   printf("\n Free Heap size: %u\n", xPortGetFreeHeapSize());
   #endif
}

void timeout_request_supervisor_task(void *pvParameters) {
   struct espconn *connection = pvParameters;
   struct connection_user_data *user_data = connection->reserve;

   vTaskDelay(user_data->request_max_duration_time);

   #ifdef ALLOW_USE_PRINTF
   printf("\n Request timeout. Time: %u\n", milliseconds_counter_g);
   #endif

   if (connection->state == ESPCONN_CONNECT) {
      #ifdef ALLOW_USE_PRINTF
      printf("\n Was connected\n");
      #endif

      espconn_disconnect(connection);
   }

   // To not delete this task in other functions
   user_data->timeout_request_supervisor_task = NULL;

   void (*execute_on_error)(struct espconn *connection) = user_data->execute_on_error;
   if (execute_on_error != NULL) {
      execute_on_error(connection);
   }

   vTaskDelete(NULL);
}

void ota_finished_callback(void *arg) {
   struct upgrade_server_info *update = arg;

   if (update->upgrade_flag == true) {
      #ifdef ALLOW_USE_PRINTF
      printf("[OTA] success; rebooting! Time: %u\n", milliseconds_counter_g);
      #endif

      system_upgrade_flag_set(UPGRADE_FLAG_FINISH);
      system_upgrade_reboot();
   } else {
      #ifdef ALLOW_USE_PRINTF
      printf("[OTA] failed! Time: %u\n", milliseconds_counter_g);
      #endif

      system_restart();
   }

   FREE(&update->sockaddrin);
   FREE(update->url);
   FREE(update);
}

void blink_leds_while_updating_task(void *pvParameters) {
   for (;;) {
      if (read_output_pin_state(AP_CONNECTION_STATUS_LED_PIN)) {
         pin_output_reset(AP_CONNECTION_STATUS_LED_PIN);
         pin_output_set(SERVER_AVAILABILITY_STATUS_LED_PIN);
      } else {
         pin_output_set(AP_CONNECTION_STATUS_LED_PIN);
         pin_output_reset(SERVER_AVAILABILITY_STATUS_LED_PIN);
      }

      vTaskDelay(100 / portTICK_RATE_MS);
   }
}

void upgrade_firmware() {
   #ifdef ALLOW_USE_PRINTF
   printf("\nUpdating firmware... Time: %u\n", milliseconds_counter_g);
   #endif

   turn_motion_detector_off();

   xTaskCreate(blink_leds_while_updating_task, "blink_leds_while_updating_task", 256, NULL, 1, NULL);

   struct upgrade_server_info *upgrade_server = (struct upgrade_server_info *) ZALLOC(sizeof(struct upgrade_server_info), __LINE__);
   struct sockaddr_in *sockaddrin = (struct sockaddr_in *) ZALLOC(sizeof(struct sockaddr_in), __LINE__);

   upgrade_server->sockaddrin = *sockaddrin;
   upgrade_server->sockaddrin.sin_family = AF_INET;
   struct in_addr sin_addr;
   char *server_ip = get_string_from_rom(SERVER_IP_ADDRESS);
   sin_addr.s_addr = inet_addr(server_ip);
   upgrade_server->sockaddrin.sin_addr = sin_addr;
   upgrade_server->sockaddrin.sin_port = htons(SERVER_PORT);
   upgrade_server->sockaddrin.sin_len = sizeof(upgrade_server->sockaddrin);
   upgrade_server->check_cb = ota_finished_callback;
   upgrade_server->check_times = 10;

   char *url_pattern = get_string_from_rom(FIRMWARE_UPDATE_GET_REQUEST);
   unsigned char user_bin = system_upgrade_userbin_check();
   char *file_to_download = user_bin == UPGRADE_FW_BIN1 ? "user2.bin" : "user1.bin";
   char *url_parameters[] = {file_to_download, server_ip, NULL};
   char *url = set_string_parameters(url_pattern, url_parameters);

   FREE(url_pattern);
   FREE(server_ip);
   upgrade_server->url = url;
   system_upgrade_start(upgrade_server);
}

void establish_connection(struct espconn *connection) {
   if (connection == NULL) {
      #ifdef ALLOW_USE_PRINTF
      printf("\n Create connection first\n");
      #endif

      return;
   }

   int connection_status = espconn_connect(connection);

   #ifdef ALLOW_USE_PRINTF
   printf("\n Connection status: ");
   #endif

   switch (connection_status) {
      case ESPCONN_OK:
      {
         struct connection_user_data *user_data = connection->reserve;
         xTaskHandle created_supervisor_task;

         if (user_data != NULL) {
            xTaskCreate(timeout_request_supervisor_task, "timeout_request_supervisor_task", 256, connection, 2, &created_supervisor_task);
            user_data->timeout_request_supervisor_task = created_supervisor_task;
         }

         #ifdef ALLOW_USE_PRINTF
         printf("Connected");
         #endif

         break;
      }
      case ESPCONN_RTE:
         #ifdef ALLOW_USE_PRINTF
         printf("Routing problem");
         #endif

         break;
      case ESPCONN_MEM:
         #ifdef ALLOW_USE_PRINTF
         printf("Out of memory");
         #endif

         break;
      case ESPCONN_ISCONN:
         #ifdef ALLOW_USE_PRINTF
         printf("Already connected");
         #endif

         break;
      case ESPCONN_ARG:
         #ifdef ALLOW_USE_PRINTF
         printf("Illegal argument");
         #endif

         break;
   }
   #ifdef ALLOW_USE_PRINTF
   printf(". Time: %u\n", milliseconds_counter_g);
   #endif

   if (connection_status != ESPCONN_OK) {
      struct connection_user_data *user_data = connection->reserve;

      if (user_data != NULL && user_data->execute_on_error != NULL) {
         user_data->execute_on_error(connection);
      }
   }
}

void activate_status_requests_task_task(void *pvParameters) {
   vTaskDelay(STATUS_REQUESTS_SEND_INTERVAL);
   xSemaphoreGive(status_request_semaphore_g);
   vTaskDelete(NULL);
}

void send_status_requests_task(void *pvParameters) {
   #ifdef ALLOW_USE_PRINTF
   printf("send_status_requests_task has been created\n");
   #endif
   vTaskDelay(5000 / portTICK_RATE_MS);

   for (;;) {
      xSemaphoreTake(status_request_semaphore_g, portMAX_DELAY);
      xSemaphoreTake(requests_mutex_g, portMAX_DELAY);

      if (read_flag(general_flags, UPDATE_FIRMWARE_FLAG)) {
         vTaskDelete(NULL);
      }

      #ifdef ALLOW_USE_PRINTF
      printf("send_status_requests_task started. Time: %u\n", milliseconds_counter_g);
      #endif

      if (!read_output_pin_state(AP_CONNECTION_STATUS_LED_PIN)) {
         #ifdef ALLOW_USE_PRINTF
         printf("Can't send status request, because not connected to AP. Time: %u\n", milliseconds_counter_g);
         #endif

         xSemaphoreGive(requests_mutex_g);
         xSemaphoreGive(status_request_semaphore_g);
         vTaskDelay(2000 / portTICK_RATE_MS);
         continue;
      }

      if (read_flag(general_flags, REQUEST_ERROR_OCCURRED_FLAG)) {
         reset_flag(&general_flags, REQUEST_ERROR_OCCURRED_FLAG);

         xSemaphoreGive(requests_mutex_g);
         xSemaphoreGive(status_request_semaphore_g);
         vTaskDelay(REQUEST_IDLE_TIME_ON_ERROR);
         continue;
      }

      char signal_strength[4];
      sprintf(signal_strength, "%d", signal_strength_g);
      char *device_name = get_string_from_rom(DEVICE_NAME);
      char errors_counter[5];
      sprintf(errors_counter, "%d", errors_counter_g);
      char uptime[10];
      sprintf(uptime, "%d", milliseconds_counter_g / MILLISECONDS_COUNTER_DIVIDER);
      char build_timestamp[30];
      sprintf(build_timestamp, "%s", __TIMESTAMP__);
      char *status_info_request_payload_template_parameters[] = {signal_strength, device_name, errors_counter, uptime, build_timestamp, NULL};
      char *status_info_request_payload_template = get_string_from_rom(STATUS_INFO_REQUEST_PAYLOAD);
      char *request_payload = set_string_parameters(status_info_request_payload_template, status_info_request_payload_template_parameters);

      FREE(device_name);
      FREE(status_info_request_payload_template);

      char *request_template = get_string_from_rom(STATUS_INFO_POST_REQUEST);
      unsigned short request_payload_length = strnlen(request_payload, 0xFFFF);
      char request_payload_length_string[3];
      sprintf(request_payload_length_string, "%d", request_payload_length);
      char *server_ip_address = get_string_from_rom(SERVER_IP_ADDRESS);
      char *request_template_parameters[] = {request_payload_length_string, server_ip_address, request_payload, NULL};
      char *request = set_string_parameters(request_template, request_template_parameters);

      FREE(request_payload);
      FREE(request_template);
      FREE(server_ip_address);

      #ifdef ALLOW_USE_PRINTF
      printf("Request created:\n<<<\n%s>>>\n", request);
      #endif

      struct espconn *connection = (struct espconn *) ZALLOC(sizeof(struct espconn), __LINE__);
      struct connection_user_data *user_data = (struct connection_user_data *) ZALLOC(sizeof(struct connection_user_data), __LINE__);

      user_data->response_received = false;
      user_data->timeout_request_supervisor_task = NULL;
      user_data->request = request;
      user_data->response = NULL;
      user_data->execute_on_succeed = status_request_on_succeed_callback;
      user_data->execute_on_error = status_request_on_error_callback;
      user_data->request_max_duration_time = REQUEST_MAX_DURATION_TIME;
      connection->reserve = user_data;
      connection->type = ESPCONN_TCP;
      connection->state = ESPCONN_NONE;

      // remote IP of TCP server
      unsigned char tcp_server_ip[] = {SERVER_IP_ADDRESS_1, SERVER_IP_ADDRESS_2, SERVER_IP_ADDRESS_3, SERVER_IP_ADDRESS_4};

      connection->proto.tcp = &user_tcp;
      memcpy(&connection->proto.tcp->remote_ip, tcp_server_ip, 4);
      connection->proto.tcp->remote_port = SERVER_PORT;
      connection->proto.tcp->local_port = espconn_port(); // local port of ESP8266

      espconn_regist_connectcb(connection, successfull_connected_tcp_handler_callback);
      espconn_regist_disconcb(connection, successfull_disconnected_tcp_handler_callback);
      espconn_regist_reconcb(connection, tcp_connection_error_handler_callback);
      espconn_regist_sentcb(connection, tcp_request_successfully_sent_handler_callback);
      espconn_regist_recvcb(connection, tcp_response_received_handler_callback);
      //espconn_regist_write_finish(&connection, tcp_request_successfully_written_into_buffer_handler_callback);

      establish_connection(connection);
   }
}

void send_general_request_task(void *pvParameters) {
   #ifdef ALLOW_USE_PRINTF
   printf("\n send_general_request_task has been created. Time: %u\n", milliseconds_counter_g);
   #endif

   if (read_flag(general_flags, UPDATE_FIRMWARE_FLAG)) {
      vTaskDelete(NULL);
   }

   GeneralRequestType request_type = (GeneralRequestType) pvParameters;

   for (;;) {
      xSemaphoreTake(requests_mutex_g, portMAX_DELAY);

      #ifdef ALLOW_USE_PRINTF
      printf("\n send_general_request_task started. Time: %u\n", milliseconds_counter_g);
      #endif

      if (!read_output_pin_state(AP_CONNECTION_STATUS_LED_PIN)) {
         #ifdef ALLOW_USE_PRINTF
         printf("\n Can't send general request, because not connected to AP. Time: %u\n", milliseconds_counter_g);
         #endif

         vTaskDelay(2000 / portTICK_RATE_MS);
         continue;
      }

      if (read_flag(general_flags, REQUEST_ERROR_OCCURRED_FLAG)) {
         reset_flag(&general_flags, REQUEST_ERROR_OCCURRED_FLAG);

         vTaskDelay(REQUEST_IDLE_TIME_ON_ERROR);
      }

      char *request_template = NULL;
      char *alarm_source = NULL;

      if (request_type == ALARM) {
         request_template = get_string_from_rom(ALARM_GET_REQUEST);
         alarm_source = get_string_from_rom(MOTION_SENSOR);
      } else if (request_type == FALSE_ALARM) {
         request_template = get_string_from_rom(FALSE_ALARM_GET_REQUEST);
         alarm_source = get_string_from_rom(MW_LED);
      }

      char *server_ip_address = get_string_from_rom(SERVER_IP_ADDRESS);
      char *request_template_parameters[] = {alarm_source, server_ip_address, NULL};
      char *request = set_string_parameters(request_template, request_template_parameters);

      FREE(alarm_source);
      FREE(request_template);
      FREE(server_ip_address);

      #ifdef ALLOW_USE_PRINTF
      printf("Request created:\n<<<\n%s>>>\n", request);
      #endif

      struct espconn *connection = (struct espconn *) ZALLOC(sizeof(struct espconn), __LINE__);
      struct connection_user_data *user_data = (struct connection_user_data *) ZALLOC(sizeof(struct connection_user_data), __LINE__);

      user_data->response_received = false;
      user_data->timeout_request_supervisor_task = NULL;
      user_data->request = request;
      user_data->response = NULL;
      user_data->execute_on_succeed = general_request_on_succeed_callback;
      user_data->execute_on_error = general_request_on_error_callback;
      user_data->parent_task = xTaskGetCurrentTaskHandle();
      user_data->request_max_duration_time = REQUEST_MAX_DURATION_TIME;
      connection->reserve = user_data;
      connection->type = ESPCONN_TCP;
      connection->state = ESPCONN_NONE;

      // remote IP of TCP server
      unsigned char tcp_server_ip[] = {SERVER_IP_ADDRESS_1, SERVER_IP_ADDRESS_2, SERVER_IP_ADDRESS_3, SERVER_IP_ADDRESS_4};

      connection->proto.tcp = &user_tcp;
      memcpy(&connection->proto.tcp->remote_ip, tcp_server_ip, 4);
      connection->proto.tcp->remote_port = SERVER_PORT;
      connection->proto.tcp->local_port = espconn_port(); // local port of ESP8266

      espconn_regist_connectcb(connection, successfull_connected_tcp_handler_callback);
      espconn_regist_disconcb(connection, successfull_disconnected_tcp_handler_callback);
      espconn_regist_reconcb(connection, tcp_connection_error_handler_callback);
      espconn_regist_sentcb(connection, tcp_request_successfully_sent_handler_callback);
      espconn_regist_recvcb(connection, tcp_response_received_handler_callback);
      //espconn_regist_write_finish(&connection, tcp_request_successfully_written_into_buffer_handler_callback);

      establish_connection(connection);
   }
}

void wifi_event_handler_callback(System_Event_t *event) {
   switch (event->event_id) {
      case EVENT_STAMODE_CONNECTED:
         pin_output_set(AP_CONNECTION_STATUS_LED_PIN);
         turn_motion_detector_on();
         break;
      case EVENT_STAMODE_DISCONNECTED:
         pin_output_reset(AP_CONNECTION_STATUS_LED_PIN);
         pin_output_reset(SERVER_AVAILABILITY_STATUS_LED_PIN);
         break;
   }
}

void set_default_wi_fi_settings() {
   wifi_station_set_auto_connect(false);
   wifi_station_set_reconnect_policy(false);
   wifi_station_dhcpc_stop();
   wifi_set_opmode(STATION_MODE);

   STATION_STATUS station_status = wifi_station_get_connect_status();
   if (station_status == STATION_GOT_IP) {
      wifi_station_disconnect();
   }

   struct station_config station_config_settings;

   wifi_station_get_config_default(&station_config_settings);

   char *default_access_point_name = get_string_from_rom(ACCESS_POINT_NAME);
   char *default_access_point_password = get_string_from_rom(ACCESS_POINT_PASSWORD);

   if (strncmp(default_access_point_name, station_config_settings.ssid, 32) != 0
         || strncmp(default_access_point_password, station_config_settings.password, 64) != 0) {
      struct station_config station_config_settings_to_save;

      memcpy(&station_config_settings_to_save.ssid, default_access_point_name, 32);
      memcpy(&station_config_settings_to_save.password, default_access_point_password, 64);
      wifi_station_set_config(&station_config_settings_to_save);
   }
   FREE(default_access_point_name);
   FREE(default_access_point_password);

   struct ip_info current_ip_info;
   wifi_get_ip_info(STATION_IF, &current_ip_info);
   char *current_ip = ipaddr_ntoa(&current_ip_info.ip);
   char *own_ip_address = get_string_from_rom(OWN_IP_ADDRESS);

   if (strncmp(current_ip, own_ip_address, 15) != 0) {
      char *own_netmask = get_string_from_rom(OWN_NETMASK);
      char *own_getaway_address = get_string_from_rom(OWN_GETAWAY_ADDRESS);
      struct ip_info ip_info_to_set;

      ip_info_to_set.ip.addr = ipaddr_addr(own_ip_address);
      ip_info_to_set.netmask.addr = ipaddr_addr(own_netmask);
      ip_info_to_set.gw.addr = ipaddr_addr(own_getaway_address);
      wifi_set_ip_info(STATION_IF, &ip_info_to_set);
      FREE(own_netmask);
      FREE(own_getaway_address);
   }
   FREE(current_ip);
   FREE(own_ip_address);
}

void ignore_alarms() {
   set_flag(&general_flags, IGNORE_ALARMS_FLAG);
   os_timer_disarm(&ignore_alarms_timer_g);
   os_timer_setfn(&ignore_alarms_timer_g, (os_timer_func_t *) stop_ignoring_alarms_timer_callback, NULL);
   os_timer_arm(&ignore_alarms_timer_g, IGNORE_ALARMS_TIMEOUT_SEC * 1000, 0);
}

void ignore_false_alarms() {
   set_flag(&general_flags, IGNORE_FALSE_ALARMS_FLAG);
   os_timer_disarm(&ignore_false_alarms_timer_g);
   os_timer_setfn(&ignore_false_alarms_timer_g, (os_timer_func_t *) stop_ignoring_false_alarms_timer_callback, NULL);
   os_timer_arm(&ignore_false_alarms_timer_g, IGNORE_FALSE_ALARMS_TIMEOUT_SEC * 1000, 0);
}

void stop_ignoring_alarms_timer_callback() {
   reset_flag(&general_flags, IGNORE_ALARMS_FLAG);
}

void stop_ignoring_false_alarms_timer_callback() {
   reset_flag(&general_flags, IGNORE_FALSE_ALARMS_FLAG);
}

void recheck_false_alarm_callback() {
   if (!read_flag(general_flags, IGNORE_FALSE_ALARMS_FLAG)) {
      // Alarm still wasn't sent
      ignore_false_alarms();
      //xTaskCreate(send_general_request_task, "send_general_request_task", 256, (void *) FALSE_ALARM, 1, NULL);
   }
}

void read_pin_state_timer_callback(void *arg) {
   unsigned int status = (unsigned int) arg;

   if (status == MOTION_DETECTOR_INPUT_PIN) {
      gpio_pin_intr_state_set(MOTION_DETECTOR_INPUT_PIN_ID, GPIO_PIN_INTR_POSEDGE); // See pins_config

      if (!read_flag(general_flags, IGNORE_MOTION_DETECTOR_FLAG) &&
            read_input_pin_state(MOTION_DETECTOR_INPUT_PIN) && !read_flag(general_flags, IGNORE_ALARMS_FLAG)) {
         ignore_alarms();
         ignore_false_alarms();

         if (!read_flag(general_flags, MANUALLY_IGNORE_ALARMS_FLAG)) {
            xTaskCreate(beep_task, "beep_task", 200, NULL, 1, NULL);
         }
         xTaskCreate(send_general_request_task, "send_general_request_task", 256, (void *) ALARM, 2, NULL);
      }
   } else if (status == MOTION_DETECTOR_INPUT_MW_LED_PIN) {
      gpio_pin_intr_state_set(MOTION_DETECTOR_INPUT_MW_LED_PIN_ID, GPIO_PIN_INTR_NEGEDGE); // See pins_config

      if (!read_flag(general_flags, IGNORE_MOTION_DETECTOR_FLAG) &&
            !read_input_pin_state(MOTION_DETECTOR_INPUT_MW_LED_PIN) && !read_flag(general_flags, IGNORE_FALSE_ALARMS_FLAG)) {
         os_timer_disarm(&recheck_false_alarm_timer_g);
         os_timer_setfn(&recheck_false_alarm_timer_g, (os_timer_func_t *) recheck_false_alarm_callback, NULL);
         os_timer_arm(&recheck_false_alarm_timer_g, RECHECK_FALSE_ALARMS_STATE_TIMEOUT_SEC * 1000, 0);
      }
   }
}

void arm_pin_state_timer(unsigned char pin_number, unsigned int status) {
   gpio_pin_intr_state_set(pin_number, GPIO_PIN_INTR_DISABLE);

   os_timer_t *timer = &pin_state_timers_g[pin_number];

   os_timer_disarm(timer);
   os_timer_setfn(timer, (os_timer_func_t *) read_pin_state_timer_callback, (void *) status);
   os_timer_arm(timer, 300, 0);
}

void pins_interrupt_handler() {
   unsigned int status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
   //clear interrupt status
   GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, status);

   if (status == MOTION_DETECTOR_INPUT_PIN) {
      arm_pin_state_timer(MOTION_DETECTOR_INPUT_PIN_ID, status);
   } else if (status == MOTION_DETECTOR_INPUT_MW_LED_PIN) {
      arm_pin_state_timer(MOTION_DETECTOR_INPUT_MW_LED_PIN_ID, status);
   }
}

void pins_config() {
   GPIO_ConfigTypeDef output_pins;
   output_pins.GPIO_Mode = GPIO_Mode_Output;
   output_pins.GPIO_Pin = AP_CONNECTION_STATUS_LED_PIN | SERVER_AVAILABILITY_STATUS_LED_PIN | BUZZER_PIN | MOTION_DETECTOR_ENABLE_PIN;
   pin_output_reset(AP_CONNECTION_STATUS_LED_PIN);
   pin_output_reset(SERVER_AVAILABILITY_STATUS_LED_PIN);
   pin_output_reset(BUZZER_PIN);
   gpio_config(&output_pins);

   GPIO_ConfigTypeDef input_pins;
   input_pins.GPIO_Mode = GPIO_Mode_Input;
   input_pins.GPIO_Pin = MOTION_DETECTOR_INPUT_PIN;
   input_pins.GPIO_Pullup = GPIO_PullUp_EN;
   gpio_config(&input_pins);
   gpio_pin_intr_state_set(MOTION_DETECTOR_INPUT_PIN_ID, GPIO_PIN_INTR_POSEDGE);

   // There is +5V on LED when it is not active
   input_pins.GPIO_Pin = MOTION_DETECTOR_INPUT_MW_LED_PIN;
   input_pins.GPIO_Pullup = GPIO_PullUp_DIS;
   gpio_config(&input_pins);
   gpio_pin_intr_state_set(MOTION_DETECTOR_INPUT_MW_LED_PIN_ID, GPIO_PIN_INTR_NEGEDGE);

   gpio_intr_handler_register(pins_interrupt_handler, NULL);
   enable_pins_interrupt();
}

void uart_config() {
   UART_WaitTxFifoEmpty(UART0);

   UART_ConfigTypeDef uart_config;
   uart_config.baud_rate         = 115200;
   uart_config.data_bits         = UART_WordLength_8b;
   uart_config.parity            = USART_Parity_None;
   uart_config.stop_bits         = USART_StopBits_1;
   uart_config.flow_ctrl         = USART_HardwareFlowControl_None;
   uart_config.UART_RxFlowThresh = 120;
   uart_config.UART_InverseMask  = UART_None_Inverse;
   UART_ParamConfig(UART0, &uart_config);

   UART_IntrConfTypeDef uart_intr;
   uart_intr.UART_IntrEnMask = UART_RXFIFO_TOUT_INT_ENA | UART_FRM_ERR_INT_ENA | UART_RXFIFO_FULL_INT_ENA;
   uart_intr.UART_RX_FifoFullIntrThresh = 30;
   uart_intr.UART_RX_TimeOutIntrThresh = 2;
   uart_intr.UART_TX_FifoEmptyIntrThresh = 20;
   UART_IntrConfig(UART0, &uart_intr);

   UART_SetPrintPort(UART0);
}

void testing_task(void *pvParameters) {
   for (;;) {
      read_pin_state_timer_callback((void *) MOTION_DETECTOR_INPUT_PIN);

      vTaskDelay(10000 / portTICK_RATE_MS);
   }
}

void user_init(void) {
   pins_config();
   turn_motion_detector_off();
   uart_config();

   start_100millisecons_counter();

   vTaskDelay(5000 / portTICK_RATE_MS);

   #ifdef ALLOW_USE_PRINTF
   printf("\n Software is running from: %s\n", system_upgrade_userbin_check() ? "user2.bin" : "user1.bin");
   #endif

   wifi_set_event_handler_cb(wifi_event_handler_callback);
   set_default_wi_fi_settings();
   espconn_init();

   xTaskCreate(autoconnect_task, "autoconnect_task", 256, NULL, 1, NULL);
   xTaskCreate(scan_access_point_task, "scan_access_point_task", 256, NULL, 1, NULL);

   vSemaphoreCreateBinary(status_request_semaphore_g);
   xSemaphoreGive(status_request_semaphore_g);
   requests_mutex_g = xSemaphoreCreateMutex();

   xTaskCreate(send_status_requests_task, "send_status_requests_task", 256, NULL, 1, NULL);

   //xTaskCreate(testing_task, "testing_task", 200, NULL, 1, NULL);
}

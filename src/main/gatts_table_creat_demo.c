/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/****************************************************************************
*
* This file is for gatt server. It can send adv data, be connected by client.
* Run the gatt_client demo, the client demo will automatically connect to the gatt_server_service_table demo.
* Client demo will enable gatt_server_service_table's notify after connection. Then two devices will exchange
* data.
*
****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "gatts_table_creat_demo.h"
#include "esp_gatt_common_api.h"

#include "esp_sleep.h"
#include "esp_log.h"
#include "esp32/ulp.h"
#include "driver/touch_pad.h"
#include "driver/adc.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"


#define GPIO_OUTPUT_IO_0    2
#define GPIO_OUTPUT_IO_1    13
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))
#define GPIO_INPUT_IO_0     4
#define GPIO_INPUT_IO_1     5
//#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0)
#define ESP_INTR_FLAG_DEFAULT 0

#define GATTS_TABLE_TAG "GATTS_TABLE_DEMO"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
//#define SAMPLE_DEVICE_NAME          "ESP_GATTS_DEMO"
#define SAMPLE_DEVICE_NAME          "Aquarius"
#define SVC_INST_ID                 0

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 100
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

#define wakeup_time_sec             18


static int cnt = 0;

static RTC_DATA_ATTR bool bleConnected = false;
static RTC_DATA_ATTR bool fillingWater = false;

static RTC_DATA_ATTR struct timeval sleep_enter_time;

static uint8_t adv_config_done       = 0;

//uint16_t heart_rate_handle_table[HRS_IDX_NB];
uint16_t aquarius_handle_table[AQUARIUS_IDX_NB];

static struct timeval systemTimeVal;
static struct timezone systemTimeZone;

static time_t calendarNow;
static struct tm timeinfo;

static RTC_DATA_ATTR struct timeAlarm {
  bool active;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
}alarm[3];


typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

//#define CONFIG_SET_RAW_ADV_DATA
#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power*/
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF, 0x00,
        /* device name */
        0x0f, 0x09, 'A', 'q', 'u', 'a', 'r', 'i', 'u', 's'
};
static uint8_t raw_scan_rsp_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power */
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF,0x00
};

#else
static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    //0x12, 0x34, 0x56, 0x78, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC
    0xBC, 0x9A, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = false,
    .min_interval        = 0x20,
    .max_interval        = 0x40,
    .appearance          = 0x00,
    .manufacturer_len    = 0,    //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //test_manufacturer,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag                = 0,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = false,
    .min_interval        = 0x20,
    .max_interval        = 0x40,
    .appearance          = 0x00,
    .manufacturer_len    = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = 16,
    .p_service_uuid      = service_uuid,
    .flag                = 0,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst heart_rate_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/* Service */
static const uint16_t GATTS_SERVICE_UUID_AQUARIUS  = 0x00FF;
static const uint16_t GATTS_CHAR_UUID_BATTERY      = 0xFF01;
static const uint16_t GATTS_CHAR_UUID_CURRENT_TIME = 0xFF02;
static const uint16_t GATTS_CHAR_UUID_POTS         = 0xFF03;
static const uint16_t GATTS_CHAR_UUID_NEW_TIME_POINT = 0xFF04;
static const uint16_t GATTS_CHAR_UUID_COMMAND      = 0xFF05;
static const uint16_t GATTS_CHAR_UUID_LOG_EVENT    = 0xFF06;


static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write          = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t heart_measurement_ccc[2]      = {0x00, 0x00};
static const uint8_t char_value[4]                 = {0x11, 0x22, 0x33, 0x44};


/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[AQUARIUS_IDX_NB] =
{
    // Service Declaration
    [IDX_SVC_AQUARIUS]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_AQUARIUS), (uint8_t *)&GATTS_SERVICE_UUID_AQUARIUS}},

    /* Characteristic Declaration */
    [IDX_CHAR_BATTERY]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_BATTERY] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_BATTERY, ESP_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    /* Client Characteristic Configuration Descriptor */
    /*[IDX_CHAR_CFG_A]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},*/

    /* Characteristic Declaration */
    [IDX_CHAR_CURRENT_TIME]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_CURRENT_TIME]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_CURRENT_TIME, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    /* Characteristic Declaration */
    [IDX_CHAR_COMMAND]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_COMMAND]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_COMMAND, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    /*Characteristic Declaration */
    [IDX_CHAR_POTS]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_POTS] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_POTS, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    /*Characteristic Declaration */
    [IDX_CHAR_NEW_TIME_POINT]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_NEW_TIME_POINT] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_NEW_TIME_POINT, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    /*Characteristic Declaration */
    [IDX_CHAR_LOG_EVENT]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_LOG_EVENT] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_LOG_EVENT, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
        GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    #ifdef CONFIG_SET_RAW_ADV_DATA
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #else
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #endif
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
            }else{
                ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "update connetion params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL) {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    } else {
        if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_OFFSET;
        } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp){
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL){
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
            }
            free(gatt_rsp);
        }else{
            ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
        }
    }
    if (status != ESP_GATT_OK){
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;

}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf){
        esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TABLE_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
    #ifdef CONFIG_SET_RAW_ADV_DATA
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if (raw_adv_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if (raw_scan_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    #else
            //config adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            //config scan response data
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    #endif
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, AQUARIUS_IDX_NB, SVC_INST_ID);
            if (create_attr_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
        }
       	    break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
       	    break;
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep){
                ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
                esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);


                uint16_t dataHandle = (int)param->write.handle;
                uint8_t* dataValue = param->write.value;

                gpio_config_t io_conf;

                switch (dataHandle) {
                  case 44:
                    ESP_LOGI("gatt write event","Current Time written");
                    uint32_t systemTime = 0;
                    int index;
                    for(index=0;index<4;index++) {
                      ESP_LOGI("dataValue","%d", *dataValue);
                      systemTime = systemTime << 8;
                      systemTime = systemTime + *dataValue;
                      ESP_LOGI("current system time","%d", systemTime);
                      dataValue++;
                    }

                    systemTimeVal.tv_sec = systemTime;
                    systemTimeVal.tv_usec = 0;
                    systemTimeZone.tz_minuteswest = -330;
                    systemTimeZone.tz_dsttime = 0;
                    settimeofday(&systemTimeVal, NULL);

                    time(&calendarNow);
                    localtime_r(&calendarNow, &timeinfo);
                    char strftime_buf[64];

                    // Set timezone to Indian Standard Time and print local time
                    //setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
                    setenv("TZ", "IST-5:30", 0);
                    tzset();
                    localtime_r(&calendarNow, &timeinfo);
                    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
                    ESP_LOGI("Current Time","The current date/time in India is: %s", strftime_buf);

                  break;

                  case 46:
                    ESP_LOGI("gatt write event","Valve Command written : %d", *dataValue);
                    switch (*dataValue) {
                      case 1:
                        ESP_LOGI("Command","Flush Open");
                      break;

                      case 2:
                        ESP_LOGI("Command","Start");
                        fillingWater = true;
                        //interrupt of low level
                        io_conf.intr_type = GPIO_INTR_LOW_LEVEL;
                        //bit mask of the pins, use GPIO4/5 here
                        io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
                        //set as input mode
                        io_conf.mode = GPIO_MODE_INPUT;
                        //enable pull-up mode
                        io_conf.pull_up_en = 1;
                        //disable pull-down mode
                        io_conf.pull_down_en = 0;
                        gpio_config(&io_conf);
                        rtc_gpio_hold_dis(GPIO_OUTPUT_IO_0);
                        rtc_gpio_hold_dis(GPIO_OUTPUT_IO_1);
                        rtc_gpio_set_level(GPIO_OUTPUT_IO_0, 0);
                        rtc_gpio_set_level(GPIO_OUTPUT_IO_1, 0);
                        rtc_gpio_hold_en(GPIO_OUTPUT_IO_0);
                        rtc_gpio_hold_en(GPIO_OUTPUT_IO_1);
                      break;

                      case 3:
                        ESP_LOGI("Command","Stop");
                        fillingWater = false;
                        //interrupt of low level
                        io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
                        //bit mask of the pins, use GPIO4/5 here
                        io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
                        //set as input mode
                        io_conf.mode = GPIO_MODE_DISABLE;
                        //enable pull-up mode
                        io_conf.pull_up_en = 1;
                        //disable pull-down mode
                        io_conf.pull_down_en = 0;
                        gpio_config(&io_conf);
                        rtc_gpio_hold_dis(GPIO_OUTPUT_IO_0);
                        rtc_gpio_hold_dis(GPIO_OUTPUT_IO_1);
                        rtc_gpio_set_level(GPIO_OUTPUT_IO_0, 1);
                        rtc_gpio_set_level(GPIO_OUTPUT_IO_1, 1);
                        rtc_gpio_hold_en(GPIO_OUTPUT_IO_0);
                        rtc_gpio_hold_en(GPIO_OUTPUT_IO_1);
                      break;

                      case 4:
                        ESP_LOGI("Command","Disconnect");

                        const int ext_wakeup_pin_1 = 15;
                        const uint64_t ext_wakeup_pin_1_mask = 1 << ext_wakeup_pin_1;
                        const int ext_wakeup_pin_2 = 4;
                        const uint64_t ext_wakeup_pin_2_mask = 1 << ext_wakeup_pin_2;

                        ESP_LOGI("External Interrupts","Enabling EXT1 wakeup on pin GPIO%d\n", ext_wakeup_pin_1);
                        esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask, ESP_EXT1_WAKEUP_ALL_LOW);
                        printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
                        esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);
                        //ESP_LOGI("External Interrupts","Enabling EXT1 wakeup on pin GPIO%d\n", ext_wakeup_pin_2);
                        //esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 0);

                        ESP_LOGI("Disconnect Command","Entering deep sleep\n");
                        esp_deep_sleep_start();
                      break;

                      case 5:
                        ESP_LOGI("Command","Flush Close");
                      break;

                      default:
                        ESP_LOGI("Command","Unknown Command");
                      break;
                    }
                  break;

                  case 48:
                    ESP_LOGI("gatt write event","Number of Pots written");
                  break;

                  case 50:
                    ESP_LOGI("gatt write event","New Time Point written: Active=%d Index=%d Hour=%d Minute=%d",dataValue[0],dataValue[1],dataValue[2],dataValue[3]);
                    alarm[dataValue[1]].active = dataValue[0];
                    alarm[dataValue[1]].hour = dataValue[2];
                    alarm[dataValue[1]].minute = dataValue[3];
                  break;

                  default:
                    ESP_LOGI("gatt write event","Unknown Handle");
                  break;
                }
                /*if (aquarius_handle_table[IDX_CHAR_CFG_A] == param->write.handle && param->write.len == 2){
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    if (descr_value == 0x0001){
                        ESP_LOGI(GATTS_TABLE_TAG, "notify enable");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i % 0xff;
                        }
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, heart_rate_handle_table[IDX_CHAR_VAL_A],
                                                sizeof(notify_data), notify_data, false);
                    }else if (descr_value == 0x0002){
                        ESP_LOGI(GATTS_TABLE_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i % 0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, heart_rate_handle_table[IDX_CHAR_VAL_A],
                                            sizeof(indicate_data), indicate_data, true);
                    }
                    else if (descr_value == 0x0000){
                        ESP_LOGI(GATTS_TABLE_TAG, "notify/indicate disable ");
                    }else{
                        ESP_LOGE(GATTS_TABLE_TAG, "unknown descr value");
                        esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                    }

                }*/
                /* send response when param->write.need_rsp is true*/
                if (param->write.need_rsp){
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }else{
                /* handle prepare write */
                example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
            }
      	    break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            example_exec_write_event_env(&prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d", param->conf.status);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            bleConnected = true;
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            bleConnected = false;
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = %d", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != AQUARIUS_IDX_NB){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to AQUARIUS_IDX_NB(%d)", param->add_attr_tab.num_handle, AQUARIUS_IDX_NB);
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
                memcpy(aquarius_handle_table, param->add_attr_tab.handles, sizeof(aquarius_handle_table));
                esp_ble_gatts_start_service(aquarius_handle_table[IDX_SVC_AQUARIUS]);
            }
            break;
        }
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == heart_rate_profile_tab[idx].gatts_if) {
                if (heart_rate_profile_tab[idx].gatts_cb) {
                    heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}


static void checkAlarms() {
  time(&calendarNow);
  localtime_r(&calendarNow, &timeinfo);
  char strftime_buf[64];

  // Set timezone to Indian Standard Time and print local time
  //setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
  setenv("TZ", "IST-5:30", 0);
  tzset();
  localtime_r(&calendarNow, &timeinfo);
  strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
  ESP_LOGI("Checking Alarms","The current date/time in India is: %s. (%d) %2d:%2d", strftime_buf, timeinfo.tm_wday, timeinfo.tm_hour, timeinfo.tm_min);

  if(alarm[0].active && (timeinfo.tm_hour == alarm[0].hour) && (timeinfo.tm_min == alarm[0].minute)) {
    ESP_LOGI("Alarm Match","Alarm 0 : Hour = %d Minute = %d", timeinfo.tm_hour, timeinfo.tm_min);
    fillingWater = true;
    gpio_config_t io_conf;
    //interrupt of low level
    io_conf.intr_type = GPIO_INTR_LOW_LEVEL;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    rtc_gpio_hold_dis(GPIO_OUTPUT_IO_0);
    rtc_gpio_hold_dis(GPIO_OUTPUT_IO_1);
    rtc_gpio_set_level(GPIO_OUTPUT_IO_0, 0);
    rtc_gpio_set_level(GPIO_OUTPUT_IO_1, 0);
    rtc_gpio_hold_en(GPIO_OUTPUT_IO_0);
    rtc_gpio_hold_en(GPIO_OUTPUT_IO_1);
    alarm[2].active = 1;
    alarm[2].hour = alarm[0].hour + 1;
    if(alarm[0].hour==23) {
      alarm[2].hour = 0;
    }
    alarm[2].minute = alarm[0].minute;
  }

  if(alarm[1].active && (timeinfo.tm_hour == alarm[1].hour) && (timeinfo.tm_min == alarm[1].minute)) {
    ESP_LOGI("Alarm Match","Alarm 1 : Hour = %d Minute = %d", timeinfo.tm_hour, timeinfo.tm_min);
    fillingWater = true;
    gpio_config_t io_conf;
    //interrupt of low level
    io_conf.intr_type = GPIO_INTR_LOW_LEVEL;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    rtc_gpio_hold_dis(GPIO_OUTPUT_IO_0);
    rtc_gpio_hold_dis(GPIO_OUTPUT_IO_1);
    rtc_gpio_set_level(GPIO_OUTPUT_IO_0, 0);
    rtc_gpio_set_level(GPIO_OUTPUT_IO_1, 0);
    rtc_gpio_hold_en(GPIO_OUTPUT_IO_0);
    rtc_gpio_hold_en(GPIO_OUTPUT_IO_1);
    alarm[2].active = 1;
    alarm[2].hour = alarm[1].hour + 1;
    if(alarm[1].hour==23) {
      alarm[2].hour = 0;
    }
    alarm[2].minute = alarm[1].minute;
  }
}


static void checkStopAlarms() {
  time(&calendarNow);
  localtime_r(&calendarNow, &timeinfo);
  char strftime_buf[64];

  // Set timezone to Indian Standard Time and print local time
  //setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
  setenv("TZ", "IST-5:30", 0);
  tzset();
  localtime_r(&calendarNow, &timeinfo);
  strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
  ESP_LOGI("Checking Alarms","The current date/time in India is: %s. (%d) %2d:%2d", strftime_buf, timeinfo.tm_wday, timeinfo.tm_hour, timeinfo.tm_min);

  if(alarm[2].active && (timeinfo.tm_hour == alarm[2].hour) && (timeinfo.tm_min == alarm[2].minute)) {
    ESP_LOGI("Alarm Match","Stopping Alarm : Hour = %d Minute = %d", timeinfo.tm_hour, timeinfo.tm_min);
    fillingWater = false;
    gpio_config_t io_conf;
    //interrupt of low level
    io_conf.intr_type = GPIO_INTR_LOW_LEVEL;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    rtc_gpio_hold_dis(GPIO_OUTPUT_IO_0);
    rtc_gpio_hold_dis(GPIO_OUTPUT_IO_1);
    rtc_gpio_set_level(GPIO_OUTPUT_IO_0, 1);
    rtc_gpio_set_level(GPIO_OUTPUT_IO_1, 1);
    rtc_gpio_hold_en(GPIO_OUTPUT_IO_0);
    rtc_gpio_hold_en(GPIO_OUTPUT_IO_1);
    alarm[2].active = 0;
  }
}


void app_main()
{
  esp_err_t ret;

  /* Initialize NVS. */
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK( ret );

    switch (esp_sleep_get_wakeup_cause()) {
      case ESP_SLEEP_WAKEUP_EXT1: {
        uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
        if (wakeup_pin_mask != 0) {
            int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
            printf("Wake up from User Button on GPIO %d\n", pin);
        } else {
            printf("Wake up from GPIO\n");
        }

        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        ret = esp_bt_controller_init(&bt_cfg);
        if (ret) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed", __func__);
            return;
        }

        ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (ret) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed", __func__);
            return;
        }

        ret = esp_bluedroid_init();
        if (ret) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed", __func__);
            return;
        }

        ret = esp_bluedroid_enable();
        if (ret) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed", __func__);
            return;
        }

        ret = esp_ble_gatts_register_callback(gatts_event_handler);
        if (ret){
            ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
            return;
        }

        ret = esp_ble_gap_register_callback(gap_event_handler);
        if (ret){
            ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
            return;
        }

        ret = esp_ble_gatts_app_register(ESP_APP_ID);
        if (ret){
            ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
            return;
        }

        esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
        if (local_mtu_ret){
            ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
        }

        const int ext_wakeup_pin_1 = 15;
        const uint64_t ext_wakeup_pin_1_mask = 1 << ext_wakeup_pin_1;
        const int ext_wakeup_pin_2 = 4;
        const uint64_t ext_wakeup_pin_2_mask = 1 << ext_wakeup_pin_2;

        ESP_LOGI("External Interrupts","Enabling EXT1 wakeup on pins GPIO%d, GPIO%d\n", ext_wakeup_pin_1, ext_wakeup_pin_2);
        esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask, ESP_EXT1_WAKEUP_ALL_LOW);

        gpio_config_t io_conf;

        //interrupt of low level
        io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
        //bit mask of the pins, use GPIO4/5 here
        io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
        //set as input mode
        io_conf.mode = GPIO_MODE_INPUT;
        //enable pull-up mode
        io_conf.pull_up_en = 1;
        //enable pull-down mode
        io_conf.pull_down_en = 0;
        gpio_config(&io_conf);

        while (1) {
          while(bleConnected || fillingWater) {
            time(&calendarNow);
            localtime_r(&calendarNow, &timeinfo);
            char strftime_buf[64];

            // Set timezone to Indian Standard Time and print local time
            //setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
            setenv("TZ", "IST-5:30", 0);
            tzset();
            localtime_r(&calendarNow, &timeinfo);
            strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
            //ESP_LOGI("Current Time","The current date/time in India is: %s. (%d) %2d:%2d", strftime_buf, timeinfo.tm_wday, timeinfo.tm_hour, timeinfo.tm_min);
            vTaskDelay(20000 / portTICK_PERIOD_MS);
            checkAlarms();
            checkStopAlarms();
            if(fillingWater) {
              if(gpio_get_level(GPIO_NUM_4) == 1) {
                fillingWater = false;
                ESP_LOGI("20 second poll","Water Sensor found to be 0");
                rtc_gpio_hold_dis(GPIO_OUTPUT_IO_0);
                rtc_gpio_hold_dis(GPIO_OUTPUT_IO_1);
                rtc_gpio_set_level(GPIO_OUTPUT_IO_0, 1);
                rtc_gpio_set_level(GPIO_OUTPUT_IO_1, 1);
                rtc_gpio_hold_en(GPIO_OUTPUT_IO_0);
                rtc_gpio_hold_en(GPIO_OUTPUT_IO_1);
              }
            }
          }
        }

        break;
      }

      case ESP_SLEEP_WAKEUP_TIMER: {
        ESP_LOGI("app_main","timer based wake up");
        gpio_config_t io_conf;
        //interrupt of low level
        io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
        //bit mask of the pins, use GPIO4/5 here
        io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
        //set as input mode
        io_conf.mode = GPIO_MODE_INPUT;
        //enable pull-up mode
        io_conf.pull_up_en = 1;
        //disable pull-down mode
        io_conf.pull_down_en = 0;
        gpio_config(&io_conf);

        vTaskDelay(2000 / portTICK_RATE_MS);

        checkAlarms();
        checkStopAlarms();
        if(fillingWater) {
          if(gpio_get_level(GPIO_NUM_4) == 1) {
            fillingWater = false;
            ESP_LOGI("20 second poll","Water Sensor found to be 0");
            rtc_gpio_hold_dis(GPIO_OUTPUT_IO_0);
            rtc_gpio_hold_dis(GPIO_OUTPUT_IO_1);
            rtc_gpio_set_level(GPIO_OUTPUT_IO_0, 1);
            rtc_gpio_set_level(GPIO_OUTPUT_IO_1, 1);
            rtc_gpio_hold_en(GPIO_OUTPUT_IO_0);
            rtc_gpio_hold_en(GPIO_OUTPUT_IO_1);
          }
        }

        const int ext_wakeup_pin_1 = 15;
        const uint64_t ext_wakeup_pin_1_mask = 1 << ext_wakeup_pin_1;
        const int ext_wakeup_pin_2 = 4;
        const uint64_t ext_wakeup_pin_2_mask = 1 << ext_wakeup_pin_2;
        ESP_LOGI("External Interrupts","Enabling EXT1 wakeup on pin GPIO%d\n", ext_wakeup_pin_1);
        esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask, ESP_EXT1_WAKEUP_ALL_LOW);
        printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
        esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

        //ESP_LOGI("External Interrupts","Enabling EXT1 wakeup on pin GPIO%d\n", ext_wakeup_pin_2);
        //esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 0);

        ESP_LOGI("Timer wakeup","Entering deep sleep\n");
        esp_deep_sleep_start();
        //printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
        break;
      }

      case ESP_SLEEP_WAKEUP_UNDEFINED:
      default:
        printf("Not a deep sleep reset\n");

        if(rtc_gpio_is_valid_gpio(2)) {
          ESP_LOGI("Power Up","Pin 2 is valid RTC GPIO");
        } else {
          ESP_LOGI("Power Up","Pin 2 is not valid RTC GPIO");
        }
        if(rtc_gpio_is_valid_gpio(13)) {
          ESP_LOGI("Power Up","Pin 13 is valid RTC GPIO");
        } else {
          ESP_LOGI("Power Up","Pin 13 is not valid RTC GPIO");
        }

        ret = rtc_gpio_init(GPIO_OUTPUT_IO_0);
        if(ret) {
          ESP_LOGI("GPIO","Could not initialize GPIO 2");
          return;
        }
        ret = rtc_gpio_init(GPIO_OUTPUT_IO_1);
        if(ret) {
          ESP_LOGI("GPIO","Could not initialize GPIO 13");
          return;
        }
        ret = rtc_gpio_set_direction(GPIO_OUTPUT_IO_0, RTC_GPIO_MODE_OUTPUT_ONLY);
        if(ret) {
          ESP_LOGI("GPIO","Could not set direction of GPIO 2");
          return;
        }
        ret = rtc_gpio_set_direction(GPIO_OUTPUT_IO_1, RTC_GPIO_MODE_OUTPUT_ONLY);
        if(ret) {
          ESP_LOGI("GPIO","Could not set direction of GPIO 13");
          return;
        }
        ret = rtc_gpio_pullup_en(GPIO_OUTPUT_IO_0);
        if(ret) {
          ESP_LOGI("GPIO","Could not enable pull up of GPIO 2");
          return;
        }
        ret = rtc_gpio_pullup_en(GPIO_OUTPUT_IO_1);
        if(ret) {
          ESP_LOGI("GPIO","Could not enable pull up of GPIO 13");
          return;
        }
        ret = rtc_gpio_set_level(GPIO_OUTPUT_IO_0, 1);
        if(ret) {
          ESP_LOGI("GPIO","Could not set level of GPIO 2");
          return;
        }
        ret = rtc_gpio_set_level(GPIO_OUTPUT_IO_1, 1);
        if(ret) {
          ESP_LOGI("GPIO","Could not set level of GPIO 13");
          return;
        }
        ret = rtc_gpio_hold_en(GPIO_OUTPUT_IO_0);
        if(ret) {
          ESP_LOGI("GPIO","Could not enable hold of GPIO 2");
          return;
        }
        ret = rtc_gpio_hold_en(GPIO_OUTPUT_IO_1);
        if(ret) {
          ESP_LOGI("GPIO","Could not enable hold of GPIO 13");
          return;
        }


        const int ext_wakeup_pin_1 = 15;
        const uint64_t ext_wakeup_pin_1_mask = 1 << ext_wakeup_pin_1;
        const int ext_wakeup_pin_2 = 4;
        const uint64_t ext_wakeup_pin_2_mask = 1 << ext_wakeup_pin_2;

        ESP_LOGI("External Interrupts","Enabling EXT1 wakeup on pin GPIO%d\n", ext_wakeup_pin_1);
        esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask, ESP_EXT1_WAKEUP_ALL_LOW);
        printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
        esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

        //ESP_LOGI("External Interrupts","Enabling EXT1 wakeup on pin GPIO%d\n", ext_wakeup_pin_2);
        //esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 0);

        ESP_LOGI("First boot","Entering deep sleep\n");
        esp_deep_sleep_start();


    }
}

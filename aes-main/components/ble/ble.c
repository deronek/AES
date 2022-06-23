#include "ble.h"
#include "esp_gatt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "app_manager.h"
// #include "gatt/gatt_int.h"

///// BLE Variable/////////////////////////////

#define GATTS_TABLE_TAG "GATTS_SPP"

#define SPP_PROFILE_NUM 1
#define SPP_PROFILE_APP_IDX 0
#define ESP_SPP_APP_ID 0x56
#define SAMPLE_DEVICE_NAME "ESP_SPP_SERVER" // The Device Name Characteristics in GAP
#define SPP_SVC_INST_ID 0

/// SPP Service
static const uint16_t spp_service_uuid = 0xABF0;

/// Characteristic UUID
#define ESP_GATT_UUID_CTRL_INDICATION 0xABF1
#define ESP_GATT_UUID_DATA_NOTIFICATION 0xABF2

#define ESP_GATT_UUID_HEARTBEAT 0xABF5

#define BLE_CTRL_MAX_LEN (20)
#define BLE_DATA_MAX_LEN (512)

// attribute state machine
enum
{
    BLE_IDX_SVC,

    BLE_IDX_CTRL_INDIC_CHAR,
    BLE_IDX_CTRL_INDIC_VAL,

    BLE_IDX_DATA_NTY_CHAR,
    BLE_IDX_DATA_NTY_VAL,
    BLE_IDX_DATA_NTY_CFG,

    BLE_IDX_HEARTBEAT_CHAR,
    BLE_IDX_HEARTBEAT_VAL,
    BLE_IDX_HEARTBEAT_CFG,

    BLE_IDX_NB,
};

static uint16_t spp_mtu_size = 23;

static uint8_t heartbeat_s[] = {'A', 'E', 'S', '-', '2', '0', '2', '2'};
static uint8_t heartbeat_count_num = 0;

static uint16_t spp_conn_id = 0xffff;
static uint16_t spp_handle_table[BLE_IDX_NB];

/*
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_control_indication = ESP_GATT_CHAR_PROP_BIT_INDICATE | ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_data_notification = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
// static const uint8_t char_prop_heartbeat = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const uint8_t char_prop_heartbeat = ESP_GATT_CHAR_PROP_BIT_INDICATE | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;

/// SPP Service - data receive characteristic, read&write without response
static const uint16_t ble_ctrl_uuid = ESP_GATT_UUID_CTRL_INDICATION;
static const uint8_t ble_ctrl_val[BLE_CTRL_MAX_LEN] = {0x00};

/// SPP Service - data notify characteristic, notify&read
static const uint16_t ble_data_uuid = ESP_GATT_UUID_DATA_NOTIFICATION;
static const uint8_t ble_data_val[BLE_DATA_MAX_LEN] = {0x00};
static const uint8_t ble_data_ccc[2] = {0x00, 0x00};

/// SPP Server - Heart beat characteristic, notify&write&read
static const uint16_t ble_heartbeat_uuid = ESP_GATT_UUID_HEARTBEAT;
static const uint8_t ble_heartbeat_val[sizeof(heartbeat_s)] = {0x00};
static const uint8_t ble_heartbeat_ccc[2] = {0x00, 0x00};

static bool is_connected = false;
static esp_bd_addr_t spp_remote_bda = {
    0x0,
};
static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

typedef struct spp_receive_data_node
{
    int32_t len;
    uint8_t *node_buff;
    struct spp_receive_data_node *next_node;
} spp_receive_data_node_t;

typedef struct spp_receive_data_buff
{
    int32_t node_num;
    int32_t buff_size;
    spp_receive_data_node_t *first_node;
} spp_receive_data_buff_t;

// static spp_receive_data_buff_t SppRecvDataBuff = {
//     .node_num = 0,
//     .buff_size = 0,
//     .first_node = NULL};

// static spp_receive_data_node_t *temp_spp_recv_data_node_p1 = NULL;
// static spp_receive_data_node_t *temp_spp_recv_data_node_p2 = NULL;

static esp_gatt_if_t spp_gatts_if = 0xff;

struct gatts_profile_inst
{
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

static const uint8_t spp_adv_data[] = {
    /* Flags */
    0x02, 0x01, 0x06,
    /* Complete List of 16-bit Service Class UUIDs */
    0x03, 0x03, 0xF0, 0xAB,
    /* Complete Local Name in advertising */
    0x0F, 0x09, 'A', 'E', 'S', '-', '2', '0', '2', '2'};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/// Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[BLE_IDX_NB] =
    {
        // SPP -  Service Declaration
        [BLE_IDX_SVC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

        // SPP -  data receive characteristic Declaration
        [BLE_IDX_CTRL_INDIC_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_control_indication}},

        // SPP -  data receive characteristic Value
        [BLE_IDX_CTRL_INDIC_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&ble_ctrl_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, BLE_CTRL_MAX_LEN, sizeof(ble_ctrl_val), (uint8_t *)ble_ctrl_val}},

        // SPP -  data notify characteristic Declaration
        [BLE_IDX_DATA_NTY_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_data_notification}},

        // SPP -  data notify characteristic Value
        [BLE_IDX_DATA_NTY_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&ble_data_uuid, ESP_GATT_PERM_READ, BLE_DATA_MAX_LEN, sizeof(ble_data_val), (uint8_t *)ble_data_val}},

        // SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
        [BLE_IDX_DATA_NTY_CFG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_data_ccc), (uint8_t *)ble_data_ccc}},

        // SPP -  Heart beat characteristic Declaration
        [BLE_IDX_HEARTBEAT_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_heartbeat}},

        // SPP -  Heart beat characteristic Value
        [BLE_IDX_HEARTBEAT_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&ble_heartbeat_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(ble_heartbeat_val), sizeof(ble_heartbeat_val), (uint8_t *)ble_heartbeat_val}},

        // SPP -  Heart beat characteristic - Client Characteristic Configuration Descriptor
        [BLE_IDX_HEARTBEAT_CFG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(ble_heartbeat_ccc), (uint8_t *)ble_heartbeat_ccc}},

};

//////////////////////////////////////////////////////////////////////////////

// constants
#define BLE_HEARTBEAT_PERIOD_TICKS pdMS_TO_TICKS(1000)
// packet ID for controller incoming packets
#define BLE_CONTROLLER_PACKET_HEADER 0x10

// enums

typedef enum ble_controller_request_id_type_tag
{
    REQUEST_START_DRIVING = 0x00,
    REQUEST_STOP_DRIVING = 0x01,
    REQUEST_MPU9255_CALIBRATE = 0x02
} ble_controller_request_id_type;

typedef enum ble_controller_response_id_type_tag
{
    RESPONSE_OK = 0x00,
    RESPONSE_FINISHED = 0x02,
    RESPONSE_FAIL = 0xFF
} ble_controller_response_id_type;

// global variables
bool ble_running = false;
QueueHandle_t ble_notify_tx_queue;
// local variables

static const char *TAG = "ble";

// function defintions
static void ble_receive_data(uint8_t id, uint8_t *packet);
static void ble_handle_task_notification(uint32_t task_notification_value);
static uint32_t ble_get_task_id_from_flag(uint32_t task_id);
static void ble_handle_tx_send(ble_notify_tx_type item);
static esp_err_t ble_send_data_notification(uint8_t *data, uint8_t len);
static void ble_handle_indication_packet(esp_ble_gatts_cb_param_t *p_data);

///////function BLE init//////////////////

// static void free_write_buffer(void);
static uint8_t find_char_and_desr_index(uint16_t handle);
// static bool store_wr_buffer(esp_ble_gatts_cb_param_t *p_data);
// static void print_write_buffer(void);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

// function declarations
void ble_init()
{
    /**
     * @todo Initialize ble
     */

    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth\n", __func__);
    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);

    /**
     * @brief Initialize data queues.
     */
    ble_notify_tx_queue = xQueueCreate(10, sizeof(ble_notify_tx_type));
    if (ble_notify_tx_queue == NULL)
    {
        abort();
    }

    return;
}

TASK ble_notify_main()
{
    ble_running = true;
    for (;;)
    {
        ble_notify_tx_type item;
        xQueueReceive(ble_notify_tx_queue, &item, portMAX_DELAY);

        ble_handle_tx_send(item);
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *)param;
    uint8_t res = 0xff;

    // ESP_LOGI(GATTS_TABLE_TAG, "event = %x\n", event);
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(TAG, "Registering BLE app ID");
        esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
        esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, sizeof(spp_adv_data));
        esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, BLE_IDX_NB, SPP_SVC_INST_ID);
        break;
    case ESP_GATTS_READ_EVT:
        ESP_LOGE(TAG, "ESP_GATTS_READ_EVT");
        res = find_char_and_desr_index(p_data->read.handle);
        // if (res == SPP_IDX_SPP_STATUS_VAL)
        // {
        //     // TODO:client read the status characteristic
        // }
        break;
    case ESP_GATTS_WRITE_EVT:
    {
        res = find_char_and_desr_index(p_data->write.handle);
        if (p_data->write.is_prep == false)
        {
            // ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT : handle = %d\n", res);
            // if (res == SPP_IDX_SPP_COMMAND_VAL)
            // {
            //     uint8_t *spp_cmd_buff = NULL;
            //     spp_cmd_buff = (uint8_t *)malloc((spp_mtu_size - 3) * sizeof(uint8_t));
            //     if (spp_cmd_buff == NULL)
            //     {
            //         ESP_LOGE(GATTS_TABLE_TAG, "%s malloc failed\n", __func__);
            //         break;
            //     }
            //     memset(spp_cmd_buff, 0x0, (spp_mtu_size - 3));
            //     memcpy(spp_cmd_buff, p_data->write.value, p_data->write.len);
            //     xQueueSend(cmd_cmd_queue, &spp_cmd_buff, 10 / portTICK_PERIOD_MS);
            // }
            if (res == BLE_IDX_HEARTBEAT_VAL)
            {
                // for (int i = 0; i < p_data->write.len; ++i)
                // {
                //     putchar(p_data->write.value[i]);
                // }
                // putchar('\n');

                if ((p_data->write.len == sizeof(heartbeat_s)) && (memcmp(heartbeat_s, p_data->write.value, sizeof(heartbeat_s)) == 0))
                {
                    // ESP_LOGI(TAG, "Resetting heartbeat timeout");
                    heartbeat_count_num = 0;
                }
            }
            else if (res == BLE_IDX_CTRL_INDIC_VAL)
            {
                ESP_LOGI(TAG, "Handling indication packet from controller");
                ble_handle_indication_packet(p_data);
            }
            else
            {
                ESP_LOGI(TAG, "Unknown write event, res %d", res);
                // TODO:
            }
        }
        else
        {
            ESP_LOGI(TAG, "Unknown write prepare event, res %d", res);
            // ESP_LOGE(TAG, "BLE_IDX_CTRL_INDIC_VAL");
            // // ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_PREP_WRITE_EVT : handle = %d\n", res);
            // store_wr_buffer(p_data);
        }
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
    {
        // ESP_LOGE(TAG, "ESP_GATTS_EXEC_WRITE_EVT");
        // // ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT\n");
        // if (p_data->exec_write.exec_write_flag)
        // {
        //     print_write_buffer();
        //     free_write_buffer();
        // }
        break;
    }
    case ESP_GATTS_MTU_EVT:
        spp_mtu_size = p_data->mtu.mtu;
        break;
    case ESP_GATTS_CONF_EVT:
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
        spp_conn_id = p_data->connect.conn_id;
        spp_gatts_if = gatts_if;
        is_connected = true;
        memcpy(&spp_remote_bda, &p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        heartbeat_count_num = 0;
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGW(TAG, "Disconnecting client");
        is_connected = false;
        heartbeat_count_num = 0;
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GATTS_OPEN_EVT:
        break;
    case ESP_GATTS_CANCEL_OPEN_EVT:
        break;
    case ESP_GATTS_CLOSE_EVT:
        break;
    case ESP_GATTS_LISTEN_EVT:
        break;
    case ESP_GATTS_CONGEST_EVT:
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
        if (param->add_attr_tab.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        }
        else if (param->add_attr_tab.num_handle != BLE_IDX_NB)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, BLE_IDX_NB);
        }
        else
        {
            memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
            esp_ble_gatts_start_service(spp_handle_table[BLE_IDX_SVC]);
        }
        break;
    }
    default:
        break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    // ESP_LOGE(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);

    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        // advertising start complete event to indicate advertising start successfully or failed
        if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed: %s\n", esp_err_to_name(err));
        }
        break;
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    // ESP_LOGI(GATTS_TABLE_TAG, "EVT %d, gatts if %d\n", event, gatts_if);

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            ESP_LOGI(TAG, "gatts_if %d", gatts_if);
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d\n", param->reg.app_id, param->reg.status);
            return;
        }
    }

    do
    {
        int idx;
        for (idx = 0; idx < SPP_PROFILE_NUM; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == spp_profile_tab[idx].gatts_if)
            {
                if (spp_profile_tab[idx].gatts_cb)
                {
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

// static void print_write_buffer(void)
// {
//     temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

//     while (temp_spp_recv_data_node_p1 != NULL)
//     {
//         uart_write_bytes(UART_NUM_0, (char *)(temp_spp_recv_data_node_p1->node_buff), temp_spp_recv_data_node_p1->len);
//         temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p1->next_node;
//     }
// }

// static bool store_wr_buffer(esp_ble_gatts_cb_param_t *p_data)
// {
//     temp_spp_recv_data_node_p1 = (spp_receive_data_node_t *)malloc(sizeof(spp_receive_data_node_t));

//     if (temp_spp_recv_data_node_p1 == NULL)
//     {
//         ESP_LOGI(GATTS_TABLE_TAG, "malloc error %s %d\n", __func__, __LINE__);
//         return false;
//     }
//     if (temp_spp_recv_data_node_p2 != NULL)
//     {
//         temp_spp_recv_data_node_p2->next_node = temp_spp_recv_data_node_p1;
//     }
//     temp_spp_recv_data_node_p1->len = p_data->write.len;
//     SppRecvDataBuff.buff_size += p_data->write.len;
//     temp_spp_recv_data_node_p1->next_node = NULL;
//     temp_spp_recv_data_node_p1->node_buff = (uint8_t *)malloc(p_data->write.len);
//     temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1;
//     memcpy(temp_spp_recv_data_node_p1->node_buff, p_data->write.value, p_data->write.len);
//     if (SppRecvDataBuff.node_num == 0)
//     {
//         SppRecvDataBuff.first_node = temp_spp_recv_data_node_p1;
//         SppRecvDataBuff.node_num++;
//     }
//     else
//     {
//         SppRecvDataBuff.node_num++;
//     }

//     return true;
// }

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;

    for (int i = 0; i < BLE_IDX_NB; i++)
    {
        if (handle == spp_handle_table[i])
        {
            return i;
        }
    }

    return error;
}

// static void free_write_buffer(void)
// {
//     temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

//     while (temp_spp_recv_data_node_p1 != NULL)
//     {
//         temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1->next_node;
//         free(temp_spp_recv_data_node_p1->node_buff);
//         free(temp_spp_recv_data_node_p1);
//         temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p2;
//     }

//     SppRecvDataBuff.node_num = 0;
//     SppRecvDataBuff.buff_size = 0;
//     SppRecvDataBuff.first_node = NULL;
// }

TASK ble_heartbeat()
{
    uint16_t cmd_id;

    for (;;)
    {
        vTaskDelay(BLE_HEARTBEAT_PERIOD_TICKS);
        // ESP_LOGI(TAG, "Char properties %hhx", char_prop_heartbeat);
        // ESP_LOGI(TAG, "Char settings %hx", spp_gatt_db[BLE_IDX_HEARTBEAT_VAL].att_desc.perm);
        // ESP_LOGI(TAG, "Address: %p", &char_prop_heartbeat);
        // ESP_LOGI(TAG, "Address in table: %p", spp_gatt_db[BLE_IDX_HEARTBEAT_CHAR].att_desc.value);
        if (is_connected)
        {
            heartbeat_count_num++;
            if (heartbeat_count_num > 5)
            {
                ESP_LOGW(TAG, "Heartbeat timeout");
                esp_ble_gap_disconnect(spp_remote_bda);
            }
        }
    }
    vTaskDelete(NULL);
}

// void ble_handle_task_notification(uint32_t task_notification_value)
// {
//     /**
//      * @brief Iterate through the task notification value,
//      * one byte at the time,
//      * to send all of the packets which need to be sent.
//      * We start from the LSB (highest priority).
//      * We stop iterating when all requested packets are sent.
//      */
//     uint32_t bitmask = 1;
//     while (task_notification_value)
//     {
//         uint32_t task_flag = task_notification_value & bitmask;
//         if (task_flag)
//         {
//             uint32_t task_id = ble_get_task_id_from_flag(task_flag);
//             ble_handle_tx_send(task_id);
//         }
//         // clear bit of task that we just sent
//         task_notification_value &= ~bitmask;

//         // shift mask to the next index
//         bitmask <<= 1;
//     }
// }

/**
 * @brief Prepare packet to send via BLE and send it using
 * ble_send_data_notification function.
 *
 * Multi-byte data is send using system endianness (Xtensa LX6 is little-endian).
 *
 * Packet structure:
 * +--------------+---------------------------------------------------+----------+
 * | Byte number  |                      Content                      |   Type   |
 * +--------------+---------------------------------------------------+----------+
 * | 0            | Task ID - ble_task_id_type enum value     | uint8_t  |
 * | 1 to 4       | Timestamp - number of FreeRTOS ticks from startup | uint32_t |
 * | 5            | Number of data bytes - n                          | uint8_t  |
 * | 6 to (n + 6) | Data bytes                                        | varying  |
 * | (n + 7)      | Checksum                                          | uint8_t  |
 * +--------------+---------------------------------------------------+----------+
 * @param task_flag
 */
void ble_handle_tx_send(ble_notify_tx_type item)
{
    uint32_t task_id = item.source;
    uint8_t data_size = app_manager_task_data_size[task_id];
    uint8_t packet_length = 1 + 4 + 1 + data_size + 1;

    uint8_t *packet = malloc(packet_length);
    if (packet == NULL)
    {
        ESP_LOGE(TAG, "ble_handle_tx_send malloc");
        abort();
    }
    packet[0] = task_id;

    uint32_t tick_count = xTaskGetTickCount();
    memcpy(packet + 1, &tick_count, sizeof(tick_count));

    packet[5] = data_size;

    memcpy(packet + 6, item.data, data_size);
    // ble_receive_data(task_id, packet + 6);

    /**
     * @todo Implement checksum - hardcoded value for now
     */
    packet[packet_length - 1] = 255;

    ble_send_data_notification(packet, packet_length);
    free(item.data);
    free(packet);
}

// #define LT(n) n, n, n, n, n, n, n, n, n, n, n, n, n, n, n, n
// static const char LogTable256[256] =
//     {
//         -1, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3,
//         LT(4), LT(5), LT(5), LT(6), LT(6), LT(6), LT(6),
//         LT(7), LT(7), LT(7), LT(7), LT(7), LT(7), LT(7), LT(7)};

// uint32_t ble_get_task_id_from_flag(uint32_t task_flag)
// {
//     /**
//      * @brief Implemented using finding log base 2 - most future proof.
//      * http://graphics.stanford.edu/~seander/bithacks.html
//      */

//     unsigned int v = task_flag;  // 32-bit word to find the log of
//     unsigned r;                  // r will be lg(v)
//     register unsigned int t, tt; // temporaries

//     if ((tt = v >> 16))
//     {
//         r = (t = tt >> 8) ? 24 + LogTable256[t] : 16 + LogTable256[tt];
//     }
//     else
//     {
//         r = (t = v >> 8) ? 8 + LogTable256[t] : LogTable256[v];
//     }

//     return r;
// }

// void ble_receive_data(uint8_t id, uint8_t *destination)
// {
//     QueueHandle_t queue;
//     switch (id)
//     {
//     case TASK_ID_HC_SR04:
//         queue = hc_sr04_queue_data;
//         break;
//     case TASK_ID_MPU9255:
//         queue = mpu9255_queue_quaternion_data;
//         break;
//     case TASK_ID_ALGO:
//         queue = algo_ble_data_queue;
//         break;
//     default:
//         abort();
//     }
//     xQueuePeek(queue, destination, 0);
// }

esp_err_t ble_send_data_notification(uint8_t *packet, uint8_t packet_length)
{
    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[BLE_IDX_DATA_NTY_VAL], packet_length, packet, false); // change sizes[i] and string[i]
    return ESP_OK;
}

bool ble_is_connected()
{
    return is_connected;
}

void ble_handle_indication_packet(esp_ble_gatts_cb_param_t *p_data)
{
    struct gatts_write_evt_param received_event = p_data->write;
    if (received_event.len != 2)
    {
        ESP_LOGE(TAG, "Received controller packet with length %d", received_event.len);
        return;
    }

    if (received_event.value[0] != BLE_CONTROLLER_PACKET_HEADER)
    {
        ESP_LOGE(TAG, "Received controller packet with header %d", received_event.value[0]);
        return;
    }

    app_manager_event_type event_to_send;
    event_to_send.source = TASK_ID_BLE;
    switch (received_event.value[1])
    {
    case REQUEST_START_DRIVING:
        event_to_send.type = EVENT_REQUEST_START;
        break;
    case REQUEST_STOP_DRIVING:
        event_to_send.type = EVENT_REQUEST_STOP;
        break;
    default:
        ESP_LOGE(TAG, "Unsupported request, value %x", received_event.value[1]);
        break;
    }

    BaseType_t retval = xQueueSend(app_manager_event_queue, &event_to_send, pdMS_TO_TICKS(100));
    if (retval != pdPASS)
    {
        ESP_LOGE(TAG, "App manager event queue full");
    }

    /**
     * @todo Implement response to controller application here.
     */
}

void ble_send_from_task(ble_task_id_type source, void *data)
{
    if (!is_connected)
    {
        return;
    }

    ble_notify_tx_type item;
    item.source = source;
    uint8_t data_size = app_manager_task_data_size[source];

    item.data = malloc(data_size);
    if (item.data == NULL)
    {
        abort();
    }
    memcpy(item.data, data, data_size);
    xQueueSend(ble_notify_tx_queue, &item, BLE_NOTIFY_TX_TIMEOUT);
}
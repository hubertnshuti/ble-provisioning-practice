#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include <assert.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "driver/gpio.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "os/os_mbuf.h"

#define TAG "DARHO_BRIDGE"

#define CONFIG_VERSION          1

#define MAX_WIFI_SSID_LEN      32
#define MAX_WIFI_PASS_LEN      64
#define MAX_GATEWAY_LEN        64
#define MAX_CLAIM_CODE_LEN     40
#define MAX_BRIDGE_NAME_LEN    40
#define MAX_SITE_CODE_LEN      16

#define STORAGE_NAMESPACE      "bridge_cfg"

#define SETUP_BUTTON_GPIO      GPIO_NUM_4
#define RESET_HOLD_TIME_MS     5000

#define PROVISIONING_WINDOW_MS (5 * 60 * 1000)

#define BRIDGE_SERVICE_UUID \
    BLE_UUID128_DECLARE(0x10, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe, \
                        0x00, 0x00, 0x00, 0x00, 0x00, 0xab, 0xcd, 0xef)

#define WIFI_SSID_UUID \
    BLE_UUID128_DECLARE(0x11, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe, \
                        0x00, 0x00, 0x00, 0x00, 0x00, 0xab, 0xcd, 0xef)

#define WIFI_PASS_UUID \
    BLE_UUID128_DECLARE(0x12, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe, \
                        0x00, 0x00, 0x00, 0x00, 0x00, 0xab, 0xcd, 0xef)

#define GATEWAY_ADDR_UUID \
    BLE_UUID128_DECLARE(0x13, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe, \
                        0x00, 0x00, 0x00, 0x00, 0x00, 0xab, 0xcd, 0xef)

#define CLAIM_CODE_UUID \
    BLE_UUID128_DECLARE(0x14, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe, \
                        0x00, 0x00, 0x00, 0x00, 0x00, 0xab, 0xcd, 0xef)

#define BRIDGE_NAME_UUID \
    BLE_UUID128_DECLARE(0x15, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe, \
                        0x00, 0x00, 0x00, 0x00, 0x00, 0xab, 0xcd, 0xef)

#define SITE_CODE_UUID \
    BLE_UUID128_DECLARE(0x16, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe, \
                        0x00, 0x00, 0x00, 0x00, 0x00, 0xab, 0xcd, 0xef)

#define COMMAND_UUID \
    BLE_UUID128_DECLARE(0x17, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe, \
                        0x00, 0x00, 0x00, 0x00, 0x00, 0xab, 0xcd, 0xef)

#define STATUS_UUID \
    BLE_UUID128_DECLARE(0x18, 0x32, 0x54, 0x76, 0x98, 0xba, 0xdc, 0xfe, \
                        0x00, 0x00, 0x00, 0x00, 0x00, 0xab, 0xcd, 0xef)

typedef enum {
    BRIDGE_STATE_FACTORY_NEW = 0,
    BRIDGE_STATE_BLE_PROVISIONING,
    BRIDGE_STATE_CONFIGURATION_RECEIVED,
    BRIDGE_STATE_CONFIGURATION_VALIDATED,
    BRIDGE_STATE_CONFIGURATION_SAVED,
    BRIDGE_STATE_PROVISIONED,
    BRIDGE_STATE_NORMAL_OPERATION,
    BRIDGE_STATE_PROVISIONING_FAILED,
    BRIDGE_STATE_RECOVERY_MODE
} bridge_state_t;

typedef enum {
    VALIDATION_OK = 0,
    ERR_MISSING_WIFI_SSID,
    ERR_INVALID_WIFI_SSID,
    ERR_MISSING_WIFI_PASSWORD,
    ERR_INVALID_WIFI_PASSWORD,
    ERR_MISSING_GATEWAY_ADDRESS,
    ERR_INVALID_GATEWAY_ADDRESS,
    ERR_MISSING_CLAIM_CODE,
    ERR_INVALID_CLAIM_CODE,
    ERR_MISSING_BRIDGE_NAME,
    ERR_INVALID_BRIDGE_NAME,
    ERR_UNSUPPORTED_CONFIG_VERSION,
    ERR_NOT_PROVISIONED,
    ERR_SAVE_FAILED
} validation_result_t;

typedef enum {
    CHAR_WIFI_SSID = 1,
    CHAR_WIFI_PASS,
    CHAR_GATEWAY_ADDR,
    CHAR_CLAIM_CODE,
    CHAR_BRIDGE_NAME,
    CHAR_SITE_CODE,
    CHAR_COMMAND,
    CHAR_STATUS
} characteristic_id_t;

typedef struct {
    uint32_t config_version;
    bool provisioned;

    char wifi_ssid[MAX_WIFI_SSID_LEN + 1];
    char wifi_password[MAX_WIFI_PASS_LEN + 1];

    char gateway_address[MAX_GATEWAY_LEN + 1];
    char claim_code[MAX_CLAIM_CODE_LEN + 1];
    char bridge_name[MAX_BRIDGE_NAME_LEN + 1];
    char site_code[MAX_SITE_CODE_LEN + 1];
} bridge_config_t;

static bridge_state_t g_bridge_state = BRIDGE_STATE_FACTORY_NEW;

static bridge_config_t g_temp_config;
static bridge_config_t g_active_config;

static char g_status_text[128] = "booting";

static uint8_t g_own_addr_type;
static uint16_t g_status_val_handle;

static const char *validation_to_text(validation_result_t result)
{
    switch (result) {
        case VALIDATION_OK: return "ok";
        case ERR_MISSING_WIFI_SSID: return "missing_wifi_ssid";
        case ERR_INVALID_WIFI_SSID: return "invalid_wifi_ssid";
        case ERR_MISSING_WIFI_PASSWORD: return "missing_wifi_password";
        case ERR_INVALID_WIFI_PASSWORD: return "invalid_wifi_password";
        case ERR_MISSING_GATEWAY_ADDRESS: return "missing_gateway_address";
        case ERR_INVALID_GATEWAY_ADDRESS: return "invalid_gateway_address";
        case ERR_MISSING_CLAIM_CODE: return "missing_claim_code";
        case ERR_INVALID_CLAIM_CODE: return "invalid_claim_code";
        case ERR_MISSING_BRIDGE_NAME: return "missing_bridge_name";
        case ERR_INVALID_BRIDGE_NAME: return "invalid_bridge_name";
        case ERR_UNSUPPORTED_CONFIG_VERSION: return "unsupported_config_version";
        case ERR_NOT_PROVISIONED: return "not_provisioned";
        case ERR_SAVE_FAILED: return "save_failed";
        default: return "unknown_error";
    }
}

static void set_status(const char *status)
{
    if (status == NULL) {
        status = "unknown_status";
    }

    size_t len = strlen(status);

    if (len >= sizeof(g_status_text)) {
        len = sizeof(g_status_text) - 1;
    }

    memcpy(g_status_text, status, len);
    g_status_text[len] = ' ';

    ESP_LOGI(TAG, "STATUS: %s", g_status_text);
}

static bool is_empty(const char *text)
{
    return text == NULL || text[0] == '\0';
}

static void clear_temp_config(void)
{
    memset(&g_temp_config, 0, sizeof(g_temp_config));
    g_temp_config.config_version = CONFIG_VERSION;
    g_temp_config.provisioned = false;
}
//h
static void trim_line_end(char *text)
{
    size_t len = strlen(text);

    while (len > 0 && (text[len - 1] == '\n' || text[len - 1] == '\r')) {
        text[len - 1] = '\0';
        len--;
    }
}

static bool copy_ble_value(struct os_mbuf *om, char *dest, size_t dest_size)
{
    int len = OS_MBUF_PKTLEN(om);

    if (len <= 0 || len >= dest_size) {
        return false;
    }

    memset(dest, 0, dest_size);

    if (os_mbuf_copydata(om, 0, len, dest) != 0) {
        return false;
    }

    dest[len] = '\0';
    trim_line_end(dest);

    return true;
}

static bool copy_text_with_limit(char *dest, size_t dest_size, const char *src)
{
    size_t len = strlen(src);

    if (len >= dest_size) {
        return false;
    }

    memcpy(dest, src, len + 1);
    return true;
}

static bool is_valid_ipv4(const char *addr)
{
    int parts = 0;
    const char *p = addr;

    while (*p) {
        if (!isdigit((unsigned char)*p)) {
            return false;
        }

        int value = 0;
        int digits = 0;

        while (*p && isdigit((unsigned char)*p)) {
            value = value * 10 + (*p - '0');
            digits++;

            if (digits > 3 || value > 255) {
                return false;
            }

            p++;
        }

        parts++;

        if (*p == '.') {
            p++;
        } else if (*p == '\0') {
            break;
        } else {
            return false;
        }
    }

    return parts == 4;
}

static bool is_valid_hostname(const char *host)
{
    size_t len = strlen(host);

    if (len < 3 || len > MAX_GATEWAY_LEN) {
        return false;
    }

    if (strstr(host, "://") != NULL) {
        return false;
    }

    if (host[0] == '.' || host[len - 1] == '.') {
        return false;
    }

    bool has_dot = false;

    for (size_t i = 0; i < len; i++) {
        char c = host[i];

        if (c == '.') {
            has_dot = true;
            continue;
        }

        if (!(isalnum((unsigned char)c) || c == '-')) {
            return false;
        }
    }

    return has_dot;
}

static bool is_valid_gateway_address(const char *gateway)
{
    if (is_empty(gateway)) {
        return false;
    }

    if (is_valid_ipv4(gateway)) {
        return true;
    }

    if (is_valid_hostname(gateway)) {
        return true;
    }

    return false;
}

static bool is_valid_claim_code(const char *claim)
{
    size_t len = strlen(claim);

    if (len < 12 || len > MAX_CLAIM_CODE_LEN) {
        return false;
    }

    if (strncmp(claim, "DARHO-", 6) != 0) {
        return false;
    }

    int dash_count = 0;

    for (size_t i = 0; i < len; i++) {
        char c = claim[i];

        if (c == '-') {
            dash_count++;
            continue;
        }

        if (!(isdigit((unsigned char)c) || (c >= 'A' && c <= 'Z'))) {
            return false;
        }
    }

    return dash_count >= 3;
}

static bool is_valid_bridge_name(const char *name)
{
    size_t len = strlen(name);

    if (len < 3 || len > MAX_BRIDGE_NAME_LEN) {
        return false;
    }

    for (size_t i = 0; i < len; i++) {
        char c = name[i];

        if (!(isalnum((unsigned char)c) || c == ' ' || c == '-' || c == '_')) {
            return false;
        }
    }

    return true;
}

static validation_result_t validate_config(const bridge_config_t *cfg, bool require_provisioned)
{
    if (cfg->config_version != CONFIG_VERSION) {
        return ERR_UNSUPPORTED_CONFIG_VERSION;
    }

    if (is_empty(cfg->wifi_ssid)) {
        return ERR_MISSING_WIFI_SSID;
    }

    if (strlen(cfg->wifi_ssid) > MAX_WIFI_SSID_LEN) {
        return ERR_INVALID_WIFI_SSID;
    }

    if (is_empty(cfg->wifi_password)) {
        return ERR_MISSING_WIFI_PASSWORD;
    }

    size_t pass_len = strlen(cfg->wifi_password);

    if (pass_len < 8 || pass_len > 63) {
        return ERR_INVALID_WIFI_PASSWORD;
    }

    if (is_empty(cfg->gateway_address)) {
        return ERR_MISSING_GATEWAY_ADDRESS;
    }

    if (!is_valid_gateway_address(cfg->gateway_address)) {
        return ERR_INVALID_GATEWAY_ADDRESS;
    }

    if (is_empty(cfg->claim_code)) {
        return ERR_MISSING_CLAIM_CODE;
    }

    if (!is_valid_claim_code(cfg->claim_code)) {
        return ERR_INVALID_CLAIM_CODE;
    }

    if (is_empty(cfg->bridge_name)) {
        return ERR_MISSING_BRIDGE_NAME;
    }

    if (!is_valid_bridge_name(cfg->bridge_name)) {
        return ERR_INVALID_BRIDGE_NAME;
    }

    if (require_provisioned && !cfg->provisioned) {
        return ERR_NOT_PROVISIONED;
    }

    return VALIDATION_OK;
}

static esp_err_t storage_save_valid_config(const bridge_config_t *cfg)
{
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_u8(handle, "provisioned", 0);
    if (err != ESP_OK) goto done;

    err = nvs_set_u32(handle, "version", CONFIG_VERSION);
    if (err != ESP_OK) goto done;

    err = nvs_set_str(handle, "ssid", cfg->wifi_ssid);
    if (err != ESP_OK) goto done;

    err = nvs_set_str(handle, "password", cfg->wifi_password);
    if (err != ESP_OK) goto done;

    err = nvs_set_str(handle, "gateway", cfg->gateway_address);
    if (err != ESP_OK) goto done;

    err = nvs_set_str(handle, "claim", cfg->claim_code);
    if (err != ESP_OK) goto done;

    err = nvs_set_str(handle, "name", cfg->bridge_name);
    if (err != ESP_OK) goto done;

    err = nvs_set_str(handle, "site", cfg->site_code);
    if (err != ESP_OK) goto done;

    err = nvs_commit(handle);
    if (err != ESP_OK) goto done;

    err = nvs_set_u8(handle, "provisioned", 1);
    if (err != ESP_OK) goto done;

    err = nvs_commit(handle);

done:
    nvs_close(handle);
    return err;
}

static esp_err_t storage_load_config(bridge_config_t *cfg)
{
    nvs_handle_t handle;
    esp_err_t err;
    uint8_t provisioned = 0;

    memset(cfg, 0, sizeof(*cfg));

    err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return err;
    }

    size_t ssid_len = sizeof(cfg->wifi_ssid);
    size_t pass_len = sizeof(cfg->wifi_password);
    size_t gateway_len = sizeof(cfg->gateway_address);
    size_t claim_len = sizeof(cfg->claim_code);
    size_t name_len = sizeof(cfg->bridge_name);
    size_t site_len = sizeof(cfg->site_code);

    err = nvs_get_u32(handle, "version", &cfg->config_version);
    if (err != ESP_OK) goto done;

    err = nvs_get_u8(handle, "provisioned", &provisioned);
    if (err != ESP_OK) goto done;

    err = nvs_get_str(handle, "ssid", cfg->wifi_ssid, &ssid_len);
    if (err != ESP_OK) goto done;

    err = nvs_get_str(handle, "password", cfg->wifi_password, &pass_len);
    if (err != ESP_OK) goto done;

    err = nvs_get_str(handle, "gateway", cfg->gateway_address, &gateway_len);
    if (err != ESP_OK) goto done;

    err = nvs_get_str(handle, "claim", cfg->claim_code, &claim_len);
    if (err != ESP_OK) goto done;

    err = nvs_get_str(handle, "name", cfg->bridge_name, &name_len);
    if (err != ESP_OK) goto done;

    err = nvs_get_str(handle, "site", cfg->site_code, &site_len);
    if (err != ESP_OK) goto done;

    cfg->provisioned = provisioned ? true : false;

done:
    nvs_close(handle);
    return err;
}

static esp_err_t storage_factory_reset(void)
{
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &handle);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }

    if (err != ESP_OK) {
        return err;
    }

    err = nvs_erase_all(handle);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }

    nvs_close(handle);

    return err;
}

static bool provisioning_allowed(void)
{
    return g_bridge_state == BRIDGE_STATE_FACTORY_NEW ||
           g_bridge_state == BRIDGE_STATE_BLE_PROVISIONING ||
           g_bridge_state == BRIDGE_STATE_CONFIGURATION_RECEIVED ||
           g_bridge_state == BRIDGE_STATE_PROVISIONING_FAILED ||
           g_bridge_state == BRIDGE_STATE_RECOVERY_MODE;
}

static void start_normal_operation(void)
{
    g_bridge_state = BRIDGE_STATE_NORMAL_OPERATION;

    set_status("normal_operation");

    ESP_LOGI(TAG, "Bridge is now in normal operation");
    ESP_LOGI(TAG, "Bridge name: %s", g_active_config.bridge_name);
    ESP_LOGI(TAG, "Gateway address: %s", g_active_config.gateway_address);
    ESP_LOGI(TAG, "Wi-Fi SSID: %s", g_active_config.wifi_ssid);

}

static void enter_ble_provisioning_mode(void)
{
    g_bridge_state = BRIDGE_STATE_BLE_PROVISIONING;
    set_status("waiting_for_configuration");
}

static void bridge_boot_decision(void)
{
    bridge_config_t saved_config;
    esp_err_t err = storage_load_config(&saved_config);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved configuration found");
        clear_temp_config();

        g_bridge_state = BRIDGE_STATE_FACTORY_NEW;
        set_status("factory_new");

        enter_ble_provisioning_mode();
        return;
    }

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not load configuration: %s", esp_err_to_name(err));

        g_bridge_state = BRIDGE_STATE_RECOVERY_MODE;
        set_status("saved_configuration_load_failed");

        enter_ble_provisioning_mode();
        return;
    }

    validation_result_t result = validate_config(&saved_config, true);

    if (result != VALIDATION_OK) {
        ESP_LOGW(TAG, "Saved configuration invalid: %s", validation_to_text(result));

        g_bridge_state = BRIDGE_STATE_RECOVERY_MODE;
        set_status(validation_to_text(result));

        enter_ble_provisioning_mode();
        return;
    }

    memcpy(&g_active_config, &saved_config, sizeof(g_active_config));

    ESP_LOGI(TAG, "Saved configuration is valid");
    ESP_LOGI(TAG, "Boot decision: start normal bridge operation");

    start_normal_operation();
}

static void setup_button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SETUP_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_conf);
}

static bool setup_button_is_held_for_reset(void)
{
    int elapsed = 0;

    while (elapsed < RESET_HOLD_TIME_MS) {
        if (gpio_get_level(SETUP_BUTTON_GPIO) != 0) {
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
        elapsed += 100;
    }

    return true;
}

static void restart_task(void *arg)
{
    int delay_ms = (int)(intptr_t)arg;

    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    esp_restart();
}

static void perform_factory_reset_and_restart(void)
{
    ESP_LOGW(TAG, "Factory reset requested");

    set_status("factory_reset_running");

    esp_err_t err = storage_factory_reset();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Factory reset failed: %s", esp_err_to_name(err));
        set_status("factory_reset_failed");
        return;
    }

    clear_temp_config();
    memset(&g_active_config, 0, sizeof(g_active_config));

    g_bridge_state = BRIDGE_STATE_FACTORY_NEW;

    set_status("factory_reset_complete");

    xTaskCreate(restart_task, "restart_task", 2048, (void *)(intptr_t)1000, 5, NULL);
}

static void reset_button_monitor_task(void *arg)
{
    while (1) {
        if (gpio_get_level(SETUP_BUTTON_GPIO) == 0) {
            ESP_LOGW(TAG, "Setup button pressed. Hold for reset...");

            if (setup_button_is_held_for_reset()) {
                perform_factory_reset_and_restart();
                vTaskDelete(NULL);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void handle_save_command(void)
{
    set_status("validating_configuration");

    g_temp_config.config_version = CONFIG_VERSION;
    g_temp_config.provisioned = false;

    validation_result_t result = validate_config(&g_temp_config, false);

    if (result != VALIDATION_OK) {
        g_bridge_state = BRIDGE_STATE_PROVISIONING_FAILED;
        set_status(validation_to_text(result));

        ESP_LOGW(TAG, "Provisioning validation failed: %s", validation_to_text(result));
        return;
    }

    g_bridge_state = BRIDGE_STATE_CONFIGURATION_VALIDATED;
    set_status("configuration_validated");

    ESP_LOGI(TAG, "Configuration validated successfully");

    esp_err_t err = storage_save_valid_config(&g_temp_config);

    if (err != ESP_OK) {
        g_bridge_state = BRIDGE_STATE_PROVISIONING_FAILED;
        set_status("save_failed");

        ESP_LOGE(TAG, "Configuration save failed: %s", esp_err_to_name(err));
        return;
    }

    g_bridge_state = BRIDGE_STATE_CONFIGURATION_SAVED;
    set_status("configuration_saved");

    memcpy(&g_active_config, &g_temp_config, sizeof(g_active_config));
    g_active_config.provisioned = true;

    g_bridge_state = BRIDGE_STATE_PROVISIONED;
    set_status("provisioning_success_restarting");

    ESP_LOGI(TAG, "Provisioning complete");
    ESP_LOGI(TAG, "Bridge will restart and boot from saved configuration");

    xTaskCreate(restart_task, "restart_after_provisioning", 2048, (void *)(intptr_t)1500, 5, NULL);
}

static void handle_command(const char *command)
{
    if (strcmp(command, "save") == 0) {
        handle_save_command();
        return;
    }

    if (strcmp(command, "cancel") == 0) {
        clear_temp_config();
        g_bridge_state = BRIDGE_STATE_BLE_PROVISIONING;
        set_status("provisioning_cancelled");
        return;
    }

    if (strcmp(command, "factory_reset") == 0) {
        if (!provisioning_allowed()) {
            set_status("reset_not_allowed_in_normal_operation");
            return;
        }

        perform_factory_reset_and_restart();
        return;
    }

    set_status("unknown_command");
}

static int gatt_access_cb(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg
)
{
    characteristic_id_t char_id = (characteristic_id_t)(intptr_t)arg;

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        if (char_id == CHAR_STATUS) {
            os_mbuf_append(ctxt->om, g_status_text, strlen(g_status_text));
            return 0;
        }

        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (!provisioning_allowed()) {
        set_status("provisioning_locked");
        return 0;
    }

    char value[96];

    if (!copy_ble_value(ctxt->om, value, sizeof(value))) {
        set_status("invalid_write_length");
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    g_bridge_state = BRIDGE_STATE_CONFIGURATION_RECEIVED;

    switch (char_id) {
        case CHAR_WIFI_SSID:
            if (!copy_text_with_limit(g_temp_config.wifi_ssid, sizeof(g_temp_config.wifi_ssid), value)) {
                set_status("wifi_ssid_too_long");
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            set_status("wifi_ssid_received");
            break;

        case CHAR_WIFI_PASS:
            if (!copy_text_with_limit(g_temp_config.wifi_password, sizeof(g_temp_config.wifi_password), value)) {
                set_status("wifi_password_too_long");
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            set_status("wifi_password_received");
            break;

        case CHAR_GATEWAY_ADDR:
            if (!copy_text_with_limit(g_temp_config.gateway_address, sizeof(g_temp_config.gateway_address), value)) {
                set_status("gateway_address_too_long");
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            set_status("gateway_address_received");
            break;

        case CHAR_CLAIM_CODE:
            if (!copy_text_with_limit(g_temp_config.claim_code, sizeof(g_temp_config.claim_code), value)) {
                set_status("claim_code_too_long");
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            set_status("claim_code_received");
            break;

        case CHAR_BRIDGE_NAME:
            if (!copy_text_with_limit(g_temp_config.bridge_name, sizeof(g_temp_config.bridge_name), value)) {
                set_status("bridge_name_too_long");
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            set_status("bridge_name_received");
            break;

        case CHAR_SITE_CODE:
            if (!copy_text_with_limit(g_temp_config.site_code, sizeof(g_temp_config.site_code), value)) {
                set_status("site_code_too_long");
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            set_status("site_code_received");
            break;

        case CHAR_COMMAND:
            handle_command(value);
            break;

        default:
            set_status("unknown_characteristic");
            break;
    }

    return 0;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BRIDGE_SERVICE_UUID,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = WIFI_SSID_UUID,
                .access_cb = gatt_access_cb,
                .arg = (void *)(intptr_t)CHAR_WIFI_SSID,
                .flags = BLE_GATT_CHR_F_WRITE
            },
            {
                .uuid = WIFI_PASS_UUID,
                .access_cb = gatt_access_cb,
                .arg = (void *)(intptr_t)CHAR_WIFI_PASS,
                .flags = BLE_GATT_CHR_F_WRITE
            },
            {
                .uuid = GATEWAY_ADDR_UUID,
                .access_cb = gatt_access_cb,
                .arg = (void *)(intptr_t)CHAR_GATEWAY_ADDR,
                .flags = BLE_GATT_CHR_F_WRITE
            },
            {
                .uuid = CLAIM_CODE_UUID,
                .access_cb = gatt_access_cb,
                .arg = (void *)(intptr_t)CHAR_CLAIM_CODE,
                .flags = BLE_GATT_CHR_F_WRITE
            },
            {
                .uuid = BRIDGE_NAME_UUID,
                .access_cb = gatt_access_cb,
                .arg = (void *)(intptr_t)CHAR_BRIDGE_NAME,
                .flags = BLE_GATT_CHR_F_WRITE
            },
            {
                .uuid = SITE_CODE_UUID,
                .access_cb = gatt_access_cb,
                .arg = (void *)(intptr_t)CHAR_SITE_CODE,
                .flags = BLE_GATT_CHR_F_WRITE
            },
            {
                .uuid = COMMAND_UUID,
                .access_cb = gatt_access_cb,
                .arg = (void *)(intptr_t)CHAR_COMMAND,
                .flags = BLE_GATT_CHR_F_WRITE
            },
            {
                .uuid = STATUS_UUID,
                .access_cb = gatt_access_cb,
                .arg = (void *)(intptr_t)CHAR_STATUS,
                .val_handle = &g_status_val_handle,
                .flags = BLE_GATT_CHR_F_READ
            },
            {
                0
            }
        }
    },
    {
        0
    }
};

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg);

static void ble_start_advertising(void)
{
    if (!provisioning_allowed()) {
        ESP_LOGI(TAG, "Provisioning is locked. BLE setup advertising not started.");
        return;
    }

    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    const char *device_name = "ESP32-BRIDGE-SETUP";

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);

    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertising fields: %d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(
        g_own_addr_type,
        NULL,
        BLE_HS_FOREVER,
        &adv_params,
        ble_gap_event_cb,
        NULL
    );

    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "BLE provisioning advertising started");
}

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(TAG, "BLE client connected");
                set_status("setup_tool_connected");
            } else {
                ESP_LOGW(TAG, "BLE connection failed");
                ble_start_advertising();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "BLE client disconnected");

            if (provisioning_allowed()) {
                set_status("waiting_for_configuration");
                ble_start_advertising();
            }
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "Advertising complete");

            if (provisioning_allowed()) {
                ble_start_advertising();
            }
            break;

        default:
            break;
    }

    return 0;
}

static void ble_on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
    assert(rc == 0);

    ble_start_advertising();
}

static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_provisioning_init(void)
{
    int rc;

    ESP_ERROR_CHECK(nimble_port_init());

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_svc_gap_device_name_set("ESP32-BRIDGE-SETUP");

    rc = ble_gatts_count_cfg(gatt_svcs);
    assert(rc == 0);

    rc = ble_gatts_add_svcs(gatt_svcs);
    assert(rc == 0);

    ble_hs_cfg.sync_cb = ble_on_sync;

    nimble_port_freertos_init(ble_host_task);
}

static void provisioning_window_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(PROVISIONING_WINDOW_MS));

    if (g_bridge_state == BRIDGE_STATE_BLE_PROVISIONING ||
        g_bridge_state == BRIDGE_STATE_CONFIGURATION_RECEIVED ||
        g_bridge_state == BRIDGE_STATE_PROVISIONING_FAILED) {

        set_status("provisioning_window_expired");

        ble_gap_adv_stop();

        ESP_LOGW(TAG, "Provisioning window expired. Advertising stopped.");
    }

    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "DARHO ESP32 Bridge - Lesson 6 and Lesson 7");

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(ret);
    }

    setup_button_init();

    clear_temp_config();

    xTaskCreate(reset_button_monitor_task, "reset_button_monitor", 3072, NULL, 5, NULL);

    bridge_boot_decision();

    if (provisioning_allowed()) {
        ble_provisioning_init();
        xTaskCreate(provisioning_window_task, "provisioning_window", 2048, NULL, 5, NULL);
    }
}
/*
 * ESP-Mesh-Lite "no_router" + UART<->UDP bridge
 *
 * LEAF:
 *   UART -> UDP -> ROOT
 *
 * ROOT:
 *   UDP -> UART
 *
 * NO ROUTER / NO INTERNET
 *
 * Key behavior:
 * - We do NOT force AP-only on ROOT (that broke Mesh-Lite and caused your crash).
 * - We DO force Mesh-Lite into MESH networking mode and clear router config so it stops trying
 *   to connect to an upstream AP/router.
 * - We print the resulting networking mode + error codes so there's no guessing.
 */

#include <inttypes.h>
#include <string.h>
#include <ctype.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_mac.h"

#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

#include "driver/uart.h"

#include "esp_bridge.h"
#include "esp_mesh_lite.h"
#include "esp_mesh_lite_core.h"
#include "esp_mesh_lite_port.h"

static const char *TAG = "no_router_uart_udp";

/* ============================================================
 * SYSTEM INFO (runs in normal task, NOT timer task)
 * ============================================================ */

static TaskHandle_t sysinfo_task_h = NULL;

static void sysinfo_task(void *arg)
{
    (void)arg;

    uint8_t primary = 0;
    uint8_t sta_mac[6] = {0};
    wifi_ap_record_t ap_info = {0};
    wifi_second_chan_t second = 0;
    wifi_sta_list_t wifi_sta_list = {0};

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (esp_mesh_lite_get_level() > 1) {
            (void)esp_wifi_sta_get_ap_info(&ap_info);
        } else {
            memset(&ap_info, 0, sizeof(ap_info));
        }

        (void)esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
        (void)esp_wifi_ap_get_sta_list(&wifi_sta_list);
        (void)esp_wifi_get_channel(&primary, &second);

        ESP_LOGI(TAG,
                 "System info, ch=%d layer=%d self=" MACSTR
                 " parent=" MACSTR " parent_rssi=%d free_heap=%" PRIu32,
                 primary,
                 esp_mesh_lite_get_level(),
                 MAC2STR(sta_mac),
                 MAC2STR(ap_info.bssid),
                 (ap_info.rssi != 0 ? ap_info.rssi : -120),
                 esp_get_free_heap_size());

        for (int i = 0; i < wifi_sta_list.num; i++) {
            ESP_LOGI(TAG, "Child mac: " MACSTR,
                     MAC2STR(wifi_sta_list.sta[i].mac));
        }
    }
}

static void sysinfo_timer_cb(TimerHandle_t t)
{
    (void)t;
    if (sysinfo_task_h) {
        xTaskNotifyGive(sysinfo_task_h);
    }
}

/* ============================================================
 * NVS
 * ============================================================ */

static esp_err_t esp_storage_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

/* ============================================================
 * WIFI / BRIDGE
 * ============================================================ */

static void wifi_init(void)
{
#if !CONFIG_MESH_ROOT
    wifi_config_t sta_cfg = {0};
    esp_bridge_wifi_set_config(WIFI_IF_STA, &sta_cfg);
#endif

    wifi_config_t ap_cfg = {
        .ap = {
            .ssid = CONFIG_BRIDGE_SOFTAP_SSID,
            .password = CONFIG_BRIDGE_SOFTAP_PASSWORD,
            .channel = CONFIG_MESH_CHANNEL,
        },
    };

    esp_bridge_wifi_set_config(WIFI_IF_AP, &ap_cfg);
}

static void app_wifi_set_softap_info(void)
{
    char ssid[33] = {0};
    char psk[64] = {0};
    uint8_t mac[6];
    size_t ssid_len = sizeof(ssid);
    size_t psk_len = sizeof(psk);

    esp_wifi_get_mac(WIFI_IF_AP, mac);

    if (esp_mesh_lite_get_softap_ssid_from_nvs(ssid, &ssid_len) != ESP_OK) {
#ifdef CONFIG_BRIDGE_SOFTAP_SSID_END_WITH_THE_MAC
        snprintf(ssid, sizeof(ssid), "%.25s_%02x%02x%02x",
                 CONFIG_BRIDGE_SOFTAP_SSID,
                 mac[3], mac[4], mac[5]);
#else
        strlcpy(ssid, CONFIG_BRIDGE_SOFTAP_SSID, sizeof(ssid));
#endif
    }

    if (esp_mesh_lite_get_softap_psw_from_nvs(psk, &psk_len) != ESP_OK) {
        strlcpy(psk, CONFIG_BRIDGE_SOFTAP_PASSWORD, sizeof(psk));
    }

    ESP_LOGI(TAG, "SoftAP SSID: %s", ssid);
    ESP_LOGI(TAG, "SoftAP PSK: [HIDDEN]");
    ESP_ERROR_CHECK(esp_mesh_lite_set_softap_info(ssid, psk));
}

/* ============================================================
 * UART
 * ============================================================ */

static void uart_init_bridge(void)
{
    uart_config_t cfg = {
        .baud_rate = CONFIG_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 4096, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1,
                                CONFIG_UART_TX_GPIO,
                                CONFIG_UART_RX_GPIO,
                                UART_PIN_NO_CHANGE,
                                UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART ready @ %d baud", CONFIG_UART_BAUD);
}

/* ============================================================
 * UDP ROOT
 * ============================================================ */

#if CONFIG_MESH_ROOT

static void root_udp_to_uart_task(void *arg)
{
    (void)arg;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Root socket() failed");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "Root bind() failed on port %d", CONFIG_UDP_PORT);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Root UDP listening on %d", CONFIG_UDP_PORT);

    uint8_t buf[256];

    for (;;) {
        int n = recvfrom(sock, buf, sizeof(buf), 0, NULL, NULL);
        if (n > 0) {
            uart_write_bytes(UART_NUM_1, (char *)buf, n);
        }
    }
}

#endif

/* ============================================================
 * Mesh-Lite policy for "no_router"
 * ============================================================ */

static void mesh_no_router_policy_apply(void)
{
    esp_err_t err;

    // 1) Force MESH-only networking mode
    err = esp_mesh_lite_set_networking_mode(ESP_MESH_LITE_MESH, 0);
    ESP_LOGI(TAG, "set_networking_mode(MESH) -> %s", esp_err_to_name(err));

    // 2) Clear router/uplink config (prevents any upstream connect target)
    // NOTE: type name comes from esp_mesh_lite_port.h in this release
    mesh_lite_sta_config_t rcfg;
    memset(&rcfg, 0, sizeof(rcfg));
    err = esp_mesh_lite_set_router_config(&rcfg);
    ESP_LOGI(TAG, "set_router_config(empty) -> %s", esp_err_to_name(err));

    // 3) Print the mode so you can verify on the monitor (no guessing)
    esp_mesh_lite_networking_mode_t mode = ESP_MESH_LITE_ROUTER;
    err = esp_mesh_lite_get_networking_mode(&mode);
    ESP_LOGI(TAG, "get_networking_mode -> %s, mode=%s",
             esp_err_to_name(err),
             (mode == ESP_MESH_LITE_MESH) ? "MESH" : "ROUTER");

    // 4) Backoff reconnect attempts (reduces connect-spam/crashes if something still triggers)
    // (min_s, max_s, max_failed_count_or_timeout depending on implementation; safe to call)
    esp_mesh_lite_set_wifi_reconnect_interval(30, 0, 3600);
}

/* ============================================================
 * APP MAIN
 * ============================================================ */

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);

    ESP_ERROR_CHECK(esp_storage_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_bridge_create_all_netif();
    wifi_init();

    esp_mesh_lite_config_t cfg = ESP_MESH_LITE_DEFAULT_INIT();
    cfg.join_mesh_ignore_router_status = true;
    cfg.join_mesh_without_configured_wifi = true;

    // void-return API in this release
    esp_mesh_lite_init(&cfg);

    // Apply "no_router" policy BEFORE start
    mesh_no_router_policy_apply();

    app_wifi_set_softap_info();

#if CONFIG_MESH_ROOT
    ESP_LOGI(TAG, "Role: ROOT");
    esp_mesh_lite_set_allowed_level(1);
#else
    ESP_LOGI(TAG, "Role: LEAF");
    esp_mesh_lite_set_disallowed_level(1);
#endif

    // void-return API
    esp_mesh_lite_start();

    uart_init_bridge();

#if CONFIG_MESH_ROOT
    xTaskCreate(root_udp_to_uart_task,
                "root_udp_to_uart",
                4096, NULL, 10, NULL);
#endif

    xTaskCreate(sysinfo_task,
                "sysinfo",
                4096, NULL, 5, &sysinfo_task_h);

    TimerHandle_t t = xTimerCreate("sysinfo_timer",
                                   pdMS_TO_TICKS(10000),
                                   true, NULL,
                                   sysinfo_timer_cb);
    xTimerStart(t, 0);
}

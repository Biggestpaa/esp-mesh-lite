/*
 * ESP-Mesh-Lite "no_router" + UART<->UDP bridge
 *
 * NODE (non-root):
 *   UART -> (batched) UDP -> ROOT
 *
 * ROOT:
 *   UDP -> UART
 *
 * NO ROUTER / NO INTERNET
 *
 * FIX:
 *   - Do NOT use esp_mesh_lite_get_root_ip() (endian/format varies per build).
 *   - Instead target the NODE's STA gateway (ip_info.gw), which is the ROOT (192.168.5.1).
 *
 * FEATURES:
 *   - MULTIHOP enabled (allowed level > 1)
 *   - FAST reconnect (cooldown + erase RTC hint + quick scan + reconnect)
 *   - UDP batching (multiple '\n' lines per datagram)
 *
 * Tested assumptions based on your logs:
 *   - STA subnet: 192.168.5.0/24, GW: 192.168.5.1 (ROOT)
 *   - Node SoftAP subnet stays on the bridge default (typically 192.168.4.1)
 */

#include <inttypes.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include "driver/uart.h"

#include "esp_bridge.h"
#include "esp_mesh_lite.h"
#include "esp_mesh_lite_core.h"
#include "esp_mesh_lite_port.h"

static const char *TAG = "no_router_uart_udp";

/* ============================
 * TUNABLES
 * ============================ */

/* Multihop: allow multiple layers */
#ifndef MESH_ALLOWED_LEVEL
#define MESH_ALLOWED_LEVEL 6
#endif

#ifndef CONFIG_MAX_LINE_LEN
#define CONFIG_MAX_LINE_LEN 128
#endif

/* Batching */
#define UDP_BATCH_MAX_BYTES      1200   /* stay below typical MTU */
#define UDP_BATCH_FLUSH_MS       15     /* small latency bound */

/* Fast reconnect */
#define FAST_RECONNECT_COALESCE_MS 150
#define FAST_RECONNECT_COOLDOWN_MS 1500

/* ============================
 * EVENT GROUP (NODE)
 * ============================ */
static EventGroupHandle_t g_evt;
#define EVT_STA_HAS_IP      (1U << 0)
#define EVT_TARGET_DIRTY    (1U << 1)

#if !CONFIG_MESH_ROOT
static int g_sock = -1;
static struct sockaddr_in g_target;
#endif

/* ============================
 * HELPERS
 * ============================ */
static inline bool is_allowed_ascii(uint8_t c)
{
    return (c >= 32 && c <= 126);
}

static esp_err_t storage_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

/* ============================
 * WIFI / BRIDGE (match example behaviour)
 * ============================ */
static void wifi_init_bridge(void)
{
#if !CONFIG_MESH_ROOT
    /* Leaf STA cfg blank (no router) */
    wifi_config_t sta_cfg = {0};
    esp_bridge_wifi_set_config(WIFI_IF_STA, &sta_cfg);
#endif

    wifi_config_t ap_cfg = {
        .ap = {
            .ssid = CONFIG_BRIDGE_SOFTAP_SSID,
            .password = CONFIG_BRIDGE_SOFTAP_PASSWORD,
            .channel = CONFIG_MESH_CHANNEL,
            .pmf_cfg = { .capable = false, .required = false },
        },
    };
    esp_bridge_wifi_set_config(WIFI_IF_AP, &ap_cfg);
}

static void app_wifi_set_softap_info(void)
{
    char ssid[33] = {0};
    char psk[64]  = {0};
    uint8_t mac[6];
    size_t ssid_len = sizeof(ssid);
    size_t psk_len  = sizeof(psk);

    esp_wifi_get_mac(WIFI_IF_AP, mac);

    if (esp_mesh_lite_get_softap_ssid_from_nvs(ssid, &ssid_len) != ESP_OK) {
#ifdef CONFIG_BRIDGE_SOFTAP_SSID_END_WITH_THE_MAC
        snprintf(ssid, sizeof(ssid), "%.25s_%02x%02x%02x",
                 CONFIG_BRIDGE_SOFTAP_SSID, mac[3], mac[4], mac[5]);
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

static void mesh_no_router_policy_apply(void)
{
    esp_err_t err;

    err = esp_mesh_lite_set_networking_mode(ESP_MESH_LITE_MESH, 0);
    ESP_LOGI(TAG, "set_networking_mode(MESH) -> %s", esp_err_to_name(err));

    mesh_lite_sta_config_t rcfg;
    memset(&rcfg, 0, sizeof(rcfg));
    err = esp_mesh_lite_set_router_config(&rcfg);
    ESP_LOGI(TAG, "set_router_config(empty) -> %s", esp_err_to_name(err));

    /* Faster reconnect backoff cap */
    esp_mesh_lite_set_wifi_reconnect_interval(1, 8, 2);
}

/* ============================
 * UART
 * ============================ */
static void uart_init_bridge(void)
{
    const uart_port_t U = UART_NUM_1;

    uart_config_t cfg = {
        .baud_rate  = CONFIG_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(U, 8192, 8192, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(U, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(U,
                                CONFIG_UART_TX_GPIO,
                                CONFIG_UART_RX_GPIO,
                                UART_PIN_NO_CHANGE,
                                UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART ready @ %d baud (UART1 TX=%d RX=%d)",
             CONFIG_UART_BAUD, CONFIG_UART_TX_GPIO, CONFIG_UART_RX_GPIO);
}

/* ============================
 * NODE: TARGET = STA GATEWAY (ROOT)
 * ============================ */
#if !CONFIG_MESH_ROOT

static esp_netif_t *get_sta_netif(void)
{
    /* Different builds use different ifkeys; try both common ones */
    esp_netif_t *n = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (n) return n;
    n = esp_netif_get_handle_from_ifkey("WIFI_STA");
    return n;
}

static void close_sock(void)
{
    if (g_sock >= 0) {
        close(g_sock);
        g_sock = -1;
    }
}

static bool rebuild_udp_target_from_gateway(void)
{
    esp_netif_t *sta = get_sta_netif();
    if (!sta) return false;

    esp_netif_ip_info_t ip;
    if (esp_netif_get_ip_info(sta, &ip) != ESP_OK) return false;
    if (ip.gw.addr == 0) return false;

    close_sock();

    g_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (g_sock < 0) {
        ESP_LOGE(TAG, "NODE: socket() failed (errno=%d)", errno);
        return false;
    }

    memset(&g_target, 0, sizeof(g_target));
    g_target.sin_family = AF_INET;
    g_target.sin_port   = htons(CONFIG_UDP_PORT);

    /* ip.gw.addr is already network-order */
    g_target.sin_addr.s_addr = ip.gw.addr;

    char ipstr[16];
    inet_ntoa_r(g_target.sin_addr, ipstr, sizeof(ipstr));
    ESP_LOGI(TAG, "NODE UDP target (GW=ROOT): %s:%d", ipstr, CONFIG_UDP_PORT);

    return true;
}

#endif /* !CONFIG_MESH_ROOT */

/* ============================
 * FAST RECONNECT (NODE)
 * ============================ */
#if !CONFIG_MESH_ROOT
static TaskHandle_t fast_reconnect_task_h = NULL;

static void fast_reconnect_task(void *arg)
{
    (void)arg;
    uint32_t last_run = 0;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(FAST_RECONNECT_COALESCE_MS));

        uint32_t now = (uint32_t)esp_log_timestamp();
        if (now - last_run < FAST_RECONNECT_COOLDOWN_MS) continue;
        last_run = now;

        esp_mesh_lite_erase_rtc_store();
        ESP_LOGW(TAG, "FAST-RECONNECT: erase_rtc_store done");

        esp_err_t e2 = esp_mesh_lite_wifi_scan_start(NULL, 2500);
        ESP_LOGW(TAG, "FAST-RECONNECT: wifi_scan_start(2500ms) -> %s", esp_err_to_name(e2));

        (void)esp_wifi_disconnect();
        esp_err_t e3 = esp_wifi_connect();
        ESP_LOGW(TAG, "FAST-RECONNECT: esp_wifi_connect -> %s", esp_err_to_name(e3));
    }
}
#endif

/* ============================
 * EVENTS
 * ============================ */
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)data;

#if !CONFIG_MESH_ROOT
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(g_evt, EVT_STA_HAS_IP);
        xEventGroupSetBits(g_evt, EVT_TARGET_DIRTY);

        if (fast_reconnect_task_h) xTaskNotifyGive(fast_reconnect_task_h);
    }
#endif
}

static void ip_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;

#if !CONFIG_MESH_ROOT
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "STA GOT IP: " IPSTR " GW: " IPSTR,
                 IP2STR(&e->ip_info.ip), IP2STR(&e->ip_info.gw));

        xEventGroupSetBits(g_evt, EVT_STA_HAS_IP | EVT_TARGET_DIRTY);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_LOST_IP) {
        ESP_LOGW(TAG, "STA LOST IP");
        xEventGroupClearBits(g_evt, EVT_STA_HAS_IP);
        xEventGroupSetBits(g_evt, EVT_TARGET_DIRTY);
    }
#endif
}

/* ============================
 * NODE: UART -> UDP (BATCHED)
 * ============================ */
#if !CONFIG_MESH_ROOT
static void node_uart_to_udp_task(void *arg)
{
    (void)arg;

    uint8_t rx[256];

    uint8_t line[CONFIG_MAX_LINE_LEN + 2];
    size_t line_len = 0;

    uint8_t batch[UDP_BATCH_MAX_BYTES];
    size_t batch_len = 0;
    uint32_t last_activity_ms = (uint32_t)esp_log_timestamp();

    for (;;) {
        /* Wait until STA has an IP */
        xEventGroupWaitBits(g_evt, EVT_STA_HAS_IP, pdFALSE, pdTRUE, portMAX_DELAY);

        /* Rebuild target if needed */
        if (xEventGroupGetBits(g_evt) & EVT_TARGET_DIRTY) {
            xEventGroupClearBits(g_evt, EVT_TARGET_DIRTY);

            /* Keep trying until gateway is valid */
            while (!rebuild_udp_target_from_gateway()) {
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }

        int n = uart_read_bytes(UART_NUM_1, rx, sizeof(rx), pdMS_TO_TICKS(20));
        uint32_t now = (uint32_t)esp_log_timestamp();

        /* Flush if idle */
        if (batch_len > 0 && (now - last_activity_ms) >= UDP_BATCH_FLUSH_MS) {
            (void)sendto(g_sock, batch, batch_len, 0,
                         (struct sockaddr *)&g_target, sizeof(g_target));
            batch_len = 0;
        }

        if (n <= 0) continue;

        for (int i = 0; i < n; i++) {
            uint8_t c = rx[i];

            if (c == '\r') continue;

            if (c == '\n') {
                if (line_len == 0) continue;

                line[line_len++] = '\n';

                /* If adding would overflow batch, flush first */
                if (batch_len + line_len > UDP_BATCH_MAX_BYTES) {
                    (void)sendto(g_sock, batch, batch_len, 0,
                                 (struct sockaddr *)&g_target, sizeof(g_target));
                    batch_len = 0;
                }

                memcpy(&batch[batch_len], line, line_len);
                batch_len += line_len;

                line_len = 0;
                last_activity_ms = (uint32_t)esp_log_timestamp();
                continue;
            }

            if (!is_allowed_ascii(c)) {
                line_len = 0;
                continue;
            }

            if (line_len < CONFIG_MAX_LINE_LEN) {
                line[line_len++] = c;
            } else {
                line_len = 0;
            }
        }
    }
}
#endif /* !CONFIG_MESH_ROOT */

/* ============================
 * ROOT: UDP -> UART
 * ============================ */
#if CONFIG_MESH_ROOT
static void root_udp_to_uart_task(void *arg)
{
    (void)arg;

    int rsock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (rsock < 0) {
        ESP_LOGE(TAG, "ROOT: socket() failed (errno=%d)", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(CONFIG_UDP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(rsock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "ROOT: bind() failed (errno=%d)", errno);
        close(rsock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "ROOT UDP listening on %d", CONFIG_UDP_PORT);

    uint8_t buf[UDP_BATCH_MAX_BYTES];

    for (;;) {
        int n = recvfrom(rsock, buf, sizeof(buf), 0, NULL, NULL);
        if (n > 0) {
            uart_write_bytes(UART_NUM_1, (const char *)buf, n);
        }
    }
}
#endif /* CONFIG_MESH_ROOT */

/* ============================
 * APP MAIN
 * ============================ */
void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);

    ESP_ERROR_CHECK(storage_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#if !CONFIG_MESH_ROOT
    g_evt = xEventGroupCreate();
#endif

    esp_bridge_create_all_netif();
    wifi_init_bridge();

    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20));

    esp_mesh_lite_config_t cfg = ESP_MESH_LITE_DEFAULT_INIT();
    cfg.join_mesh_ignore_router_status = true;
    cfg.join_mesh_without_configured_wifi = true;

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,   ESP_EVENT_ANY_ID, &ip_event_handler,   NULL));

    esp_mesh_lite_init(&cfg);

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

#if CONFIG_MESH_ROOT
    /* ROOT must be AP-only (prevents unwanted STA behaviour) */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
#endif

    mesh_no_router_policy_apply();
    app_wifi_set_softap_info();

    /* MULTIHOP */
    esp_mesh_lite_set_allowed_level(MESH_ALLOWED_LEVEL);

#if !CONFIG_MESH_ROOT
    xTaskCreate(fast_reconnect_task, "fast_reconnect", 4096, NULL, 12, &fast_reconnect_task_h);
#endif

    esp_mesh_lite_start();

    uart_init_bridge();

#if CONFIG_MESH_ROOT
    xTaskCreate(root_udp_to_uart_task, "root_udp_to_uart", 4096, NULL, 10, NULL);
#else
    xTaskCreate(node_uart_to_udp_task, "node_uart_to_udp", 4096, NULL, 10, NULL);
#endif
}

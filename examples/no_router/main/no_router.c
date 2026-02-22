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
 * - Force Mesh-Lite networking mode to MESH and clear router config (no uplink)
 * - ROOT does NOT attempt upstream connect when used with the Mesh-Lite core fix
 * - LEAF forwards scanner UART lines as UDP datagrams to ROOT (target = STA gateway)
 */

#include <inttypes.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <stdbool.h>

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
#include "esp_event.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include "driver/uart.h"

#include "esp_bridge.h"
#include "esp_mesh_lite.h"
#include "esp_mesh_lite_core.h"
#include "esp_mesh_lite_port.h"

static const char *TAG = "no_router_uart_udp";

/* ============================================================
 * WATCHDOGS 1–4 (NO HW WDT)
 * ============================================================ */

/* ---- Tunables (conservative defaults) ---- */
#define WD_BOOT_GRACE_MS                 (30 * 1000)   // no resets for first 60s after boot
#define WD_RESET_MIN_INTERVAL_MS         (1 * 60 * 1000) // minimum 5 minutes between resets

#define WD_LEAF_NO_PARENT_MS             (30 * 1000)  // leaf: no parent for 2 minutes => reset
#define WD_ROOT_NO_CHILD_MS              (30 * 1000)  // root: no children for 3 minutes => reset

#define WD_TASK_HEARTBEAT_TIMEOUT_MS     (45 * 1000)   // if task heartbeat stale => reset
#define WD_HEAP_FLOOR_BYTES              (50 * 1024)   // heap below 60KB => reset

#define WD_UDP_FAIL_WINDOW_MS            (60 * 1000)   // count UDP fails over last 60s
#define WD_UDP_FAIL_THRESHOLD            (20)          // >= 20 fails in 60s => reset (leaf only)

/* ---- State ---- */
static uint32_t wd_boot_ms = 0;
static uint32_t wd_last_reset_attempt_ms = 0;

static uint32_t wd_last_parent_ok_ms = 0;   // leaf
static uint32_t wd_last_child_ok_ms = 0;    // root

static uint32_t wd_leaf_task_hb_ms = 0;
static uint32_t wd_root_task_hb_ms = 0;

/* UDP fail tracking (leaf) */
static uint32_t wd_udp_fail_window_start_ms = 0;
static uint32_t wd_udp_fail_count_in_window = 0;

/* ---- Helpers ---- */
static inline bool wd_in_boot_grace(uint32_t now_ms)
{
    return (now_ms - wd_boot_ms) < WD_BOOT_GRACE_MS;
}

static inline bool wd_reset_rate_limited(uint32_t now_ms)
{
    return (now_ms - wd_last_reset_attempt_ms) < WD_RESET_MIN_INTERVAL_MS;
}

static void wd_request_reset(const char *reason)
{
    uint32_t now = esp_log_timestamp();

    if (wd_in_boot_grace(now)) {
        ESP_LOGW(TAG, "WATCHDOG: would reset (%s) but still in boot grace", reason);
        return;
    }
    if (wd_reset_rate_limited(now)) {
        ESP_LOGW(TAG, "WATCHDOG: would reset (%s) but rate-limited", reason);
        return;
    }

    wd_last_reset_attempt_ms = now;
    ESP_LOGE(TAG, "WATCHDOG RESET: %s", reason);
    fflush(stdout);
    esp_restart();
}

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

        uint32_t now = esp_log_timestamp();

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

        /* ========================================================
         * WATCHDOG INPUTS
         * ======================================================== */

#if CONFIG_MESH_ROOT
        /* ROOT: if we have any children connected, we are "healthy" */
        if (wifi_sta_list.num > 0) {
            wd_last_child_ok_ms = now;
        }

        /* ROOT task heartbeat must be recent */
        if (!wd_in_boot_grace(now)) {
            if ((now - wd_root_task_hb_ms) > WD_TASK_HEARTBEAT_TIMEOUT_MS) {
                wd_request_reset("ROOT task heartbeat timeout");
            }
        }

        /* ROOT: no children for too long => reset */
        if (!wd_in_boot_grace(now)) {
            if ((now - wd_last_child_ok_ms) > WD_ROOT_NO_CHILD_MS) {
                wd_request_reset("ROOT no children timeout");
            }
        }

#else
        /* LEAF: parent bssid non-zero indicates attached to an AP */
        bool has_parent = false;
        for (int i = 0; i < 6; i++) {
            if (ap_info.bssid[i] != 0) { has_parent = true; break; }
        }
        if (has_parent) {
            wd_last_parent_ok_ms = now;
        }

        /* LEAF task heartbeat must be recent */
        if (!wd_in_boot_grace(now)) {
            if ((now - wd_leaf_task_hb_ms) > WD_TASK_HEARTBEAT_TIMEOUT_MS) {
                wd_request_reset("LEAF task heartbeat timeout");
            }
        }

        /* LEAF: no parent for too long => reset */
        if (!wd_in_boot_grace(now)) {
            if ((now - wd_last_parent_ok_ms) > WD_LEAF_NO_PARENT_MS) {
                wd_request_reset("LEAF no parent timeout");
            }
        }

        /* LEAF: UDP failure window check */
        if (!wd_in_boot_grace(now)) {
            if (wd_udp_fail_window_start_ms == 0) {
                wd_udp_fail_window_start_ms = now;
                wd_udp_fail_count_in_window = 0;
            } else if ((now - wd_udp_fail_window_start_ms) > WD_UDP_FAIL_WINDOW_MS) {
                /* roll window */
                wd_udp_fail_window_start_ms = now;
                wd_udp_fail_count_in_window = 0;
            }

            if (wd_udp_fail_count_in_window >= WD_UDP_FAIL_THRESHOLD) {
                wd_request_reset("LEAF excessive UDP send failures");
            }
        }
#endif

        /* Heap floor watchdog (both roles) */
        if (!wd_in_boot_grace(now)) {
            if (esp_get_free_heap_size() < WD_HEAP_FLOOR_BYTES) {
                wd_request_reset("Heap below floor");
            }
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
    // Leaf: empty STA config is OK; mesh decides parent
    wifi_config_t sta_cfg = {0};
    esp_bridge_wifi_set_config(WIFI_IF_STA, &sta_cfg);
#endif

    wifi_config_t ap_cfg = {
        .ap = {
            .ssid = CONFIG_BRIDGE_SOFTAP_SSID,
            .password = CONFIG_BRIDGE_SOFTAP_PASSWORD,
            .channel = CONFIG_MESH_CHANNEL,

            // ✅ FIX: stop SA Query / reason 209 disconnect loops by disabling PMF on SoftAP
            .pmf_cfg = {
                .capable  = false,
                .required = false,
            },
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
    const uart_port_t U = UART_NUM_1;

    uart_config_t cfg = {
        .baud_rate = CONFIG_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // RX buffer enabled so leaf can read scanner stream
    ESP_ERROR_CHECK(uart_driver_install(U, 4096, 4096, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(U, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(U,
                                CONFIG_UART_TX_GPIO,
                                CONFIG_UART_RX_GPIO,
                                UART_PIN_NO_CHANGE,
                                UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART ready @ %d baud (UART1 TX=%d RX=%d)",
             CONFIG_UART_BAUD, CONFIG_UART_TX_GPIO, CONFIG_UART_RX_GPIO);
}

/* ============================================================
 * Mesh-Lite policy for "no_router"
 * ============================================================ */

static void mesh_no_router_policy_apply(void)
{
    esp_err_t err;

    // 1) Force MESH-only networking mode (no uplink)
    err = esp_mesh_lite_set_networking_mode(ESP_MESH_LITE_MESH, 0);
    ESP_LOGI(TAG, "set_networking_mode(MESH) -> %s", esp_err_to_name(err));

    // 2) Clear router/uplink config (prevents any upstream connect target)
    mesh_lite_sta_config_t rcfg;
    memset(&rcfg, 0, sizeof(rcfg));
    err = esp_mesh_lite_set_router_config(&rcfg);
    ESP_LOGI(TAG, "set_router_config(empty) -> %s", esp_err_to_name(err));

    // 3) Print mode for verification
    esp_mesh_lite_networking_mode_t mode = ESP_MESH_LITE_ROUTER;
    err = esp_mesh_lite_get_networking_mode(&mode);
    ESP_LOGI(TAG, "get_networking_mode -> %s, mode=%s",
             esp_err_to_name(err),
             (mode == ESP_MESH_LITE_MESH) ? "MESH" : "ROUTER");

    // 4) Backoff reconnect spam (safe even if ignored internally)
    esp_mesh_lite_set_wifi_reconnect_interval(30, 0, 3600);
}

/* ============================================================
 * PACKET SIZE / LINE LIMIT (shared by ROOT + LEAF)
 * ============================================================ */

#ifndef CONFIG_MAX_LINE_LEN
#define CONFIG_MAX_LINE_LEN 128
#endif

/* ============================================================
 * LEAF: UART -> UDP (to ROOT)
 * ============================================================ */

#if !CONFIG_MESH_ROOT

static int leaf_udp_sock = -1;
static struct sockaddr_in root_addr;

static inline bool is_allowed_ascii(uint8_t c)
{
    // scanner sends ASCII like: room,tag,rssi\n
    // allow printable ASCII
    return (c >= 32 && c <= 126);
}

static void leaf_udp_init_when_ready(void)
{
    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!sta) {
        ESP_LOGE(TAG, "Leaf: WIFI_STA_DEF netif not found");
        return;
    }

    esp_netif_ip_info_t ip;
    if (esp_netif_get_ip_info(sta, &ip) != ESP_OK) {
        ESP_LOGW(TAG, "Leaf: esp_netif_get_ip_info failed");
        return;
    }

    if (ip.gw.addr == 0) {
        ESP_LOGW(TAG, "Leaf: no GW yet; waiting for IP");
        return;
    }

    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s < 0) {
        ESP_LOGE(TAG, "Leaf: socket() failed");
        return;
    }

    memset(&root_addr, 0, sizeof(root_addr));
    root_addr.sin_family = AF_INET;
    root_addr.sin_port = htons(CONFIG_UDP_PORT);
    root_addr.sin_addr.s_addr = ip.gw.addr; // ROOT is our gateway (current parent)

    leaf_udp_sock = s;

    ESP_LOGI(TAG, "Leaf UDP target: %s:%d",
             inet_ntoa(root_addr.sin_addr), CONFIG_UDP_PORT);
}

// Reset/re-resolve UDP target after parent changes
static void leaf_udp_reset_and_reresolve(void)
{
    if (leaf_udp_sock >= 0) {
        close(leaf_udp_sock);
        leaf_udp_sock = -1;
    }

    while (leaf_udp_sock < 0) {
        leaf_udp_init_when_ready();
        if (leaf_udp_sock >= 0) break;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void leaf_uart_to_udp_task(void *arg)
{
    (void)arg;
    const uart_port_t U = UART_NUM_1;

    leaf_udp_reset_and_reresolve();

    static uint8_t line[CONFIG_MAX_LINE_LEN + 2];
    size_t len = 0;

    uint8_t rx[256];

    for (;;) {
        /* Task heartbeat for watchdog */
        wd_leaf_task_hb_ms = esp_log_timestamp();

        int n = uart_read_bytes(U, rx, sizeof(rx), pdMS_TO_TICKS(100));
        if (n <= 0) continue;

        for (int i = 0; i < n; i++) {
            uint8_t c = rx[i];

            if (c == '\r') continue;

            if (c == '\n') {
                if (len == 0) continue;

                line[len++] = '\n';

                int sent = sendto(leaf_udp_sock, line, len, 0,
                                  (struct sockaddr *)&root_addr, sizeof(root_addr));
                if (sent < 0) {
                    ESP_LOGW(TAG, "Leaf: sendto failed; resetting UDP target (errno=%d)", errno);

                    /* Count failure in watchdog window */
                    uint32_t now = esp_log_timestamp();
                    if (wd_udp_fail_window_start_ms == 0) {
                        wd_udp_fail_window_start_ms = now;
                        wd_udp_fail_count_in_window = 0;
                    }
                    if ((now - wd_udp_fail_window_start_ms) > WD_UDP_FAIL_WINDOW_MS) {
                        wd_udp_fail_window_start_ms = now;
                        wd_udp_fail_count_in_window = 0;
                    }
                    wd_udp_fail_count_in_window++;

                    leaf_udp_reset_and_reresolve();
                    (void)sendto(leaf_udp_sock, line, len, 0,
                                 (struct sockaddr *)&root_addr, sizeof(root_addr));
                } else {
                    /* On success, slowly forgive by resetting window if quiet */
                    uint32_t now = esp_log_timestamp();
                    if (wd_udp_fail_window_start_ms == 0) {
                        wd_udp_fail_window_start_ms = now;
                        wd_udp_fail_count_in_window = 0;
                    }
                    if ((now - wd_udp_fail_window_start_ms) > WD_UDP_FAIL_WINDOW_MS) {
                        wd_udp_fail_window_start_ms = now;
                        wd_udp_fail_count_in_window = 0;
                    }
                }

                len = 0;
                continue;
            }

            if (!is_allowed_ascii(c)) {
                len = 0;
                continue;
            }

            if (len < (size_t)CONFIG_MAX_LINE_LEN) {
                line[len++] = c;
            } else {
                len = 0;
            }
        }
    }
}

#endif /* !CONFIG_MESH_ROOT */

/* ============================================================
 * ROOT: UDP -> UART
 * ============================================================ */

#if CONFIG_MESH_ROOT

static void root_udp_to_uart_task(void *arg)
{
    (void)arg;
    const uart_port_t U = UART_NUM_1;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Root: socket() failed");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "Root: bind() failed on port %d", CONFIG_UDP_PORT);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Root UDP listening on %d", CONFIG_UDP_PORT);

    uint8_t buf[CONFIG_MAX_LINE_LEN + 2];

    for (;;) {
        /* Task heartbeat for watchdog */
        wd_root_task_hb_ms = esp_log_timestamp();

        int n = recvfrom(sock, buf, sizeof(buf), 0, NULL, NULL);
        if (n > 0) {
            uart_write_bytes(U, (const char *)buf, n);
        }
    }
}

#endif /* CONFIG_MESH_ROOT */

/* ============================================================
 * APP MAIN
 * ============================================================ */

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);

    /* Watchdog init */
    wd_boot_ms = esp_log_timestamp();
    wd_last_reset_attempt_ms = 0;

    wd_last_parent_ok_ms = wd_boot_ms;
    wd_last_child_ok_ms  = wd_boot_ms;

    wd_leaf_task_hb_ms = wd_boot_ms;
    wd_root_task_hb_ms = wd_boot_ms;

    wd_udp_fail_window_start_ms = 0;
    wd_udp_fail_count_in_window = 0;

    ESP_ERROR_CHECK(esp_storage_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_bridge_create_all_netif();
    wifi_init();

    // ✅ FIX: force HT20 bandwidth (reduces channel-width churn / instability)
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20));

    esp_mesh_lite_config_t cfg = ESP_MESH_LITE_DEFAULT_INIT();
    cfg.join_mesh_ignore_router_status = true;
    cfg.join_mesh_without_configured_wifi = true;

    esp_mesh_lite_init(&cfg);

    // ✅ FIX: disable Wi-Fi power save
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

#if CONFIG_MESH_ROOT
    // ✅ FIX: force ROOT AP-only
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
#endif

    mesh_no_router_policy_apply();

    app_wifi_set_softap_info();

#if CONFIG_MESH_ROOT
    ESP_LOGI(TAG, "Role: ROOT");
    esp_mesh_lite_set_allowed_level(1);
#else
    ESP_LOGI(TAG, "Role: LEAF");
    esp_mesh_lite_set_disallowed_level(10);
#endif

    esp_mesh_lite_start();

    uart_init_bridge();

#if CONFIG_MESH_ROOT
    xTaskCreate(root_udp_to_uart_task, "root_udp_to_uart", 4096, NULL, 10, NULL);
#else
    xTaskCreate(leaf_uart_to_udp_task, "leaf_uart_to_udp", 4096, NULL, 10, NULL);
#endif

    xTaskCreate(sysinfo_task, "sysinfo", 4096, NULL, 5, &sysinfo_task_h);

    TimerHandle_t t = xTimerCreate("sysinfo_timer",
                                   pdMS_TO_TICKS(10000),
                                   true, NULL,
                                   sysinfo_timer_cb);
    xTimerStart(t, 0);
}

/*
 * ESP-Mesh-Lite "no_router" + UART<->UDP bridge for your system
 *
 * LEAF (CONFIG_MESH_ROOT=n):
 *   - Reads UART @115200
 *   - Buffers until '\n'
 *   - Drops non-ASCII / oversized / partial lines
 *   - Sends each line as ONE UDP datagram to Root (gateway IP) on CONFIG_UDP_PORT
 *
 * ROOT (CONFIG_MESH_ROOT=y):
 *   - Receives UDP datagrams on CONFIG_UDP_PORT
 *   - Forwards payload AS-IS to UART @115200 (to UI)
 *
 * Messaging stays: "room,tag,rssi\n" (ASCII, newline terminated)
 */

#include <inttypes.h>
#include <string.h>
#include <ctype.h>

#include "esp_log.h"
#include "esp_system.h"
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

static const char *TAG = "no_router_uart_udp";

/* -----------------------------
 * Existing periodic info print
 * ----------------------------- */
static void print_system_info_timercb(TimerHandle_t timer)
{
    uint8_t primary = 0;
    uint8_t sta_mac[6] = {0};
    wifi_ap_record_t ap_info = {0};
    wifi_second_chan_t second = 0;
    wifi_sta_list_t wifi_sta_list = {0x0};

    if (esp_mesh_lite_get_level() > 1) {
        esp_wifi_sta_get_ap_info(&ap_info);
    }
    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    esp_wifi_get_channel(&primary, &second);

    ESP_LOGI(TAG,
             "System info, ch=%d layer=%d self=" MACSTR " parent=" MACSTR " parent_rssi=%d free_heap=%" PRIu32,
             primary, esp_mesh_lite_get_level(),
             MAC2STR(sta_mac), MAC2STR(ap_info.bssid),
             (ap_info.rssi != 0 ? ap_info.rssi : -120),
             esp_get_free_heap_size());

    for (int i = 0; i < wifi_sta_list.num; i++) {
        ESP_LOGI(TAG, "Child mac: " MACSTR, MAC2STR(wifi_sta_list.sta[i].mac));
    }
}

/* -----------------------------
 * NVS init
 * ----------------------------- */
static esp_err_t esp_storage_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

/* -----------------------------
 * WiFi / Mesh-Lite init
 * ----------------------------- */
static void wifi_init(void)
{
    /*
     * CRITICAL FIX:
     * - LEAF must configure STA (it joins the ROOT SoftAP network and uses GW IP as target).
     * - ROOT must NOT configure STA in "no_router" mode (no uplink/router exists), otherwise it will spam
     *   esp_wifi_connect() failures trying to connect to an empty SSID.
     */
#if !CONFIG_MESH_ROOT
    wifi_config_t wifi_config;
    memset(&wifi_config, 0x0, sizeof(wifi_config_t));
    esp_bridge_wifi_set_config(WIFI_IF_STA, &wifi_config);
#endif

    wifi_config_t wifi_softap_config = {
        .ap = {
            .ssid = CONFIG_BRIDGE_SOFTAP_SSID,
            .password = CONFIG_BRIDGE_SOFTAP_PASSWORD,
            .channel = CONFIG_MESH_CHANNEL,
        },
    };
    esp_bridge_wifi_set_config(WIFI_IF_AP, &wifi_softap_config);
}

static void app_wifi_set_softap_info(void)
{
    char softap_ssid[33];
    char softap_psw[64];
    uint8_t softap_mac[6];
    size_t ssid_size = sizeof(softap_ssid);
    size_t psw_size = sizeof(softap_psw);

    esp_wifi_get_mac(WIFI_IF_AP, softap_mac);
    memset(softap_ssid, 0x0, sizeof(softap_ssid));
    memset(softap_psw, 0x0, sizeof(softap_psw));

    if (esp_mesh_lite_get_softap_ssid_from_nvs(softap_ssid, &ssid_size) == ESP_OK) {
        ESP_LOGI(TAG, "SoftAP SSID from NVS: %s", softap_ssid);
    } else {
#ifdef CONFIG_BRIDGE_SOFTAP_SSID_END_WITH_THE_MAC
        snprintf(softap_ssid, sizeof(softap_ssid), "%.25s_%02x%02x%02x",
                 CONFIG_BRIDGE_SOFTAP_SSID, softap_mac[3], softap_mac[4], softap_mac[5]);
#else
        snprintf(softap_ssid, sizeof(softap_ssid), "%.32s", CONFIG_BRIDGE_SOFTAP_SSID);
#endif
        ESP_LOGI(TAG, "SoftAP SSID default: %s", softap_ssid);
    }

    if (esp_mesh_lite_get_softap_psw_from_nvs(softap_psw, &psw_size) == ESP_OK) {
        ESP_LOGI(TAG, "SoftAP PSK from NVS: [HIDDEN]");
    } else {
        strlcpy(softap_psw, CONFIG_BRIDGE_SOFTAP_PASSWORD, sizeof(softap_psw));
        ESP_LOGI(TAG, "SoftAP PSK default: [HIDDEN]");
    }

    esp_mesh_lite_set_softap_info(softap_ssid, softap_psw);
}

/* -----------------------------
 * UART helpers
 * ----------------------------- */
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

    ESP_ERROR_CHECK(uart_driver_install(U, 4096, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(U, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(U, CONFIG_UART_TX_GPIO, CONFIG_UART_RX_GPIO,
                                UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART1 init: %d baud TX=%d RX=%d",
             CONFIG_UART_BAUD, CONFIG_UART_TX_GPIO, CONFIG_UART_RX_GPIO);
}

static inline bool is_allowed_ascii(uint8_t c)
{
    // Allow: printable ASCII + comma + minus; we also tolerate CR
    if (c == '\r') return true;
    if (c == '\n') return true;
    return (c >= 0x20 && c <= 0x7E);
}

/* -----------------------------
 * Network helpers
 * ----------------------------- */
static bool get_gateway_ip(struct in_addr *out_gw)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) return false;

    esp_netif_ip_info_t ip;
    if (esp_netif_get_ip_info(netif, &ip) != ESP_OK) return false;
    if (ip.gw.addr == 0) return false;

    out_gw->s_addr = ip.gw.addr;
    return true;
}

/* -----------------------------
 * LEAF: UART -> UDP
 * ----------------------------- */
#if !CONFIG_MESH_ROOT

static int leaf_udp_sock = -1;
static struct sockaddr_in root_addr;

static void leaf_udp_init_when_ready(void)
{
    // Wait until we have a gateway; in Mesh-Lite this is effectively the upstream/root bridge IP.
    struct in_addr gw;
    while (!get_gateway_ip(&gw)) {
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    leaf_udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (leaf_udp_sock < 0) {
        ESP_LOGE(TAG, "Leaf socket() failed");
        return;
    }

    memset(&root_addr, 0, sizeof(root_addr));
    root_addr.sin_family = AF_INET;
    root_addr.sin_port = htons(CONFIG_UDP_PORT);
    root_addr.sin_addr = gw;

    ESP_LOGI(TAG, "Leaf UDP target: %s:%d", inet_ntoa(root_addr.sin_addr), CONFIG_UDP_PORT);
}

static void leaf_uart_to_udp_task(void *arg)
{
    const uart_port_t U = UART_NUM_1;

    leaf_udp_init_when_ready();
    if (leaf_udp_sock < 0) {
        ESP_LOGE(TAG, "Leaf UDP not available; task stopping");
        vTaskDelete(NULL);
        return;
    }

    static uint8_t line[CONFIG_MAX_LINE_LEN + 2]; // + '\n' + safety
    size_t len = 0;

    uint8_t rx[256];

    for (;;) {
        int n = uart_read_bytes(U, rx, sizeof(rx), pdMS_TO_TICKS(100));
        if (n <= 0) continue;

        for (int i = 0; i < n; i++) {
            uint8_t c = rx[i];

            if (!is_allowed_ascii(c)) {
                // Drop current partial line on any non-ASCII byte
                len = 0;
                continue;
            }

            if (c == '\r') {
                // ignore CR
                continue;
            }

            if (c == '\n') {
                if (len == 0) continue; // ignore empty lines

                // append newline for exact payload semantics
                line[len++] = '\n';

                // send as ONE datagram
                int sent = sendto(leaf_udp_sock, line, len, 0,
                                  (struct sockaddr *)&root_addr, sizeof(root_addr));
                if (sent < 0) {
                    ESP_LOGW(TAG, "Leaf sendto failed");
                }

                len = 0;
                continue;
            }

            // normal byte
            if (len < (size_t)CONFIG_MAX_LINE_LEN) {
                line[len++] = c;
            } else {
                // Oversized -> drop the line entirely
                len = 0;
            }
        }
    }
}

#endif /* !CONFIG_MESH_ROOT */

/* -----------------------------
 * ROOT: UDP -> UART
 * ----------------------------- */
#if CONFIG_MESH_ROOT

static void root_udp_to_uart_task(void *arg)
{
    const uart_port_t U = UART_NUM_1;

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Root socket() failed");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(CONFIG_UDP_PORT);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) != 0) {
        ESP_LOGE(TAG, "Root bind() failed on port %d", CONFIG_UDP_PORT);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Root UDP listening on %d", CONFIG_UDP_PORT);

    uint8_t buf[CONFIG_MAX_LINE_LEN + 2];

    for (;;) {
        int n = recvfrom(sock, buf, sizeof(buf), 0, NULL, NULL);
        if (n <= 0) continue;

        // Forward payload AS-IS to UART
        uart_write_bytes(U, (const char *)buf, n);
    }
}

#endif /* CONFIG_MESH_ROOT */

/* -----------------------------
 * app_main
 * ----------------------------- */
void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);

    ESP_ERROR_CHECK(esp_storage_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_bridge_create_all_netif();
    wifi_init();

    esp_mesh_lite_config_t mesh_lite_config = ESP_MESH_LITE_DEFAULT_INIT();

    // Keep joining mesh even if "router status" would be considered down (no_router means no uplink).
    mesh_lite_config.join_mesh_ignore_router_status = true;

    /*
     * In "no_router" mode, BOTH root and leaf must be allowed to form/join mesh
     * without any configured upstream Wi-Fi credentials.
     */
    mesh_lite_config.join_mesh_without_configured_wifi = true;

    // These are VOID-returning APIs in your version, so do not wrap with ESP_ERROR_CHECK
    esp_mesh_lite_init(&mesh_lite_config);

    // *** KEY FIX ***
    // Force mesh-only networking mode so ROOT won't attempt to connect to an external AP/router.
    ESP_ERROR_CHECK(esp_mesh_lite_set_networking_mode(ESP_MESH_LITE_MESH, 0));

    app_wifi_set_softap_info();

#if CONFIG_MESH_ROOT
    ESP_LOGI(TAG, "Role: ROOT");
    esp_mesh_lite_set_allowed_level(1);
#else
    ESP_LOGI(TAG, "Role: LEAF/CHILD");
    esp_mesh_lite_set_disallowed_level(1);
#endif

    // VOID-returning API
    esp_mesh_lite_start();

    // UART bridge init for both roles
    uart_init_bridge();

#if CONFIG_MESH_ROOT
    xTaskCreate(root_udp_to_uart_task, "root_udp_to_uart", 4096, NULL, 10, NULL);
#else
    xTaskCreate(leaf_uart_to_udp_task, "leaf_uart_to_udp", 4096, NULL, 10, NULL);
#endif

    TimerHandle_t timer = xTimerCreate("print_system_info", pdMS_TO_TICKS(10000),
                                       true, NULL, print_system_info_timercb);
    xTimerStart(timer, 0);
}

/*
 * ESP-Mesh-Lite "no_router" UART <-> UDP bridge (multihop-safe)
 *
 * NODE:
 *   UART -> UDP -> ROOT   (target = STA gateway IP; works for multihop)
 *
 * ROOT:
 *   UDP -> UART
 *
 * NO ROUTER / NO INTERNET
 *
 * FIXES + IMPROVEMENTS:
 *  1) Correct ROOT targeting:
 *     - Do NOT use esp_mesh_lite_get_root_ip() (can be 0 or endian-trappy depending on build)
 *     - Instead: send UDP to STA gateway (parent) IP; Mesh-Lite routes toward root.
 *
 *  2) DHCP/IP-change safe:
 *     - On IP_EVENT_STA_GOT_IP or IP_EVENT_STA_LOST_IP: rebuild UDP socket/target.
 *     - On WIFI disconnect: clear readiness and rebuild later.
 *
 *  3) Scale improvement (25 rooms / 2000 tags):
 *     - Batch multiple UART lines into a single UDP datagram (still newline-delimited).
 *       This massively reduces packets/sec and drop probability.
 *     - Increase UART RX buffer and keep a fast parse/send task.
 *
 *  4) Quick reconnect (NODE):
 *     - On STA_DISCONNECTED:
 *         cooldown -> erase Mesh-Lite RTC parent hint -> short scan -> reconnect now.
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "driver/uart.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include "esp_bridge.h"
#include "esp_mesh_lite.h"

/* ============================ USER CONFIG ============================ */

#ifndef CONFIG_UDP_PORT
#define CONFIG_UDP_PORT 4510
#endif

#ifndef CONFIG_MAX_LINE_LEN
#define CONFIG_MAX_LINE_LEN 128
#endif

/* Bigger UART RX buffer helps avoid overflow when scanners burst */
#ifndef CONFIG_UART_RX_BUF
#define CONFIG_UART_RX_BUF (16 * 1024)
#endif

/* UDP batching target size (stay well below MTU) */
#ifndef CONFIG_UDP_BATCH_MAX
#define CONFIG_UDP_BATCH_MAX 1200
#endif

/* Flush batch if no new line for this long */
#ifndef CONFIG_UDP_BATCH_FLUSH_MS
#define CONFIG_UDP_BATCH_FLUSH_MS 10
#endif

/* Socket send buffer (best-effort; lwIP may clamp) */
#ifndef CONFIG_UDP_SNDBUF
#define CONFIG_UDP_SNDBUF (32 * 1024)
#endif

static const char *TAG = "no_router_uart_udp";

/* ============================ GLOBALS ============================ */

static EventGroupHandle_t g_evt;
#define EVT_STA_HAS_IP      (1U << 0)
#define EVT_REBUILD_TARGET  (1U << 1)

#if !CONFIG_MESH_ROOT
static int g_sock = -1;
static struct sockaddr_in g_target;
#endif

/* ============================ HELPERS ============================ */

static inline bool is_allowed_ascii(uint8_t c)
{
    return (c >= 32 && c <= 126);
}

static esp_err_t storage_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

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

    ESP_ERROR_CHECK(uart_driver_install(U, CONFIG_UART_RX_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(U, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(U,
                                CONFIG_UART_TX_GPIO,
                                CONFIG_UART_RX_GPIO,
                                UART_PIN_NO_CHANGE,
                                UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART ready @ %d baud (UART1 TX=%d RX=%d, RXbuf=%d)",
             CONFIG_UART_BAUD, CONFIG_UART_TX_GPIO, CONFIG_UART_RX_GPIO, CONFIG_UART_RX_BUF);
}

static void wifi_init_bridge(void)
{
    /* SoftAP config always present (mesh uses it) */
    wifi_config_t ap_cfg = {
        .ap = {
            .ssid = CONFIG_BRIDGE_SOFTAP_SSID,
            .password = CONFIG_BRIDGE_SOFTAP_PASSWORD,
            .channel = CONFIG_MESH_CHANNEL,
            .pmf_cfg = { .capable = false, .required = false },
        },
    };
    esp_bridge_wifi_set_config(WIFI_IF_AP, &ap_cfg);

#if !CONFIG_MESH_ROOT
    /* No-router STA cfg blank */
    wifi_config_t sta_cfg = {0};
    esp_bridge_wifi_set_config(WIFI_IF_STA, &sta_cfg);
#endif
}

/* ============================ NODE TARGETING: STA GATEWAY ============================ */

#if !CONFIG_MESH_ROOT

static bool get_sta_gateway(uint32_t *gw_addr_net_order)
{
    if (!gw_addr_net_order) return false;

    esp_netif_t *sta = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!sta) return false;

    esp_netif_ip_info_t ip;
    if (esp_netif_get_ip_info(sta, &ip) != ESP_OK) return false;

    if (ip.gw.addr == 0) return false;

    *gw_addr_net_order = ip.gw.addr; /* already network order */
    return true;
}

static void close_sock(void)
{
    if (g_sock >= 0) {
        close(g_sock);
        g_sock = -1;
    }
}

static void rebuild_udp_target(void)
{
    uint32_t gw_net = 0;

    /* Must have IP first */
    if ((xEventGroupGetBits(g_evt) & EVT_STA_HAS_IP) == 0) {
        return;
    }

    if (!get_sta_gateway(&gw_net)) {
        return;
    }

    close_sock();

    g_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (g_sock < 0) {
        ESP_LOGE(TAG, "NODE: socket() failed (errno=%d)", errno);
        return;
    }

    /* Try increase send buffer */
    int snd = CONFIG_UDP_SNDBUF;
    (void)setsockopt(g_sock, SOL_SOCKET, SO_SNDBUF, &snd, sizeof(snd));

    memset(&g_target, 0, sizeof(g_target));
    g_target.sin_family = AF_INET;
    g_target.sin_port   = htons(CONFIG_UDP_PORT);
    g_target.sin_addr.s_addr = gw_net;

    char ipstr[16];
    inet_ntoa_r(g_target.sin_addr, ipstr, sizeof(ipstr));
    ESP_LOGI(TAG, "NODE UDP target (GW->ROOT): %s:%d", ipstr, CONFIG_UDP_PORT);
}

#endif /* !CONFIG_MESH_ROOT */

/* ============================ NODE QUICK RECONNECT ============================ */

#if !CONFIG_MESH_ROOT

static TaskHandle_t fast_reconnect_task_h = NULL;

#define FAST_RECONNECT_COALESCE_MS    150
#define FAST_RECONNECT_COOLDOWN_MS   1500

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

        /* Clear old parent hint */
        esp_mesh_lite_erase_rtc_store();
        ESP_LOGW(TAG, "FAST-RECONNECT: erase_rtc_store done");

        /* Quick scan to find alternate parent */
        esp_err_t e2 = esp_mesh_lite_wifi_scan_start(NULL, 2500);
        ESP_LOGW(TAG, "FAST-RECONNECT: wifi_scan_start(2500ms) -> %s", esp_err_to_name(e2));

        /* Kick reconnect */
        (void)esp_wifi_disconnect();
        esp_err_t e3 = esp_wifi_connect();
        ESP_LOGW(TAG, "FAST-RECONNECT: esp_wifi_connect -> %s", esp_err_to_name(e3));
    }
}

#endif /* !CONFIG_MESH_ROOT */

/* ============================ EVENTS ============================ */

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)data;

#if !CONFIG_MESH_ROOT
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        /* Not ready until we regain IP */
        xEventGroupClearBits(g_evt, EVT_STA_HAS_IP);
        xEventGroupSetBits(g_evt, EVT_REBUILD_TARGET);

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

        xEventGroupSetBits(g_evt, EVT_STA_HAS_IP | EVT_REBUILD_TARGET);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_LOST_IP) {
        ESP_LOGW(TAG, "STA LOST IP");
        xEventGroupClearBits(g_evt, EVT_STA_HAS_IP);
        xEventGroupSetBits(g_evt, EVT_REBUILD_TARGET);
    }
#endif
}

/* ============================ NODE: UART -> UDP (BATCHED) ============================ */

#if !CONFIG_MESH_ROOT

static void node_uart_to_udp_task(void *arg)
{
    (void)arg;

    uint8_t rx[256];

    /* line assembly */
    uint8_t line[CONFIG_MAX_LINE_LEN + 1];
    int line_len = 0;

    /* UDP batch buffer */
    uint8_t batch[CONFIG_UDP_BATCH_MAX];
    int batch_len = 0;
    uint32_t last_flush_ms = (uint32_t)esp_log_timestamp();

    for (;;) {
        /* Wait until we have IP at least once */
        xEventGroupWaitBits(g_evt, EVT_STA_HAS_IP, pdFALSE, pdTRUE, portMAX_DELAY);

        /* Rebuild target if requested */
        EventBits_t b = xEventGroupGetBits(g_evt);
        if (b & EVT_REBUILD_TARGET) {
            xEventGroupClearBits(g_evt, EVT_REBUILD_TARGET);
            rebuild_udp_target();
        }

        /* If socket not ready yet, keep trying */
        if (g_sock < 0) {
            rebuild_udp_target();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int n = uart_read_bytes(UART_NUM_1, rx, sizeof(rx), pdMS_TO_TICKS(20));
        uint32_t now = (uint32_t)esp_log_timestamp();

        /* Flush if idle for a moment and batch has data */
        if (batch_len > 0 && (now - last_flush_ms) >= CONFIG_UDP_BATCH_FLUSH_MS) {
            int sent = sendto(g_sock, batch, batch_len, 0,
                              (struct sockaddr *)&g_target, sizeof(g_target));
            if (sent < 0) {
                ESP_LOGW(TAG, "NODE: batch send failed (errno=%d) -> rebuild target", errno);
                xEventGroupSetBits(g_evt, EVT_REBUILD_TARGET);
            }
            batch_len = 0;
            last_flush_ms = now;
        }

        if (n <= 0) continue;

        for (int i = 0; i < n; i++) {
            uint8_t c = rx[i];

            if (c == '\r') continue;

            if (c == '\n') {
                if (line_len == 0) continue;

                /* append newline */
                line[line_len++] = '\n';

                /* If line won't fit in batch, flush batch first */
                if (line_len > CONFIG_UDP_BATCH_MAX) {
                    /* absurdly large (should never happen due to MAX_LINE), drop */
                    line_len = 0;
                    continue;
                }

                if (batch_len + line_len > CONFIG_UDP_BATCH_MAX) {
                    int sent = sendto(g_sock, batch, batch_len, 0,
                                      (struct sockaddr *)&g_target, sizeof(g_target));
                    if (sent < 0) {
                        ESP_LOGW(TAG, "NODE: batch send failed (errno=%d) -> rebuild target", errno);
                        xEventGroupSetBits(g_evt, EVT_REBUILD_TARGET);
                        batch_len = 0;
                        line_len = 0;
                        break;
                    }
                    batch_len = 0;
                    last_flush_ms = (uint32_t)esp_log_timestamp();
                }

                memcpy(&batch[batch_len], line, line_len);
                batch_len += line_len;
                line_len = 0;

                /* If batch is “big enough”, flush immediately */
                if (batch_len >= (CONFIG_UDP_BATCH_MAX - (CONFIG_MAX_LINE_LEN + 2))) {
                    int sent = sendto(g_sock, batch, batch_len, 0,
                                      (struct sockaddr *)&g_target, sizeof(g_target));
                    if (sent < 0) {
                        ESP_LOGW(TAG, "NODE: batch send failed (errno=%d) -> rebuild target", errno);
                        xEventGroupSetBits(g_evt, EVT_REBUILD_TARGET);
                    }
                    batch_len = 0;
                    last_flush_ms = (uint32_t)esp_log_timestamp();
                }

                continue;
            }

            /* ASCII-only rule */
            if (!is_allowed_ascii(c)) {
                line_len = 0;
                continue;
            }

            if (line_len < CONFIG_MAX_LINE_LEN) {
                line[line_len++] = c;
            } else {
                /* oversize -> drop */
                line_len = 0;
            }
        }
    }
}

#endif /* !CONFIG_MESH_ROOT */

/* ============================ ROOT: UDP -> UART ============================ */

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

    uint8_t buf[CONFIG_UDP_BATCH_MAX];

    for (;;) {
        int n = recvfrom(rsock, buf, sizeof(buf), 0, NULL, NULL);
        if (n > 0) {
            uart_write_bytes(UART_NUM_1, (const char *)buf, n);
        }
    }
}

#endif /* CONFIG_MESH_ROOT */

/* ============================ APP MAIN ============================ */

void app_main(void)
{
    ESP_ERROR_CHECK(storage_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    g_evt = xEventGroupCreate();

    esp_bridge_create_all_netif();
    wifi_init_bridge();

    esp_mesh_lite_config_t cfg = ESP_MESH_LITE_DEFAULT_INIT();
    cfg.join_mesh_ignore_router_status = true;
    cfg.join_mesh_without_configured_wifi = true;

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &ip_event_handler, NULL));

    esp_mesh_lite_init(&cfg);

#if CONFIG_MESH_ROOT
    /* ROOT must be AP-only in no_router mode */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
#else
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

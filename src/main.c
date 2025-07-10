/*
 * Archivo: main.c
 * Descripción: Firmware refactorizado y optimizado para nRF9151 con NTN.
 * Versión: 3.0
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/drivers/watchdog.h>

#include <modem/lte_lc.h>
#include <modem/at_cmd_parser.h>
#include <modem/at_monitor.h>
#include <nrf_modem_at.h>
#include <nrf_modem_gnss.h>


LOG_MODULE_REGISTER(ntn_app, LOG_LEVEL_INF);

// --- CONFIGURACIÓN DE GESTIÓN DE FASES ---
enum integration_phase {
    PHASE_TN_TESTING,
    PHASE_NTN_TESTING
};
#define CURRENT_INTEGRATION_PHASE PHASE_NTN_TESTING

// --- CONFIGURACIÓN DE LA APLICACIÓN ---
#define SERVER_IP "100.100.100.100"
#define SERVER_PORT 17777
#define SATELIOT_PLMN "90197"
#define PAYLOAD_BUFFER_SIZE 256

// --- ESTADOS DE LA MÁQUINA DE ESTADOS ---
enum app_state {
    STATE_INIT,
    STATE_GETTING_GPS_FIX,
    STATE_IDLE,
    STATE_ATTEMPTING_CONNECTION,
    STATE_SENDING_DATA,
    STATE_ERROR
};

// --- ESTRUCTURAS DE DATOS ---
struct satellite_info {
    char tle_line1[70];
    char tle_line2[70];
    int64_t next_pass_start;
    int64_t next_pass_end;
};

// --- VARIABLES GLOBALES ---
static enum app_state current_state = STATE_INIT;
static K_SEM_DEFINE(lte_connected_sem, 0, 1);
static K_SEM_DEFINE(gps_fix_sem, 0, 1);
static struct nrf_modem_gnss_pvt_data_frame last_gps_data;
static char payload_buffer[PAYLOAD_BUFFER_SIZE];
static const struct device *const wdt_dev = DEVICE_DT_GET(DT_ALIAS(watchdog0));
static int wdt_channel_id;


// --- DECLARACIÓN DE FUNCIONES ---
static void lte_handler(const struct lte_lc_evt *const evt);
static void gnss_event_handler(int event);
static int setup_watchdog(void);
static int configure_power_management(void);
static int modem_configure_for_ntn(void);
static int robust_data_send(const char *payload);
static int format_telemetry_data(char *buffer, size_t buffer_size);
static int calculate_satellite_pass(struct satellite_info *sat_info, double ground_lat, double ground_lon);


// =================================================================
//  INICIALIZACIÓN Y SISTEMA
// =================================================================

static int setup_watchdog(void) {
    if (!device_is_ready(wdt_dev)) {
        LOG_ERR("Watchdog no está listo");
        return -ENODEV;
    }
    struct wdt_timeout_cfg wdt_config = {
        .flags = WDT_FLAG_RESET_SOC,
        .window.min = 0,
        .window.max = 60000, // 60 segundos
        .callback = NULL,
    };
    wdt_channel_id = wdt_install_timeout(wdt_dev, &wdt_config);
    if (wdt_channel_id < 0) {
        LOG_ERR("Fallo al instalar el timeout del watchdog: %d", wdt_channel_id);
        return wdt_channel_id;
    }
    return wdt_setup(wdt_dev, WDT_OPT_PAUSE_HALTED_BY_DBG);
}

static int configure_power_management(void) {
    int err;
    // T3324 (PSM Active Timer) = 1 minuto, T3412 (Periodic TAU Timer) = 1 hora
    const char *psm_val = "00100001"; // T3412
    const char *tau_val = "00000001"; // T3324
    
    err = lte_lc_psm_param_set(tau_val, psm_val);
    if (err) {
        LOG_ERR("Fallo al establecer parámetros de PSM: %d", err);
        return err;
    }
    LOG_INF("Parámetros de PSM establecidos.");

    // eDRX para NB-IoT, ciclo de 163.84 segundos
    err = lte_lc_edrx_param_set(LTE_LC_LTE_MODE_NBIOT, "1001");
    if (err) {
        LOG_WRN("Fallo al establecer eDRX: %d", err);
    } else {
        LOG_INF("eDRX configurado.");
    }
    return 0;
}


// =================================================================
//  LÓGICA DE PREDICCIÓN DE SATÉLITE
// =================================================================

static int calculate_satellite_pass(struct satellite_info *sat_info, double ground_lat, double ground_lon) {
    // Aquí iría la implementación real con una librería SGP4
    // Placeholder mejorado:
    int64_t current_time = k_uptime_get();
    int64_t orbital_period_ms = 96 * 60 * 1000; // Periodo orbital de 96 min
    int64_t time_since_epoch = current_time % orbital_period_ms;
    int64_t time_to_next_pass = orbital_period_ms - time_since_epoch;

    sat_info->next_pass_start = current_time + time_to_next_pass;
    sat_info->next_pass_end = sat_info->next_pass_start + (8 * 60 * 1000); // Ventana de 8 min

    LOG_INF("Próximo paso de satélite simulado en %llds", time_to_next_pass / 1000);
    return 0;
}

// =================================================================
//  LÓGICA DE GPS (GNSS)
// =================================================================

static void gnss_event_handler(int event) {
    if (event == NRF_MODEM_GNSS_EVT_PVT) {
        int err = nrf_modem_gnss_read(&last_gps_data, sizeof(last_gps_data), NRF_MODEM_GNSS_DATA_PVT);
        if (err == 0 && (last_gps_data.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID)) {
            LOG_INF("GNSS: Fix válido obtenido!");
            k_sem_give(&gps_fix_sem);
        }
    }
}

static int gnss_init_and_start(void) {
    if (nrf_modem_gnss_event_handler_set(gnss_event_handler) != 0) {
        LOG_ERR("Fallo al establecer el manejador de eventos GNSS.");
        return -EFAULT;
    }
    if (nrf_modem_gnss_start() != 0) {
        LOG_ERR("Fallo al iniciar el GNSS.");
        return -EFAULT;
    }
    return 0;
}

static int gnss_stop(void) {
    return nrf_modem_gnss_stop();
}

// =================================================================
//  LÓGICA DEL MÓDEM Y RED
// =================================================================

static void lte_handler(const struct lte_lc_evt *const evt) {
    if (evt->type == LTE_LC_EVT_NW_REG_STATUS &&
       (evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ||
        evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING)) {
        LOG_INF("Red registrada!");
        k_sem_give(&lte_connected_sem);
    }
}

static int modem_configure_for_ntn(void) {
    int err;
    char cmd_buf[128];

    LOG_INF("Configurando módem para NTN...");
    
    // Usar AT%XTIME para proveer asistencia de localización
    snprintf(cmd_buf, sizeof(cmd_buf), "AT%%XTIME=1,%.6f,%.6f,%.0f,%lld",
             last_gps_data.latitude,
             last_gps_data.longitude,
             last_gps_data.altitude,
             k_uptime_get() / 1000);
    err = nrf_modem_at_printf("%s", cmd_buf);
    if (err) {
        LOG_ERR("Fallo al establecer la localización: %d", err);
        return err;
    }

    // Configurar PLMN para Sateliot
    err = nrf_modem_at_printf("AT+COPS=1,2,\"%s\"", SATELIOT_PLMN);
    if (err) {
        LOG_ERR("Fallo al establecer el PLMN: %d", err);
        return err;
    }
    return 0;
}

static int format_telemetry_data(char *buffer, size_t buffer_size) {
    int ret = snprintf(buffer, buffer_size,
        "{\"ts\":%lld,\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f,\"sats\":%d}",
        k_uptime_get(),
        last_gps_data.latitude,
        last_gps_data.longitude,
        last_gps_data.altitude,
        last_gps_data.sv_count
    );
    return (ret > 0 && ret < buffer_size) ? 0 : -ENOMEM;
}

static int robust_data_send(const char *payload) {
    int err = -1, retry_count = 0, sock;
    const int max_retries = 3;
    struct sockaddr_in server_addr;

    while (retry_count < max_retries && err != 0) {
        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0) {
            LOG_ERR("Fallo al crear socket, intento %d/%d", retry_count + 1, max_retries);
            retry_count++;
            k_sleep(K_SECONDS(10));
            continue;
        }

        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(SERVER_PORT);
        inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr);

        err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&server_addr, sizeof(server_addr));
        close(sock);

        if (err < 0) {
            LOG_ERR("Fallo al enviar datos, intento %d/%d: %d", retry_count + 1, max_retries, -errno);
            retry_count++;
            k_sleep(K_SECONDS(5));
        } else {
            LOG_INF("Datos enviados con éxito en el intento %d.", retry_count + 1);
            return 0;
        }
    }
    LOG_ERR("Todos los intentos de envío fallaron.");
    return -EIO;
}

// =================================================================
//  FUNCIÓN PRINCIPAL
// =================================================================
int main(void) {
    int err;
    struct satellite_info sat_info;

    LOG_INF("Iniciando firmware NTN optimizado v3.0...");
    
    err = setup_watchdog();
    if (err) {
        LOG_ERR("FALLO CRÍTICO: No se pudo iniciar el watchdog.");
        // Bucle infinito para indicar un fallo irrecuperable
        while(1) { k_sleep(K_FOREVER); }
    }

    err = lte_lc_init_and_connect_async(lte_handler);
    if (err) {
        LOG_ERR("Fallo al inicializar el módem: %d", err);
        set_state(STATE_ERROR);
    }

    err = gnss_init_and_start();
    if (err) {
        LOG_ERR("Fallo al inicializar GNSS: %d", err);
        set_state(STATE_ERROR);
    }
    
    err = configure_power_management();
    if (err) {
        LOG_WRN("No se pudo configurar la gestión de energía.");
    }

    set_state(STATE_IDLE);

    while (1) {
        wdt_feed(wdt_dev, wdt_channel_id); // Alimenta al perro guardián en cada ciclo
        
        switch (current_state) {
            case STATE_IDLE:
                if (CURRENT_INTEGRATION_PHASE == PHASE_NTN_TESTING) {
                    calculate_satellite_pass(&sat_info, 0, 0); // Usar lat/lon real
                    int64_t sleep_ms = sat_info.next_pass_start - k_uptime_get();
                    if (sleep_ms > 0) {
                        LOG_INF("Modo NTN: Durmiendo por %llds.", sleep_ms / 1000);
                        k_sleep(K_MSEC(sleep_ms));
                    }
                } else {
                    LOG_INF("Modo TN: Esperando 60s.");
                    k_sleep(K_SECONDS(60));
                }
                set_state(STATE_GETTING_GPS_FIX);
                break;

            case STATE_GETTING_GPS_FIX:
                LOG_INF("Esperando fix de GNSS...");
                k_sem_reset(&gps_fix_sem);
                err = k_sem_take(&gps_fix_sem, K_SECONDS(180)); // Timeout de 3 min
                if (err) {
                    LOG_WRN("No se obtuvo fix de GNSS.");
                    set_state(STATE_IDLE);
                } else {
                    set_state(STATE_ATTEMPTING_CONNECTION);
                }
                break;

            case STATE_ATTEMPTING_CONNECTION:
                LOG_INF("Intentando conectar a la red...");
                if (CURRENT_INTEGRATION_PHASE == PHASE_NTN_TESTING) {
                    modem_configure_for_ntn();
                }
                
                lte_lc_connect_async(lte_handler);

                err = k_sem_take(&lte_connected_sem, K_MINUTES(10));
                if (err) {
                    LOG_WRN("Timeout de conexión de red.");
                    lte_lc_offline();
                    set_state(STATE_IDle);
                } else {
                    set_state(STATE_SENDING_DATA);
                }
                break;

            case STATE_SENDING_DATA:
                if (format_telemetry_data(payload_buffer, PAYLOAD_BUFFER_SIZE) == 0) {
                    robust_data_send(payload_buffer);
                } else {
                    LOG_ERR("Fallo al formatear el payload.");
                }
                lte_lc_offline();
                LOG_INF("Ciclo completado.");
                set_state(STATE_IDLE);
                break;

            case STATE_ERROR:
                LOG_ERR("Estado de error. El watchdog reiniciará el sistema si se bloquea.");
                k_sleep(K_MINUTES(5));
                set_state(STATE_IDLE);
                break;
        }
        k_sleep(K_MSEC(500));
    }
    return 0;
}
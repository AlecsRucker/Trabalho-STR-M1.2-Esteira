// ============================================================
// ESTEIRA INDUSTRIAL - ESP32 + FreeRTOS
// Sistemas em Tempo Real
//
// Tasks:
// ENC_SENSE   -> periódica a cada 5 ms
// SPD_CTRL    -> acionada pela ENC_SENSE
// SORT_ACT    -> acionada pelo Touch B
// SAFETY_TASK -> acionada pelo Touch D
//
// Touch B (T7 / GPIO27) -> detecção de objeto
// Touch C (T8 / GPIO33) -> solicitação HMI
// Touch D (T9 / GPIO32) -> parada de emergência
// ============================================================

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_timer.h"
#include "esp_err.h"
#include "driver/touch_sens.h"


// ============================================================
// CONFIGURAÇÕES
// ============================================================

#define TAG "ESTEIRA"

// Canais Touch
#define TP_OBJ      7       // Touch B
#define TP_HMI      8       // Touch C
#define TP_ESTOP    9       // Touch D

// Período da ENC_SENSE
#define ENC_T_MS    5

// Deadlines em microssegundos
#define D_ENC_US     5000
#define D_CTRL_US   10000
#define D_SORT_US   10000
#define D_SAFE_US    5000

// Prioridades CUSTOM:
// Segurança recebe a maior prioridade.
#define PRIO_ESTOP  5
#define PRIO_ENC    4
#define PRIO_CTRL   3
#define PRIO_SORT   3

#define STK         3072

// Número de canais utilizados
#define TOUCH_COUNT 3


// ============================================================
// HANDLES DAS TASKS
// ============================================================

static TaskHandle_t hENC   = NULL;
static TaskHandle_t hCTRL  = NULL;
static TaskHandle_t hSORT  = NULL;
static TaskHandle_t hSAFE  = NULL;


// ============================================================
// ESTRUTURAS DE EVENTOS
// ============================================================

// Evento enviado pelo Touch B para SORT_ACT
typedef struct {
    int64_t t_evt_us;
} sort_evt_t;

// Evento enviado pelo Touch D para SAFETY_TASK
typedef struct {
    int64_t t_evt_us;
} safety_evt_t;


// ============================================================
// FILAS
// ============================================================

static QueueHandle_t qSort   = NULL;
static QueueHandle_t qSafety = NULL;


// ============================================================
// FLAGS
// ============================================================

// Touch C apenas solicita a impressão da HMI.
// SPD_CTRL atende essa solicitação em trecho não crítico.
static volatile bool hmi_requested = false;


// ============================================================
// TOUCH HANDLES
// ============================================================

static touch_sensor_handle_t touch_sens = NULL;

static touch_channel_handle_t touch_obj   = NULL;
static touch_channel_handle_t touch_hmi   = NULL;
static touch_channel_handle_t touch_estop = NULL;


// ============================================================
// ESTADO SIMULADO DA ESTEIRA
// ============================================================

typedef struct {
    float rpm;
    float pos_mm;
    float set_rpm;
    float pwm;
} belt_state_t;

static belt_state_t g_belt = {
    .rpm = 0.0f,
    .pos_mm = 0.0f,
    .set_rpm = 120.0f,
    .pwm = 0.0f
};


// ============================================================
// ESTATÍSTICAS
// ============================================================

typedef struct {

    uint32_t executions;
    uint32_t misses;

    int64_t wcet_us;
    int64_t max_latency_us;

} task_stats_t;


static task_stats_t stat_enc  = {0};
static task_stats_t stat_ctrl = {0};
static task_stats_t stat_sort = {0};
static task_stats_t stat_safe = {0};


// ============================================================
// CARGA DE CPU SIMULADA
// ============================================================

static inline void cpu_tight_loop_us(uint32_t us)
{
    int64_t start = esp_timer_get_time();

    while ((esp_timer_get_time() - start) < us) {
        __asm__ __volatile__("nop");
    }
}


// ============================================================
// ENC_SENSE
// Periódica: T = 5 ms
// Estima RPM e posição
// ============================================================

static void task_enc_sense(void *arg)
{
    TickType_t next = xTaskGetTickCount();

    const TickType_t period =
        pdMS_TO_TICKS(ENC_T_MS);

    for (;;) {

        int64_t t_start =
            esp_timer_get_time();

        // Simulação da dinâmica da esteira
        float error =
            g_belt.set_rpm - g_belt.rpm;

        g_belt.rpm +=
            0.05f * error;

        g_belt.pos_mm +=
            (g_belt.rpm / 60.0f) *
            (ENC_T_MS / 1000.0f) *
            100.0f;

        // WCET simulado aproximado
        cpu_tight_loop_us(700);

        int64_t t_end =
            esp_timer_get_time();

        int64_t exec =
            t_end - t_start;

        stat_enc.executions++;

        if (exec > stat_enc.wcet_us)
            stat_enc.wcet_us = exec;

        if (exec > D_ENC_US)
            stat_enc.misses++;

        // Acorda a tarefa de controle
        if (hCTRL != NULL) {

            // Envia o instante da amostra para SPD_CTRL
            xTaskNotify(
                hCTRL,
                (uint32_t)t_start,
                eSetValueWithOverwrite
            );
        }

        vTaskDelayUntil(
            &next,
            period
        );
    }
}


// ============================================================
// SPD_CTRL
// Encadeada pela ENC_SENSE
// Controle PI simulado
// ============================================================

static void task_spd_ctrl(void *arg)
{
    float kp = 0.4f;
    float ki = 0.1f;
    float integral = 0.0f;

    for (;;) {

        uint32_t notification_value;

        xTaskNotifyWait(
            0,
            UINT32_MAX,
            &notification_value,
            portMAX_DELAY
        );

        int64_t t_start =
            esp_timer_get_time();

        float error =
            g_belt.set_rpm -
            g_belt.rpm;

        integral +=
            error *
            (ENC_T_MS / 1000.0f);

        float u =
            kp * error +
            ki * integral;

        // PWM simulado
        g_belt.pwm = u;

        // Limita PWM simulado
        if (g_belt.pwm > 100.0f)
            g_belt.pwm = 100.0f;

        if (g_belt.pwm < 0.0f)
            g_belt.pwm = 0.0f;

        cpu_tight_loop_us(1200);

        int64_t t_end =
            esp_timer_get_time();

        int64_t exec =
            t_end - t_start;

        stat_ctrl.executions++;

        if (exec > stat_ctrl.wcet_us)
            stat_ctrl.wcet_us = exec;

        // Deadline aproximada da própria execução
        if (exec > D_CTRL_US)
            stat_ctrl.misses++;

        // Touch C solicita HMI
        if (hmi_requested) {

            hmi_requested = false;

            printf(
                "%s HMI | RPM=%.1f | SET=%.1f | "
                "POS=%.1f mm | PWM=%.1f\n",
                TAG,
                g_belt.rpm,
                g_belt.set_rpm,
                g_belt.pos_mm,
                g_belt.pwm
            );
        }
    }
}


// ============================================================
// SORT_ACT
// Evento Touch B
// Deadline = 10 ms
// ============================================================

static void task_sort_act(void *arg)
{
    sort_evt_t ev;

    for (;;) {

        if (xQueueReceive(
                qSort,
                &ev,
                portMAX_DELAY) == pdTRUE) {

            int64_t t_start =
                esp_timer_get_time();

            // Latência Touch -> início da task
            int64_t latency =
                t_start - ev.t_evt_us;

            // Simula acionamento do solenoide
            cpu_tight_loop_us(700);

            int64_t t_end =
                esp_timer_get_time();

            int64_t exec =
                t_end - t_start;

            int64_t response =
                t_end - ev.t_evt_us;

            stat_sort.executions++;

            if (exec > stat_sort.wcet_us)
                stat_sort.wcet_us = exec;

            if (latency > stat_sort.max_latency_us)
                stat_sort.max_latency_us = latency;

            if (response > D_SORT_US)
                stat_sort.misses++;

            printf(
                "%s SORT | "
                "evento=%lld | inicio=%lld | fim=%lld | "
                "lat=%lld us | exec=%lld us | %s\n",
                TAG,
                (long long)ev.t_evt_us,
                (long long)t_start,
                (long long)t_end,
                (long long)latency,
                (long long)exec,
                (response <= D_SORT_US)
                    ? "DEADLINE OK"
                    : "MISS"
            );
        }
    }
}


// ============================================================
// SAFETY_TASK
// Evento Touch D
// Deadline = 5 ms
// ============================================================

static void task_safety(void *arg)
{
    safety_evt_t ev;

    for (;;) {

        if (xQueueReceive(
                qSafety,
                &ev,
                portMAX_DELAY) == pdTRUE) {

            int64_t t_start =
                esp_timer_get_time();

            int64_t latency =
                t_start - ev.t_evt_us;

            // PARADA DE EMERGÊNCIA
            g_belt.set_rpm = 0.0f;
            g_belt.pwm = 0.0f;

            // Simula processamento de segurança
            cpu_tight_loop_us(900);

            int64_t t_end =
                esp_timer_get_time();

            int64_t exec =
                t_end - t_start;

            int64_t response =
                t_end - ev.t_evt_us;

            stat_safe.executions++;

            if (exec > stat_safe.wcet_us)
                stat_safe.wcet_us = exec;

            if (latency > stat_safe.max_latency_us)
                stat_safe.max_latency_us = latency;

            if (response > D_SAFE_US)
                stat_safe.misses++;

            printf(
                "%s E-STOP | "
                "evento=%lld | inicio=%lld | fim=%lld | "
                "lat=%lld us | exec=%lld us | %s\n",
                TAG,
                (long long)ev.t_evt_us,
                (long long)t_start,
                (long long)t_end,
                (long long)latency,
                (long long)exec,
                (response <= D_SAFE_US)
                    ? "DEADLINE OK"
                    : "MISS"
            );
        }
    }
}


// ============================================================
// CALLBACK TOUCH
//
// É chamada quando um canal Touch fica ativo.
// O callback identifica B, C ou D.
// ============================================================

static bool touch_on_active(
    touch_sensor_handle_t sens_handle,
    const touch_active_event_data_t *event,
    void *user_ctx)
{
    BaseType_t task_woken =
        pdFALSE;

    int64_t now =
        esp_timer_get_time();

    // --------------------------------------------------------
    // TOUCH B -> objeto -> SORT_ACT
    // --------------------------------------------------------

    if (event->chan_id == TP_OBJ) {

        sort_evt_t ev = {
            .t_evt_us = now
        };

        xQueueSendFromISR(
            qSort,
            &ev,
            &task_woken
        );
    }

    // --------------------------------------------------------
    // TOUCH C -> HMI
    // --------------------------------------------------------

    else if (event->chan_id == TP_HMI) {

        hmi_requested = true;
    }

    // --------------------------------------------------------
    // TOUCH D -> E-STOP
    // --------------------------------------------------------

    else if (event->chan_id == TP_ESTOP) {

        safety_evt_t ev = {
            .t_evt_us = now
        };

        xQueueSendFromISR(
            qSafety,
            &ev,
            &task_woken
        );
    }

    return (task_woken == pdTRUE);
}


// ============================================================
// CALIBRAÇÃO DOS TOUCH
// ============================================================

static void touch_calibrate(void)
{
    touch_channel_handle_t channels[TOUCH_COUNT] = {
        touch_obj,
        touch_hmi,
        touch_estop
    };

    const char *names[TOUCH_COUNT] = {
        "B",
        "C",
        "D"
    };

    // Habilita para fazer as leituras iniciais
    ESP_ERROR_CHECK(
        touch_sensor_enable(
            touch_sens
        )
    );

    // Três varreduras para estabilizar as leituras
    for (int i = 0; i < 3; i++) {

        ESP_ERROR_CHECK(
            touch_sensor_trigger_oneshot_scanning(
                touch_sens,
                2000
            )
        );
    }

    // Desabilita antes de reconfigurar
    ESP_ERROR_CHECK(
        touch_sensor_disable(
            touch_sens
        )
    );

    // Calibra individualmente B, C e D
    for (int i = 0; i < TOUCH_COUNT; i++) {

        uint32_t reference[1] = {0};

        ESP_ERROR_CHECK(
            touch_channel_read_data(
                channels[i],
                TOUCH_CHAN_DATA_TYPE_SMOOTH,
                reference
            )
        );

        // ESP32 Touch V1:
        // threshold absoluto = referência * (1 - coeficiente)
        touch_channel_config_t cfg = {

            .abs_active_thresh = {
                (uint32_t)(
                    reference[0] *
                    (1.0f - 0.015f)
                )
            },

            .charge_speed =
                TOUCH_CHARGE_SPEED_7,

            .init_charge_volt =
                TOUCH_INIT_CHARGE_VOLT_DEFAULT,

            .group =
                TOUCH_CHAN_TRIG_GROUP_BOTH,
        };

        ESP_ERROR_CHECK(
            touch_sensor_reconfig_channel(
                channels[i],
                &cfg
            )
        );

        printf(
            "%s Touch %s calibrado | "
            "referencia=%lu | threshold=%lu\n",
            TAG,
            names[i],
            (unsigned long)reference[0],
            (unsigned long)
                cfg.abs_active_thresh[0]
        );
    }
}


// ============================================================
// INICIALIZAÇÃO DOS TOUCH
// ============================================================

static void touch_init(void)
{
    // Configuração de amostragem para ESP32 Touch V1
    touch_sensor_sample_config_t sample_cfg =
        TOUCH_SENSOR_V1_DEFAULT_SAMPLE_CONFIG(
            1.0,
            TOUCH_VOLT_LIM_L_0V5,
            TOUCH_VOLT_LIM_H_2V7
        );

    touch_sensor_config_t sens_cfg =
        TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(
            1,
            &sample_cfg
        );

    // Cria controlador Touch
    ESP_ERROR_CHECK(
        touch_sensor_new_controller(
            &sens_cfg,
            &touch_sens
        )
    );

    // Configuração inicial dos canais
    touch_channel_config_t chan_cfg = {

        .abs_active_thresh = {1000},

        .charge_speed =
            TOUCH_CHARGE_SPEED_7,

        .init_charge_volt =
            TOUCH_INIT_CHARGE_VOLT_DEFAULT,

        .group =
            TOUCH_CHAN_TRIG_GROUP_BOTH,
    };

    // --------------------------------------------------------
    // Touch B
    // --------------------------------------------------------

    ESP_ERROR_CHECK(
        touch_sensor_new_channel(
            touch_sens,
            TP_OBJ,
            &chan_cfg,
            &touch_obj
        )
    );

    // --------------------------------------------------------
    // Touch C
    // --------------------------------------------------------

    ESP_ERROR_CHECK(
        touch_sensor_new_channel(
            touch_sens,
            TP_HMI,
            &chan_cfg,
            &touch_hmi
        )
    );

    // --------------------------------------------------------
    // Touch D
    // --------------------------------------------------------

    ESP_ERROR_CHECK(
        touch_sensor_new_channel(
            touch_sens,
            TP_ESTOP,
            &chan_cfg,
            &touch_estop
        )
    );

    // Touch V1 utiliza filtro por software
    touch_sensor_filter_config_t filter_cfg =
        TOUCH_SENSOR_DEFAULT_FILTER_CONFIG();

    ESP_ERROR_CHECK(
        touch_sensor_config_filter(
            touch_sens,
            &filter_cfg
        )
    );

    // Calibração inicial
    touch_calibrate();

    // Registra callback
    touch_event_callbacks_t callbacks = {
        .on_active = touch_on_active,
    };

    ESP_ERROR_CHECK(
        touch_sensor_register_callbacks(
            touch_sens,
            &callbacks,
            NULL
        )
    );

    // Habilita controlador
    ESP_ERROR_CHECK(
        touch_sensor_enable(
            touch_sens
        )
    );

    // Inicia varredura contínua
    ESP_ERROR_CHECK(
        touch_sensor_start_continuous_scanning(
            touch_sens
        )
    );

    printf(
        "%s Touch B/C/D inicializados.\n",
        TAG
    );
}


// ============================================================
// TASK DE RELATÓRIO DE ESTATÍSTICAS
//
// Não participa do controle.
// Apenas imprime dados periodicamente para coleta experimental.
// ============================================================

static void task_report(void *arg)
{
    for (;;) {

        // Evita interferência constante nos testes
        vTaskDelay(
            pdMS_TO_TICKS(5000)
        );

        printf("\n");
        printf("========== ESTATISTICAS ==========\n");

        printf(
            "ENC  | exec=%lu | miss=%lu | WCET=%lld us\n",
            (unsigned long)stat_enc.executions,
            (unsigned long)stat_enc.misses,
            (long long)stat_enc.wcet_us
        );

        printf(
            "CTRL | exec=%lu | miss=%lu | WCET=%lld us\n",
            (unsigned long)stat_ctrl.executions,
            (unsigned long)stat_ctrl.misses,
            (long long)stat_ctrl.wcet_us
        );

        printf(
            "SORT | exec=%lu | miss=%lu | WCET=%lld us | "
            "lat_max=%lld us\n",
            (unsigned long)stat_sort.executions,
            (unsigned long)stat_sort.misses,
            (long long)stat_sort.wcet_us,
            (long long)stat_sort.max_latency_us
        );

        printf(
            "SAFE | exec=%lu | miss=%lu | WCET=%lld us | "
            "lat_max=%lld us\n",
            (unsigned long)stat_safe.executions,
            (unsigned long)stat_safe.misses,
            (long long)stat_safe.wcet_us,
            (long long)stat_safe.max_latency_us
        );

        printf("==================================\n\n");
    }
}


// ============================================================
// APP_MAIN
// ============================================================

void app_main(void)
{
    // --------------------------------------------------------
    // Criação das filas
    // --------------------------------------------------------

    qSort =
        xQueueCreate(
            10,
            sizeof(sort_evt_t)
        );

    qSafety =
        xQueueCreate(
            10,
            sizeof(safety_evt_t)
        );

    if (qSort == NULL ||
        qSafety == NULL) {

        printf(
            "%s ERRO ao criar filas.\n",
            TAG
        );

        return;
    }

    // --------------------------------------------------------
    // Inicializa Touch antes de criar as tasks
    // --------------------------------------------------------

    touch_init();

    // --------------------------------------------------------
    // Criação das tasks
    //
    // Todas no Core 0.
    // Depois também configuraremos UNICORE pelo menuconfig.
    // --------------------------------------------------------

    xTaskCreatePinnedToCore(
        task_safety,
        "SAFETY",
        STK,
        NULL,
        PRIO_ESTOP,
        &hSAFE,
        0
    );

    xTaskCreatePinnedToCore(
        task_spd_ctrl,
        "SPD_CTRL",
        STK,
        NULL,
        PRIO_CTRL,
        &hCTRL,
        0
    );

    xTaskCreatePinnedToCore(
        task_sort_act,
        "SORT_ACT",
        STK,
        NULL,
        PRIO_SORT,
        &hSORT,
        0
    );

    xTaskCreatePinnedToCore(
        task_enc_sense,
        "ENC_SENSE",
        STK,
        NULL,
        PRIO_ENC,
        &hENC,
        0
    );

    // Task apenas para impressão das estatísticas
    xTaskCreatePinnedToCore(
        task_report,
        "REPORT",
        3072,
        NULL,
        1,
        NULL,
        0
    );

    printf(
        "%s Sistema iniciado.\n",
        TAG
    );
}
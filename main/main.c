// ============================================================
// ESTEIRA INDUSTRIAL - ESP32 + FreeRTOS
// Sistemas em Tempo Real
//
// Tasks principais:
// ENC_SENSE   -> executada periodicamente a cada 5 ms
// SPD_CTRL    -> acionada pela ENC_SENSE para controle de velocidade
// SORT_ACT    -> acionada pelo Touch B para separação de objetos
// SAFETY_TASK -> acionada pelo Touch D para parada de emergência
//
// Sensores Touch:
// Touch B (T7 / GPIO27) -> detecção de objeto
// Touch C (T8 / GPIO33) -> solicitação da HMI
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
// CONFIGURAÇÕES GERAIS
// ============================================================

// Identificação utilizada nas mensagens exibidas no terminal
#define TAG "ESTEIRA"

// Canais Touch utilizados no projeto
#define TP_OBJ      7       // Touch B - detecção de objeto
#define TP_HMI      8       // Touch C - solicitação da HMI
#define TP_ESTOP    9       // Touch D - parada de emergência

// Período da tarefa periódica ENC_SENSE
#define ENC_T_MS    5

// Deadlines das tarefas em microssegundos
#define D_ENC_US     5000
#define D_CTRL_US   10000
#define D_SORT_US   10000
#define D_SAFE_US    5000

// Prioridades utilizadas no escalonamento customizado
// A tarefa de segurança recebe a maior prioridade
#define PRIO_ESTOP  5
#define PRIO_ENC    4
#define PRIO_CTRL   3
#define PRIO_SORT   3

// Tamanho da pilha utilizado pelas tasks principais
#define STK         3072

// Quantidade de canais Touch utilizados
#define TOUCH_COUNT 3


// ============================================================
// HANDLES DAS TASKS
//
// Os handles permitem referenciar as tasks depois da criação.
// Por exemplo, ENC_SENSE utiliza hCTRL para notificar SPD_CTRL.
// ============================================================

static TaskHandle_t hENC   = NULL;
static TaskHandle_t hCTRL  = NULL;
static TaskHandle_t hSORT  = NULL;
static TaskHandle_t hSAFE  = NULL;


// ============================================================
// ESTRUTURAS DOS EVENTOS
//
// Os eventos armazenam o instante em que um Touch foi detectado.
// Esse timestamp permite calcular posteriormente a latência entre
// a ocorrência do evento e o início da task correspondente.
// ============================================================

// Evento de detecção de objeto gerado pelo Touch B
typedef struct {
    int64_t t_evt_us;       // Instante em que o objeto foi detectado
} sort_evt_t;

// Evento de parada de emergência gerado pelo Touch D
typedef struct {
    int64_t t_evt_us;       // Instante em que o E-stop foi detectado
} safety_evt_t;


// ============================================================
// FILAS DE COMUNICAÇÃO
//
// As filas transferem os eventos detectados pelos sensores Touch
// para as tasks responsáveis pelo processamento.
// ============================================================

// Fila utilizada na comunicação Touch B -> SORT_ACT
static QueueHandle_t qSort   = NULL;

// Fila utilizada na comunicação Touch D -> SAFETY_TASK
static QueueHandle_t qSafety = NULL;


// ============================================================
// FLAG DA HMI
// ============================================================

// Indica que o Touch C solicitou a exibição dos dados da HMI.
// A impressão será realizada pela SPD_CTRL.
static volatile bool hmi_requested = false;


// ============================================================
// HANDLES DOS SENSORES TOUCH
// ============================================================

// Handle do controlador geral do periférico Touch
static touch_sensor_handle_t touch_sens = NULL;

// Handles individuais dos três canais utilizados
static touch_channel_handle_t touch_obj   = NULL;
static touch_channel_handle_t touch_hmi   = NULL;
static touch_channel_handle_t touch_estop = NULL;


// ============================================================
// ESTADO SIMULADO DA ESTEIRA
//
// Estrutura utilizada para representar as principais grandezas
// da esteira durante a simulação.
// ============================================================

typedef struct {

    // Velocidade atual simulada da esteira
    float rpm;

    // Posição acumulada simulada
    float pos_mm;

    // Velocidade de referência desejada
    float set_rpm;

    // Saída simulada do controlador
    float pwm;

} belt_state_t;


// Define o estado inicial da esteira
static belt_state_t g_belt = {

    // A esteira começa parada
    .rpm = 0.0f,

    // Posição inicial
    .pos_mm = 0.0f,

    // Referência inicial de velocidade
    .set_rpm = 120.0f,

    // PWM inicialmente zerado
    .pwm = 0.0f
};


// ============================================================
// ESTATÍSTICAS DAS TASKS
//
// Estrutura utilizada para armazenar os resultados necessários
// para a análise temporal dos experimentos.
// ============================================================

typedef struct {

    // Quantidade total de execuções da task
    uint32_t executions;

    // Quantidade de vezes que o deadline foi perdido
    uint32_t misses;

    // Maior tempo de execução observado
    int64_t wcet_us;

    // Maior latência observada
    int64_t max_latency_us;

} task_stats_t;


// Estatísticas individuais das quatro tasks principais
static task_stats_t stat_enc  = {0};
static task_stats_t stat_ctrl = {0};
static task_stats_t stat_sort = {0};
static task_stats_t stat_safe = {0};


// ============================================================
// SIMULAÇÃO DE CARGA DE CPU
//
// Mantém o processador ocupado durante aproximadamente o número
// de microssegundos informado em "us". Isso permite representar
// o custo computacional das operações simuladas da esteira.
// ============================================================

static inline void cpu_tight_loop_us(uint32_t us)
{
    // Registra o instante inicial da simulação
    int64_t start = esp_timer_get_time();

    // Mantém a CPU ocupada até atingir o tempo solicitado
    while ((esp_timer_get_time() - start) < us) {

        // NOP não altera o estado do programa, mas consome
        // ciclos do processador durante o laço
        __asm__ __volatile__("nop");
    }
}


// ============================================================
// ENC_SENSE
//
// Task periódica que representa a leitura do encoder da esteira.
// É executada a cada 5 ms, atualiza a velocidade e a posição
// simuladas e, ao terminar, notifica SPD_CTRL para executar uma
// nova atualização do controle de velocidade.
// ============================================================

static void task_enc_sense(void *arg)
{
    // Obtém o tick atual para controlar a periodicidade
    TickType_t next = xTaskGetTickCount();

    // Converte o período definido em milissegundos para ticks
    const TickType_t period =
        pdMS_TO_TICKS(ENC_T_MS);

    for (;;) {

        // Registra o instante de início desta execução
        int64_t t_start =
            esp_timer_get_time();

        // Calcula a diferença entre a velocidade desejada
        // e a velocidade atual da esteira
        float error =
            g_belt.set_rpm - g_belt.rpm;

        // Simula a resposta dinâmica da velocidade.
        // A RPM atual se aproxima gradualmente do setpoint.
        g_belt.rpm +=
            0.05f * error;

        // Atualiza a posição percorrida utilizando a velocidade
        // atual e o intervalo de 5 ms entre as leituras
        g_belt.pos_mm +=
            (g_belt.rpm / 60.0f) *
            (ENC_T_MS / 1000.0f) *
            100.0f;

        // Simula aproximadamente 700 us de processamento
        cpu_tight_loop_us(700);

        // Registra o instante em que o processamento terminou
        int64_t t_end =
            esp_timer_get_time();

        // Calcula o tempo gasto nesta execução
        int64_t exec =
            t_end - t_start;

        // Contabiliza mais uma execução da ENC_SENSE
        stat_enc.executions++;

        // Atualiza o maior tempo de execução observado
        if (exec > stat_enc.wcet_us)
            stat_enc.wcet_us = exec;

        // Verifica se o tempo de execução ultrapassou
        // o deadline definido para ENC_SENSE
        if (exec > D_ENC_US)
            stat_enc.misses++;

        // Após produzir uma nova leitura, acorda SPD_CTRL
        if (hCTRL != NULL) {

            // Envia uma notificação para SPD_CTRL contendo
            // também o instante em que esta leitura começou
            xTaskNotify(
                hCTRL,
                (uint32_t)t_start,
                eSetValueWithOverwrite
            );
        }

        // Aguarda o próximo instante periódico.
        // vTaskDelayUntil mantém a periodicidade de 5 ms.
        vTaskDelayUntil(
            &next,
            period
        );
    }
}


// ============================================================
// SPD_CTRL
//
// Task responsável pelo controle PI da velocidade da esteira.
// Permanece bloqueada até receber uma notificação da ENC_SENSE.
// Também atende as solicitações de HMI realizadas pelo Touch C.
// ============================================================

static void task_spd_ctrl(void *arg)
{
    // Ganho proporcional do controlador PI
    float kp = 0.4f;

    // Ganho integral do controlador PI
    float ki = 0.1f;

    // Acumula o erro utilizado pela parcela integral
    float integral = 0.0f;

    for (;;) {

        // Variável que recebe o valor enviado pela ENC_SENSE
        uint32_t notification_value;

        // Aguarda indefinidamente uma nova notificação.
        // Enquanto não houver nova amostra, a task permanece bloqueada.
        xTaskNotifyWait(
            0,
            UINT32_MAX,
            &notification_value,
            portMAX_DELAY
        );

        // Registra o instante de início do controle
        int64_t t_start =
            esp_timer_get_time();

        // Calcula o erro entre a referência e a velocidade atual
        float error =
            g_belt.set_rpm -
            g_belt.rpm;

        // Atualiza a parcela integral utilizando o período de 5 ms
        integral +=
            error *
            (ENC_T_MS / 1000.0f);

        // Calcula a saída do controlador PI
        float u =
            kp * error +
            ki * integral;

        // Utiliza a saída calculada como PWM simulado
        g_belt.pwm = u;

        // Limita o PWM ao máximo de 100%
        if (g_belt.pwm > 100.0f)
            g_belt.pwm = 100.0f;

        // Impede valores negativos de PWM
        if (g_belt.pwm < 0.0f)
            g_belt.pwm = 0.0f;

        // Simula aproximadamente 1,2 ms de processamento
        cpu_tight_loop_us(1200);

        // Registra o final da execução
        int64_t t_end =
            esp_timer_get_time();

        // Calcula o tempo gasto pelo controle
        int64_t exec =
            t_end - t_start;

        // Contabiliza mais uma execução da SPD_CTRL
        stat_ctrl.executions++;

        // Atualiza o maior tempo de execução observado
        if (exec > stat_ctrl.wcet_us)
            stat_ctrl.wcet_us = exec;

        // Verifica se houve perda do deadline
        if (exec > D_CTRL_US)
            stat_ctrl.misses++;

        // Verifica se o Touch C solicitou a exibição da HMI
        if (hmi_requested) {

            // Limpa a solicitação após atendê-la
            hmi_requested = false;

            // Exibe as principais informações da esteira
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
//
// Task responsável pela separação dos objetos na esteira.
// Permanece bloqueada aguardando eventos enviados pelo Touch B.
// Quando um objeto é detectado, simula o acionamento do desviador
// e registra as métricas temporais da operação.
//
// Deadline: 10 ms.
// ============================================================

static void task_sort_act(void *arg)
{
    // Estrutura onde será armazenado o evento recebido
    sort_evt_t ev;

    for (;;) {

        // Aguarda indefinidamente um evento do Touch B.
        // Enquanto não houver objeto, a task permanece bloqueada.
        if (xQueueReceive(
                qSort,
                &ev,
                portMAX_DELAY) == pdTRUE) {

            // Registra o instante em que a task começou a executar
            int64_t t_start =
                esp_timer_get_time();

            // Calcula a latência entre a detecção do objeto
            // e o início efetivo da SORT_ACT
            int64_t latency =
                t_start - ev.t_evt_us;

            // Simula aproximadamente 700 us necessários
            // para acionar o desviador da esteira
            cpu_tight_loop_us(700);

            // Registra o instante em que a operação terminou
            int64_t t_end =
                esp_timer_get_time();

            // Calcula somente o tempo de execução da task
            int64_t exec =
                t_end - t_start;

            // Calcula o tempo total entre o evento do Touch B
            // e a conclusão da resposta do sistema
            int64_t response =
                t_end - ev.t_evt_us;

            // Contabiliza uma nova execução
            stat_sort.executions++;

            // Atualiza o maior tempo de execução observado
            if (exec > stat_sort.wcet_us)
                stat_sort.wcet_us = exec;

            // Atualiza a maior latência observada
            if (latency > stat_sort.max_latency_us)
                stat_sort.max_latency_us = latency;

            // Verifica se o tempo total de resposta
            // ultrapassou o deadline de 10 ms
            if (response > D_SORT_US)
                stat_sort.misses++;

            // Exibe os tempos medidos e informa se
            // o deadline foi atendido ou perdido
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
//
// Task responsável pela parada de emergência da esteira.
// Permanece bloqueada aguardando eventos enviados pelo Touch D.
// Quando acionada, zera a referência de velocidade e o PWM,
// simulando uma parada imediata do sistema.
//
// Deadline: 5 ms.
// ============================================================

static void task_safety(void *arg)
{
    // Estrutura utilizada para receber o evento de emergência
    safety_evt_t ev;

    for (;;) {

        // Aguarda indefinidamente um evento do Touch D
        if (xQueueReceive(
                qSafety,
                &ev,
                portMAX_DELAY) == pdTRUE) {

            // Registra o instante em que SAFETY_TASK iniciou
            int64_t t_start =
                esp_timer_get_time();

            // Calcula a latência entre a detecção do E-stop
            // e o início efetivo da task
            int64_t latency =
                t_start - ev.t_evt_us;

            // Executa a parada de emergência
            // zerando referência e PWM
            g_belt.set_rpm = 0.0f;
            g_belt.pwm = 0.0f;

            // Simula aproximadamente 900 us de processamento
            // necessário para a rotina de segurança
            cpu_tight_loop_us(900);

            // Registra o término da operação
            int64_t t_end =
                esp_timer_get_time();

            // Calcula somente o tempo de execução da task
            int64_t exec =
                t_end - t_start;

            // Calcula o tempo total entre o acionamento do
            // E-stop e a conclusão da rotina de segurança
            int64_t response =
                t_end - ev.t_evt_us;

            // Contabiliza mais uma execução
            stat_safe.executions++;

            // Atualiza o maior tempo de execução observado
            if (exec > stat_safe.wcet_us)
                stat_safe.wcet_us = exec;

            // Atualiza a maior latência observada
            if (latency > stat_safe.max_latency_us)
                stat_safe.max_latency_us = latency;

            // Verifica se o tempo total de resposta
            // ultrapassou o deadline de 5 ms
            if (response > D_SAFE_US)
                stat_safe.misses++;

            // Exibe os tempos medidos e informa
            // se o deadline foi cumprido
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
// CALLBACK DOS SENSORES TOUCH
//
// Esta função é chamada quando um dos canais Touch configurados
// detecta uma ativação.
//
// Touch B -> envia um evento para SORT_ACT
// Touch C -> solicita a exibição da HMI
// Touch D -> envia um evento para SAFETY_TASK
// ============================================================

static bool touch_on_active(
    touch_sensor_handle_t sens_handle,
    const touch_active_event_data_t *event,
    void *user_ctx)
{
    // Indica se o evento desbloqueou uma task
    // que deve receber imediatamente o processador
    BaseType_t task_woken =
        pdFALSE;

    // Registra o instante exato em que o Touch foi detectado.
    // Esse valor será usado posteriormente no cálculo da latência.
    int64_t now =
        esp_timer_get_time();


    // --------------------------------------------------------
    // TOUCH B - DETECÇÃO DE OBJETO
    // --------------------------------------------------------

    if (event->chan_id == TP_OBJ) {

        // Cria o evento com o instante da detecção
        sort_evt_t ev = {
            .t_evt_us = now
        };

        // Envia o evento para a fila da SORT_ACT
        xQueueSendFromISR(
            qSort,
            &ev,
            &task_woken
        );
    }


    // --------------------------------------------------------
    // TOUCH C - SOLICITAÇÃO DA HMI
    // --------------------------------------------------------

    else if (event->chan_id == TP_HMI) {

        // Não realiza printf dentro da callback.
        // Apenas sinaliza que SPD_CTRL deverá exibir a HMI.
        hmi_requested = true;
    }


    // --------------------------------------------------------
    // TOUCH D - PARADA DE EMERGÊNCIA
    // --------------------------------------------------------

    else if (event->chan_id == TP_ESTOP) {

        // Cria o evento contendo o instante do E-stop
        safety_evt_t ev = {
            .t_evt_us = now
        };

        // Envia o evento para SAFETY_TASK
        xQueueSendFromISR(
            qSafety,
            &ev,
            &task_woken
        );
    }

    // Informa se uma task de maior prioridade
    // foi desbloqueada durante o tratamento do evento
    return (task_woken == pdTRUE);
}


// ============================================================
// CALIBRAÇÃO DOS SENSORES TOUCH
//
// Realiza leituras iniciais dos Touch B, C e D sem contato
// e utiliza os valores obtidos para calcular automaticamente
// o threshold de ativação de cada canal.
// ============================================================

static void touch_calibrate(void)
{
    // Vetor contendo os três canais que serão calibrados
    touch_channel_handle_t channels[TOUCH_COUNT] = {
        touch_obj,
        touch_hmi,
        touch_estop
    };

    // Nomes utilizados somente para identificação no terminal
    const char *names[TOUCH_COUNT] = {
        "B",
        "C",
        "D"
    };

    // Habilita o controlador para permitir as leituras iniciais
    ESP_ERROR_CHECK(
        touch_sensor_enable(
            touch_sens
        )
    );

    // Realiza três varreduras iniciais para estabilizar
    // os valores obtidos pelos sensores
    for (int i = 0; i < 3; i++) {

        ESP_ERROR_CHECK(
            touch_sensor_trigger_oneshot_scanning(
                touch_sens,
                2000
            )
        );
    }

    // Desabilita temporariamente o controlador
    // antes de alterar as configurações dos canais
    ESP_ERROR_CHECK(
        touch_sensor_disable(
            touch_sens
        )
    );

    // Percorre e calibra individualmente B, C e D
    for (int i = 0; i < TOUCH_COUNT; i++) {

        // Armazena o valor de referência do canal sem toque
        uint32_t reference[1] = {0};

        // Obtém o valor filtrado do respectivo canal
        ESP_ERROR_CHECK(
            touch_channel_read_data(
                channels[i],
                TOUCH_CHAN_DATA_TYPE_SMOOTH,
                reference
            )
        );

        // Configura o threshold em 98,5% do valor de referência.
        // Dessa forma, uma redução suficiente no valor do sensor
        // será interpretada como uma ativação do Touch.
        touch_channel_config_t cfg = {

            .abs_active_thresh = {
                (uint32_t)(
                    reference[0] *
                    (1.0f - 0.015f)
                )
            },

            // Define a velocidade de carga do sensor
            .charge_speed =
                TOUCH_CHARGE_SPEED_7,

            // Utiliza a tensão inicial padrão
            .init_charge_volt =
                TOUCH_INIT_CHARGE_VOLT_DEFAULT,

            // Permite que o canal participe da detecção
            .group =
                TOUCH_CHAN_TRIG_GROUP_BOTH,
        };

        // Atualiza o canal com o threshold calculado
        ESP_ERROR_CHECK(
            touch_sensor_reconfig_channel(
                channels[i],
                &cfg
            )
        );

        // Exibe os valores obtidos durante a calibração
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
// INICIALIZAÇÃO DOS SENSORES TOUCH
//
// Cria o controlador Touch, configura os canais B, C e D,
// aplica o filtro, executa a calibração, registra a callback
// e inicia a leitura contínua dos sensores.
// ============================================================

static void touch_init(void)
{
    // Configuração de amostragem utilizada pelo Touch V1
    // presente no ESP32
    touch_sensor_sample_config_t sample_cfg =
        TOUCH_SENSOR_V1_DEFAULT_SAMPLE_CONFIG(
            1.0,
            TOUCH_VOLT_LIM_L_0V5,
            TOUCH_VOLT_LIM_H_2V7
        );

    // Cria a configuração geral do controlador Touch
    touch_sensor_config_t sens_cfg =
        TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(
            1,
            &sample_cfg
        );

    // Cria o controlador responsável pelo periférico Touch
    ESP_ERROR_CHECK(
        touch_sensor_new_controller(
            &sens_cfg,
            &touch_sens
        )
    );

    // Configuração inicial comum aos três canais.
    // O threshold de 1000 é apenas inicial e será substituído
    // pelo valor calculado durante touch_calibrate().
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
    // CRIAÇÃO DO TOUCH B
    // Responsável pela detecção dos objetos na esteira
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
    // CRIAÇÃO DO TOUCH C
    // Responsável pela solicitação de dados da HMI
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
    // CRIAÇÃO DO TOUCH D
    // Responsável pela parada de emergência
    // --------------------------------------------------------

    ESP_ERROR_CHECK(
        touch_sensor_new_channel(
            touch_sens,
            TP_ESTOP,
            &chan_cfg,
            &touch_estop
        )
    );


    // Cria a configuração padrão do filtro
    touch_sensor_filter_config_t filter_cfg =
        TOUCH_SENSOR_DEFAULT_FILTER_CONFIG();

    // Aplica o filtro ao controlador Touch
    ESP_ERROR_CHECK(
        touch_sensor_config_filter(
            touch_sens,
            &filter_cfg
        )
    );

    // Executa a calibração automática dos três canais
    touch_calibrate();

    // Define a função que será chamada quando
    // algum dos sensores Touch for ativado
    touch_event_callbacks_t callbacks = {
        .on_active = touch_on_active,
    };

    // Registra a callback no controlador Touch
    ESP_ERROR_CHECK(
        touch_sensor_register_callbacks(
            touch_sens,
            &callbacks,
            NULL
        )
    );

    // Habilita novamente o controlador após a configuração
    ESP_ERROR_CHECK(
        touch_sensor_enable(
            touch_sens
        )
    );

    // Inicia a varredura contínua dos sensores
    ESP_ERROR_CHECK(
        touch_sensor_start_continuous_scanning(
            touch_sens
        )
    );

    // Confirma no terminal que os três sensores estão ativos
    printf(
        "%s Touch B/C/D inicializados.\n",
        TAG
    );
}


// ============================================================
// TASK DE RELATÓRIO DE ESTATÍSTICAS
//
// Task auxiliar de baixa prioridade utilizada somente para
// apresentar periodicamente os dados coletados durante os testes.
// A cada 5 segundos são exibidas execuções, misses, WCET e,
// quando aplicável, a maior latência observada.
// ============================================================

static void task_report(void *arg)
{
    for (;;) {

        // Aguarda 5 segundos entre cada relatório
        // para evitar impressões excessivas no terminal
        vTaskDelay(
            pdMS_TO_TICKS(5000)
        );

        // Cabeçalho do relatório
        printf("\n");
        printf("========== ESTATISTICAS ==========\n");

        // Exibe as estatísticas da ENC_SENSE
        printf(
            "ENC  | exec=%lu | miss=%lu | WCET=%lld us\n",
            (unsigned long)stat_enc.executions,
            (unsigned long)stat_enc.misses,
            (long long)stat_enc.wcet_us
        );

        // Exibe as estatísticas da SPD_CTRL
        printf(
            "CTRL | exec=%lu | miss=%lu | WCET=%lld us\n",
            (unsigned long)stat_ctrl.executions,
            (unsigned long)stat_ctrl.misses,
            (long long)stat_ctrl.wcet_us
        );

        // Exibe as estatísticas da SORT_ACT
        printf(
            "SORT | exec=%lu | miss=%lu | WCET=%lld us | "
            "lat_max=%lld us\n",
            (unsigned long)stat_sort.executions,
            (unsigned long)stat_sort.misses,
            (long long)stat_sort.wcet_us,
            (long long)stat_sort.max_latency_us
        );

        // Exibe as estatísticas da SAFETY_TASK
        printf(
            "SAFE | exec=%lu | miss=%lu | WCET=%lld us | "
            "lat_max=%lld us\n",
            (unsigned long)stat_safe.executions,
            (unsigned long)stat_safe.misses,
            (long long)stat_safe.wcet_us,
            (long long)stat_safe.max_latency_us
        );

        // Finaliza o bloco de estatísticas
        printf("==================================\n\n");
    }
}


// ============================================================
// APP_MAIN
//
// Ponto de entrada da aplicação.
// Cria os mecanismos de comunicação, inicializa os sensores
// Touch e cria todas as tasks utilizadas pelo sistema.
// ============================================================

void app_main(void)
{
    // --------------------------------------------------------
    // CRIAÇÃO DAS FILAS
    // --------------------------------------------------------

    // Cria a fila que transporta os eventos do Touch B
    // para a task responsável pela separação de objetos
    qSort =
        xQueueCreate(
            10,
            sizeof(sort_evt_t)
        );

    // Cria a fila que transporta os eventos do Touch D
    // para a task responsável pela parada de emergência
    qSafety =
        xQueueCreate(
            10,
            sizeof(safety_evt_t)
        );

    // Verifica se alguma das filas não pôde ser criada
    if (qSort == NULL ||
        qSafety == NULL) {

        // Informa o erro no terminal
        printf(
            "%s ERRO ao criar filas.\n",
            TAG
        );

        // Interrompe a inicialização caso falte alguma fila
        return;
    }


    // --------------------------------------------------------
    // INICIALIZAÇÃO DOS SENSORES TOUCH
    // --------------------------------------------------------

    // Configura Touch B, C e D antes da criação das tasks
    touch_init();


    // --------------------------------------------------------
    // CRIAÇÃO DAS TASKS
    //
    // Todas são fixadas no Core 0 para reduzir variações
    // provocadas pela execução em núcleos diferentes.
    // --------------------------------------------------------


    // Cria SAFETY_TASK.
    // Recebe a maior prioridade por tratar o E-stop.
    xTaskCreatePinnedToCore(
        task_safety,
        "SAFETY",
        STK,
        NULL,
        PRIO_ESTOP,
        &hSAFE,
        0
    );


    // Cria SPD_CTRL.
    // Essa task será acordada periodicamente pela ENC_SENSE.
    xTaskCreatePinnedToCore(
        task_spd_ctrl,
        "SPD_CTRL",
        STK,
        NULL,
        PRIO_CTRL,
        &hCTRL,
        0
    );


    // Cria SORT_ACT.
    // A task permanecerá bloqueada aguardando eventos do Touch B.
    xTaskCreatePinnedToCore(
        task_sort_act,
        "SORT_ACT",
        STK,
        NULL,
        PRIO_SORT,
        &hSORT,
        0
    );


    // Cria ENC_SENSE.
    // Essa é a task periódica executada a cada 5 ms.
    xTaskCreatePinnedToCore(
        task_enc_sense,
        "ENC_SENSE",
        STK,
        NULL,
        PRIO_ENC,
        &hENC,
        0
    );


    // Cria a task auxiliar de relatório.
    // Possui prioridade baixa para interferir o mínimo possível
    // na execução das tasks principais.
    xTaskCreatePinnedToCore(
        task_report,
        "REPORT",
        3072,
        NULL,
        1,
        NULL,
        0
    );


    // Confirma que a inicialização da aplicação terminou
    printf(
        "%s Sistema iniciado.\n",
        TAG
    );
}
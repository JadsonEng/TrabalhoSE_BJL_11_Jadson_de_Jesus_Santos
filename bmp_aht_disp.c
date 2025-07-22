#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/bootrom.h"
#include "lwip/tcp.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "aht20.h"
#include "bmp280.h"
#include "ssd1306.h"
#include "font.h"
#include <string.h>
#include <stdlib.h>
#include "ws2818b.pio.h"

#define WIFI_SSID "Son"            // Nome da rede Wi-Fi
#define WIFI_PASS "14164881j"      // Senha da rede Wi-Fi

// Sensores na I2C
#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1
#define SEA_LEVEL_PRESSURE 101325.0

// Display na I2C
#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define endereco 0x3C

#define BOTAO_A 5
#define BOTAO_B 6
#define LED_RED 13
#define LED_BLUE 12
#define LED_GREEN 11
#define BUZZER 21
#define LED_PIN 7
#define LED_COUNT 25
uint sm;
bool state_A = 0;

// =====     Configuracoes para o Buzzer     ===== //

void init_pwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM); // Configura o GPIO como PWM
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_clkdiv(slice_num, 125.0f);     // Define o divisor do clock para 1 MHz
    pwm_set_wrap(slice_num, 1000);         // Define o TOP para frequencia de 1 kHz
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), 0); // Razao ciclica inicial
    pwm_set_enabled(slice_num, true);      // Habilita o PWM
}   
void set_buzzer_tone(uint gpio, uint freq) { // liga o buzzer
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    uint top = 1000000 / freq;             // Calcula o TOP para a frequencia desejada
    pwm_set_wrap(slice_num, top);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), top / 2); // 50% duty cycle
}   
void stop_buzzer(uint gpio) { // para o buzzer
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), 0); // Desliga o PWM
}

//////////////////////////////////////////////////////////////////////////////////////////
// =====     Configuracoes para a Matriz de LEDs     ===== //
// Estrutura para representar um pixel RGB na matriz de LEDs
struct pixel_t {
    uint8_t G, R, B;           // Componentes de cor: verde, vermelho, azul
};
typedef struct pixel_t pixel_t;
typedef pixel_t npLED_t;       // Tipo para LEDs NeoPixel (WS2812B)
npLED_t leds[LED_COUNT];       // Array que armazena o estado de cada LED
PIO np_pio;                    // Instancia do PIO para controlar a matriz
void npDisplayDigit(int digit);
// Matrizes que definem os padroes de exibicao na matriz de LEDs (5x5 pixels)
const uint8_t digits[4][5][5][3] = {
    // Matriz desligada
    {
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}
    },
    // Atencao Vermelho
    {
        {{0, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{100, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {100, 0, 0}},
        {{0, 0, 0}, {100, 0, 0}, {100, 0, 0}, {100, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {0, 0, 0}}
    },
    // Atencao Verde
    {
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 100, 0}, {0, 100, 0}, {0, 100, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 100, 0}, {0, 100, 0}, {0, 100, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}
    },
    // Atencao Azul
    {
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 100}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 100}, {0, 0, 100}, {0, 0, 100}, {0, 0, 0}},
        {{0, 0, 100}, {0, 0, 0}, {0, 0, 100}, {0, 0, 0}, {0, 0, 100}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 100}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 100}, {0, 0, 0}, {0, 0, 0}}
    }
};

// Define as cores de um LED na matriz
void npSetLED(uint index, uint8_t r, uint8_t g, uint8_t b) {
    leds[index].R = r; // Componente vermelho
    leds[index].G = g; // Componente verde
    leds[index].B = b; // Componente azul
}

// Limpa a matriz de LEDs, exibindo o padrao de digito 4 (padrao para limpar)
void npClear() {
    npDisplayDigit(0);
}

// Inicializa a matriz de LEDs WS2812B usando o PIO
void npInit(uint pin) {
    uint offset = pio_add_program(pio0, &ws2818b_program); // Carrega programa PIO
    np_pio = pio0; // Usa PIO0
    sm = pio_claim_unused_sm(np_pio, true); // Reserva uma maquina de estado
    ws2818b_program_init(np_pio, sm, offset, pin, 800000.f); // Inicializa PIO
    npClear(); // Limpa a matriz ao inicializar
}

// Escreve os dados dos LEDs na matriz
void npWrite() {
    for (uint i = 0; i < LED_COUNT; i++) {
        pio_sm_put_blocking(np_pio, sm, leds[i].G); // Envia componente verde
        pio_sm_put_blocking(np_pio, sm, leds[i].R); // Envia componente vermelho
        pio_sm_put_blocking(np_pio, sm, leds[i].B); // Envia componente azul
    }
    sleep_us(100); // Pequeno atraso para estabilizar a comunicacao
}

// Calcula o indice de um LED na matriz com base nas coordenadas (x, y)
int getIndex(int x, int y) {
    if (y % 2 == 0) {
        return 24 - (y * 5 + x); // Linhas pares: ordem direta
    } else {
        return 24 - (y * 5 + (4 - x)); // Linhas impares: ordem invertida
    }
}

// Exibe um digito ou padrao na matriz de LEDs
void npDisplayDigit(int digit) {
    for (int coluna = 0; coluna < 5; coluna++) {
        for (int linha = 0; linha < 5; linha++) {
            int posicao = getIndex(linha, coluna); // Calcula indice do LED
            npSetLED(posicao, digits[digit][coluna][linha][0], // Componente R
                                 digits[digit][coluna][linha][1], // Componente G
                                 digits[digit][coluna][linha][2]); // Componente B
        }
    }
    npWrite(); // Atualiza a matriz com os novos dados
}
//////////////////////////////////////////////////////////////////////////////////////////

// Maximo e Minimo de Umidade
//uint32_t umid_max = 50;
//uint32_t umid_min = 40;

// Declaracoes forward das estruturas
struct sensor_data_t;
struct config_t;

// Estrutura para armazenar configuracoes
struct config_t {
    float temp_min;
    float temp_max;
    float umid_min;
    float umid_max;
    float press_min;
    float press_max;
    float temp_offset;
    float umid_offset;
    float press_offset;
};

// Estrutura para historico de dados com timestamps
#define HISTORY_SIZE 30
struct sensor_history_t {
    float temp_aht[HISTORY_SIZE];
    float temp_bmp[HISTORY_SIZE];
    float humidity[HISTORY_SIZE];
    float pressure[HISTORY_SIZE];
    float altitude[HISTORY_SIZE];
    uint32_t timestamps[HISTORY_SIZE]; // timestamps em segundos desde boot
    int index;
    int count;
};

// Estrutura para dados atuais dos sensores
struct sensor_data_t {
    float temp_aht;
    float temp_bmp;
    float humidity;
    float pressure;
    float altitude;
    bool valid_aht;
    bool valid_bmp;
    absolute_time_t last_reading;
};

// Variaveis globais
static struct config_t config = {
    .temp_min = 20.0, .temp_max = 30.0,
    .umid_min = 30.0, .umid_max = 50.0,
    .press_min = 1000.0, .press_max = 1020.0,
    .temp_offset = 0.0, .umid_offset = 0.0, .press_offset = 0.0
};

static struct sensor_history_t history = {0};
static struct sensor_data_t current_data = {0};
static struct bmp280_calib_param bmp_params;
static absolute_time_t last_interrupt_time = 0;
static bool sensors_initialized = false;

// Declaracoes de funcoes
void gpio_irq_handler(uint gpio, uint32_t events);
bool read_sensors(struct sensor_data_t* data);
bool init_sensors(void);
void init_display(ssd1306_t *ssd);
void update_display(ssd1306_t *ssd, struct sensor_data_t *data, const char *ip_str);
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err);

// HTML da pagina principal (interface completa sem pausar)
// --- PARTE 1 DO HTML ---
const char HTML_PART1[] =
"<html><head><title>Sensores</title><style>"
"body{font:14px Arial;margin:10px;background:#333;color:#fff}"
".d{background:#555;padding:8px;border-radius:5px;margin:5px 0;display:flex;justify-content:space-between}"
".v{font-size:18px;color:#0f0;font-weight:bold}"
".e{color:#f00}"
"button,a,input[type='submit']{padding:5px 10px;margin:2px;background:#666;color:#fff;border:none;border-radius:3px;text-decoration:none;display:inline-block}"
"button:hover,a:hover,input[type='submit']:hover{background:#777}"
".time{font-size:10px;color:#999;text-align:right}"
".pulse{animation:pulse 1s infinite}"
"@keyframes pulse{0%{opacity:1}50%{opacity:0.5}100%{opacity:1}}"
".grid{display:grid;grid-template-columns:1fr 1fr;gap:5px}"
// Padronizando o tamanho da fonte para os spans dentro de .d para 16px (rótulos)
".d span{font-size:16px;}" // Alterado de 14px para 16px
".sensor{font-size:12px;color:#aaa}"
".config-section{background:#444;padding:10px;border-radius:5px;margin-top:10px}"
".config-item{margin-bottom:8px;display:flex;align-items:center}"
".config-item label{flex:1;margin-right:10px}"
".config-item input[type='number']{flex:1;padding:5px;border-radius:3px;border:1px solid #777;background:#666;color:#fff;max-width:80px}"
"</style></head><body>"
// Removendo <h2> e ajustando para um span com font-size de 16px para padronizar o título "Sistema de Sensores"
"<span style='font-size:16px; font-weight:bold; display:block; margin-bottom:10px;'>Sistema de Sensores</span>" 
"<div class='grid'>"
"<div class='d'><span>🌡️ Temp AHT20:</span><span id='t1' class='v'>--C</span></div>"
"<div class='d'><span>🌡️ Temp BMP280:</span><span id='t2' class='v'>--C</span></div>"
"<div class='d'><span>💧 Umidade:</span><span id='h' class='v'>--%</span></div>"
"<div class='d'><span>📊 Pressao:</span><span id='p' class='v'>-- hPa</span></div>"
"<div class='d'><span>⛰️ Altitude:</span><span id='a' class='v'>-- m</span></div>"
"<div class='d'><span>⚡ Status:</span><span id='s' class='v'>--</span></div>" 
"</div>"
"<div class='time'>Ultima atualizacao: <span id='time'>--</span></div>"
"<a href='/graphs' style='font-size:16px;'> Graficos</a>" // Adicionado style para o link também
"<div class='config-section'>"
"<h3>Configuracoes</h3>"
"<form id='configForm' action='/config' method='POST'>"
"<div class='config-item'><label for='umid_min'>Umidade Minima (%):</label><input type='number' step='0.1' id='umid_min' name='umid_min' value=''></div>"
"<div class='config-item'><label for='umid_max'>Umidade Maxima (%):</label><input type='number' step='0.1' id='umid_max' name='umid_max' value=''></div>"
"<div class='config-item'><label for='temp_min'>Temperatura Minima (C):</label><input type='number' step='0.1' id='temp_min' name='temp_min' value=''></div>";

// --- PARTE 2 DO HTML ---
const char HTML_PART2[] =
"<div class='config-item'><label for='temp_max'>Temperatura Maxima (C):</label><input type='number' step='0.1' id='temp_max' name='temp_max' value=''></div>"
//"<div class='config-item'><label for='press_min'>Pressao Minima (hPa):</label><input type='number' step='0.1' id='press_min' name='press_min' value=''></div>"
//"<div class='config-item'><label for='press_max'>Pressao Maxima (hPa):</label><input type='number' step='0.1' id='press_max' name='press_max' value=''></div>"
//"<div class='config-item'><label for='temp_offset'>Offset Temperatura (C):</label><input type='number' step='0.1' id='temp_offset' name='temp_offset' value=''></div>"
//"<div class='config-item'><label for='umid_offset'>Offset Umidade (%):</label><input type='number' step='0.1' id='umid_offset' name='umid_offset' value=''></div>"
//"<div class='config-item'><label for='press_offset'>Offset Pressao (hPa):</label><input type='number' step='0.1' id='press_offset' name='press_offset' value=''></div>"
"<input type='submit' value='Atualizar Configuracoes'>"
"</form>"
"<p id='configStatus' style='color:#0f0'></p>"
"</div>"
"<script>"
"function updateSensorData(){" // Nova função para atualizar apenas os dados dos sensores
"fetch('/data').then(r=>r.json()).then(d=>{"
"document.getElementById('t1').textContent=d.valid_aht?(d.temp_aht.toFixed(1)+'C'):'ERR';"
"document.getElementById('t2').textContent=d.valid_bmp?(d.temp_bmp.toFixed(1)+'C'):'ERR';"
"document.getElementById('h').textContent=d.valid_aht?(d.humidity.toFixed(1)+'%'):'ERR';"
"document.getElementById('p').textContent=d.valid_bmp?(d.pressure.toFixed(1)+' hPa'):'ERR';"
"document.getElementById('a').textContent=d.valid_bmp?(d.altitude.toFixed(1)+' m'):'ERR';"
"let status='OK';"
"if(!d.valid_aht&&!d.valid_bmp)status='ERRO';"
"else if(!d.valid_aht||!d.valid_bmp)status='PARCIAL';"
"document.getElementById('s').textContent=status;"
"document.getElementById('t1').className=d.valid_aht?'v':'e';"
"document.getElementById('t2').className=d.valid_bmp?'v':'e';"
"document.getElementById('h').className=d.valid_aht?'v':'e';"
"document.getElementById('p').className=d.valid_bmp?'v':'e';"
"document.getElementById('a').className=d.valid_bmp?'v':'e';"
"document.getElementById('time').textContent=new Date().toLocaleTimeString();"
"}).catch(e=>{"
"document.getElementById('s').textContent='OFF';"
"});"
"}"

"function updateConfigValues(){" // Nova função para atualizar apenas os valores de configuração
"fetch('/data').then(r=>r.json()).then(d=>{"
"document.getElementById('umid_min').value = d.umid_min.toFixed(1);"
"document.getElementById('umid_max').value = d.umid_max.toFixed(1);"
"document.getElementById('temp_min').value = d.temp_min.toFixed(1);"
"document.getElementById('temp_max').value = d.temp_max.toFixed(1);"
//"document.getElementById('press_min').value = d.press_min.toFixed(1);"
//"document.getElementById('press_max').value = d.press_max.toFixed(1);"
//"document.getElementById('temp_offset').value = d.temp_offset.toFixed(1);"
//"document.getElementById('umid_offset').value = d.umid_offset.toFixed(1);"
//"document.getElementById('press_offset').value = d.press_offset.toFixed(1);"
"}).catch(e=>{ console.error('Erro ao buscar configs:', e); });"
"}"

// Adicionar listener para o formulario de configuracao
"document.getElementById('configForm').addEventListener('submit', function(event) {"
"event.preventDefault();" // Impedir o envio padrao do formulario
"const formData = new FormData(this);"
"const data = {};"
"for (let [key, value] of formData.entries()) {"
"data[key] = parseFloat(value);"
"}"
"fetch(this.action, {"
"method: this.method,"
"headers: {'Content-Type': 'application/json'},"
"body: JSON.stringify(data)"
"})"
".then(response => response.text())"
".then(text => {"
"document.getElementById('configStatus').textContent = text;"
"setTimeout(() => { document.getElementById('configStatus').textContent = ''; updateConfigValues(); }, 2000);" // Atualiza as configs após 2s
"})"
".catch(error => {"
"console.error('Erro ao atualizar configuracoes:', error);"
"document.getElementById('configStatus').textContent = 'Erro ao atualizar!'; document.getElementById('configStatus').style.color = '#f00';"
"setTimeout(() => { document.getElementById('configStatus').textContent = ''; }, 2000);"
"});"
"});"
// Configurar intervalos de atualização diferentes
"setInterval(updateSensorData, 1000);" // Atualiza dados dos sensores a cada 3 segundos
"setInterval(updateConfigValues, 60000);" // Atualiza valores de configuracao a cada 15 segundos
"updateSensorData();" // Chamada inicial para dados dos sensores
"updateConfigValues();" // Chamada inicial para valores de configuracao
"</script></body></html>";

// Funcao para calcular altitude
double calculate_altitude(double pressure) {
    return 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 0.1903));
}

// Pagina de teste simples

// Estrutura para HTTP (buffer otimizado para tempo real com 4 graficos)
struct http_state {
    char response[8192]; // Buffer aumentado para pagina com 4 graficos
    size_t len;
    size_t sent;
};

// Pagina de teste simples
const char TEST_PAGE[] = 
"<!DOCTYPE html><html><head><title>Teste</title></head><body>"
"<h1>Teste OK</h1><p>Servidor funcionando!</p>"
"<a href='/'>Voltar</a></body></html>";

// Pagina de graficos (4 graficos em grid 2x2)
const char GRAPHS_PAGE[] = 
"<html><head><title>Graficos</title>"
"<script src='https://cdnjs.cloudflare.com/ajax/libs/Chart.js/3.9.1/chart.min.js'></script>"
"<style>"
"body{font:14px Arial;margin:10px;background:#333;color:#fff}"
".chart-container{background:#555;padding:8px;margin:5px;border-radius:5px}"
"canvas{max-width:100%; height:280px;max-height:280px}"
"button{padding:5px 10px;margin:5px;background:#666;color:#fff;border:none;border-radius:3px}"
"button:hover{background:#777}"
"a{color:#4af;text-decoration:none}"
".charts{display:grid;grid-template-columns:1fr 1fr;gap:8px}"
"@media(max-width:800px){.charts{grid-template-columns:1fr}}"
"h3{margin:5px 0;font-size:14px;text-align:center}"
"</style></head><body>"
"<h2>Sensores AHT20 + BMP280</h2>"
"<button onclick='updateCharts()'> Atualizar</button> " 
"<a href='/'> Voltar</a>" 
"<div class='charts'>"
"<div class='chart-container'><h3>Temperatura (C)</h3><canvas id='tempChart'></canvas></div>"
"<div class='chart-container'><h3>Umidade (%)</h3><canvas id='humidChart'></canvas></div>"
"<div class='chart-container'><h3>Pressao (hPa)</h3><canvas id='pressChart'></canvas></div>"
"<div class='chart-container'><h3>Altitude (m)</h3><canvas id='altChart'></canvas></div>"
"</div>"
"<script>"
"let tempChart,humidChart,pressChart,altChart;"
"function initCharts(){"
"const tempCtx=document.getElementById('tempChart').getContext('2d');"
"const humidCtx=document.getElementById('humidChart').getContext('2d');"
"const pressCtx=document.getElementById('pressChart').getContext('2d');"
"const altCtx=document.getElementById('altChart').getContext('2d');"
"const chartOptions={responsive:true,maintainAspectRatio:false,scales:{y:{grid:{color:'#666'},ticks:{color:'#fff'}},x:{grid:{color:'#666'},ticks:{color:'#fff'}}},plugins:{legend:{labels:{color:'#fff'}}}};"
"tempChart=new Chart(tempCtx,{type:'line',data:{labels:[],datasets:[{label:'AHT20',data:[],borderColor:'#e74c3c',backgroundColor:'rgba(231,76,60,0.1)',tension:0.4},{label:'BMP280',data:[],borderColor:'#f39c12',backgroundColor:'rgba(243,156,18,0.1)',tension:0.4}]},options:chartOptions});"
"humidChart=new Chart(humidCtx,{type:'line',data:{labels:[],datasets:[{label:'Umidade',data:[],borderColor:'#3498db',backgroundColor:'rgba(52,152,219,0.1)',tension:0.4}]},options:chartOptions});"
"pressChart=new Chart(pressCtx,{type:'line',data:{labels:[],datasets:[{label:'Pressao',data:[],borderColor:'#9b59b6',backgroundColor:'rgba(155,89,182,0.1)',tension:0.4}]},options:chartOptions});"
"altChart=new Chart(altCtx,{type:'line',data:{labels:[],datasets:[{label:'Altitude',data:[],borderColor:'#27ae60',backgroundColor:'rgba(39,174,96,0.1)',tension:0.4}]},options:chartOptions});"
"updateCharts();"
"}"
"function updateCharts(){"
"fetch('/history').then(r=>r.json()).then(d=>{"
"const labels=d.timestamps||[];"
"tempChart.data.labels=labels;"
"tempChart.data.datasets[0].data=d.temp_aht||[];"
"tempChart.data.datasets[1].data=d.temp_bmp||[];"
"tempChart.update();"
"humidChart.data.labels=labels;"
"humidChart.data.datasets[0].data=d.humidity||[];"
"humidChart.update();"
"pressChart.data.labels=labels;"
"pressChart.data.datasets[0].data=d.pressure||[];"
"pressChart.update();"
"altChart.data.labels=labels;"
"altChart.data.datasets[0].data=d.altitude||[];"
"altChart.update();"
"}).catch(e=>console.log('Erro:',e));"
"}"
"window.onload=initCharts;"
"setInterval(updateCharts,3000);"
"</script></body></html>";

// Funcao para extrair valor numerico float de uma string JSON
float extract_json_float(const char* json, const char* key) {
    char search_key[64];
    // Formata a chave para incluir as aspas e o dois pontos, como "umid_min":
    snprintf(search_key, sizeof(search_key), "\"%s\":", key);
    
    char* pos = strstr(json, search_key);
    if (pos) {
        pos += strlen(search_key); // Avanca o ponteiro para o inicio do valor
        // Pula espacos em branco, quebras de linha ou tabulacoes que possam existir antes do numero
        // Adicionado '\n' e '\r' para maior robustez
        while (*pos == ' ' || *pos == '\t' || *pos == '\n' || *pos == '\r') {
            pos++;
        }
        // Converte a string numerica para float
        return strtof(pos, NULL);
    }
    return 0.0f; // Retorna 0.0f se a chave nao for encontrada (ou um valor padrao que faca sentido para sua aplicacao)
}

// Parser completo para JSON de configuracao
void parse_json_config(const char* json, struct config_t* cfg) {
    cfg->temp_min = extract_json_float(json, "temp_min");
    cfg->temp_max = extract_json_float(json, "temp_max");
    cfg->umid_min = extract_json_float(json, "umid_min");
    cfg->umid_max = extract_json_float(json, "umid_max");
    cfg->press_min = extract_json_float(json, "press_min");
    cfg->press_max = extract_json_float(json, "press_max");
    cfg->temp_offset = extract_json_float(json, "temp_offset");
    cfg->umid_offset = extract_json_float(json, "umid_offset");
    cfg->press_offset = extract_json_float(json, "press_offset");

    // Opcional: Imprimir os valores atualizados para depuracao
    printf("DEBUG - Configuracoes atualizadas:\n");
    printf("    Temp Min/Max: %.1f / %.1f\n", cfg->temp_min, cfg->temp_max);
    printf("    Umid Min/Max: %.1f / %.1f\n", cfg->umid_min, cfg->umid_max);
    printf("    Press Min/Max: %.1f / %.1f\n", cfg->press_min, cfg->press_max);
    printf("    Offsets: Temp=%.1f, Umid=%.1f, Press=%.1f\n", cfg->temp_offset, cfg->umid_offset, cfg->press_offset);
}

// Funcao para adicionar dados ao historico com timestamp (mais frequente)
void add_to_history(struct sensor_data_t* data) {
    // So adicionar se pelo menos um sensor tem dados validos
    if (!data->valid_aht && !data->valid_bmp) return;
    
    if (history.count < HISTORY_SIZE) {
        history.count++;
    }
    
    history.temp_aht[history.index] = data->temp_aht;
    history.temp_bmp[history.index] = data->temp_bmp;
    history.humidity[history.index] = data->humidity;
    history.pressure[history.index] = data->pressure;
    history.altitude[history.index] = data->altitude;
    history.timestamps[history.index] = to_ms_since_boot(get_absolute_time()) / 1000; // segundos desde boot
    
    history.index = (history.index + 1) % HISTORY_SIZE;
    
    // Log menos frequente para nao spam no console
    if (history.count % 5 == 0) {
        printf("Historico: %d pontos, AHT: %.1fC/%.1f%%, BMP: %.1fC/%.1fhPa\n", 
               history.count, data->temp_aht, data->humidity, data->temp_bmp, data->pressure);
    }
}

// Funcao para ler sensores (AHT20 + BMP280) - CORRIGIDA
bool read_sensors(struct sensor_data_t* data) {
    if (!sensors_initialized) return false;
    
    // Verificar se ja passou tempo suficiente desde a ultima leitura
    absolute_time_t now = get_absolute_time();
    if (absolute_time_diff_us(data->last_reading, now) < 1000000) { // 1 segundo
        return true; // usar dados anteriores
    }
    
    data->last_reading = now;
    data->valid_aht = false;
    data->valid_bmp = false;
    
    // Leitura do AHT20
    AHT20_Data aht_data;
    if (aht20_read(I2C_PORT, &aht_data)) {
        data->temp_aht = aht_data.temperature + config.temp_offset;
        data->humidity = aht_data.humidity + config.umid_offset;
        data->valid_aht = true;
        
        // Printf apenas a cada 10 leituras para nao spam
        static int read_count = 0;
        if (++read_count % 10 == 0) {
            printf("AHT20 [%d] - Temp: %.2fC, Umid: %.2f%%\n", read_count, data->temp_aht, data->humidity);
        }
    } else {
        printf("Erro na leitura do AHT20\n");
    }
    
    // Leitura do BMP280 - IMPLEMENTACAO CORRIGIDA BASEADA NO CODIGO QUE FUNCIONA
    int32_t raw_temp_bmp, raw_pressure;
    bmp280_read_raw(I2C_PORT, &raw_temp_bmp, &raw_pressure);
    
    // Conversao usando os parametros de calibracao (como no codigo que funciona)
    int32_t temperature = bmp280_convert_temp(raw_temp_bmp, &bmp_params);
    int32_t pressure = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &bmp_params);
    
    // Verificar se os valores convertidos sao validos
    if (temperature != 0 && pressure != 0) {
        data->temp_bmp = (temperature / 100.0f) + config.temp_offset;
        data->pressure = (pressure / 100.0f) + config.press_offset; // hPa
        // Calcular altitude usando a pressao em Pa (como no codigo que funciona)
        data->altitude = calculate_altitude(pressure); // pressure ja esta em Pa
        data->valid_bmp = true;
        
        // Printf apenas a cada 10 leituras para nao spam
        static int bmp_count = 0;
        if (++bmp_count % 10 == 0) {
            printf("BMP280 [%d] - Temp: %.2fC, Press: %.2f hPa, Alt: %.2f m\n", 
                   bmp_count, data->temp_bmp, data->pressure, data->altitude);
        }
    } else {
        printf("Erro na leitura do BMP280 - conversao falhou\n");
    }
    
    // Adicionar ao historico apenas se pelo menos um sensor funcionou
    if (data->valid_aht || data->valid_bmp) {
        add_to_history(data);
    }
    
    return (data->valid_aht || data->valid_bmp);
}

// Callback para dados enviados
static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    struct http_state *hs = (struct http_state *)arg;
    hs->sent += len;
    if (hs->sent >= hs->len) {
        tcp_close(tpcb);
        free(hs);
    }
    return ERR_OK;
}

// Funcao para encontrar o corpo da requisicao POST
char* find_http_body(char* request) {
    char* body = strstr(request, "\r\n\r\n");
    if (body) {
        return body + 4; // pular os 4 caracteres \r\n\r\n
    }
    return NULL;
}

// Callback para dados recebidos
static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (!p) {
        tcp_close(tpcb);
        return ERR_OK;
    }

    // Verificar tamanho do payload
    // Aumentamos para 2048 para permitir payloads POST maiores
    if (p->tot_len > 2048) { 
        printf("AVISO: Payload muito grande: %d bytes\n", p->tot_len);
        pbuf_free(p);
        tcp_close(tpcb);
        return ERR_OK;
    }

    char *req = (char *)p->payload;
    struct http_state *hs = malloc(sizeof(struct http_state));
    if (!hs) {
        printf("Erro: Falha ao alocar %d bytes para resposta HTTP\n", sizeof(struct http_state));
        pbuf_free(p);
        tcp_close(tpcb);
        return ERR_MEM;
    }
    
    // Inicializar estrutura
    memset(hs, 0, sizeof(struct http_state));
    hs->sent = 0;

    // Extrair primeira linha da requisicao para debug
    char request_line[100] = {0};
    char *line_end = strstr(req, "\r\n");
    if (line_end && (line_end - req) < sizeof(request_line) - 1) {
        strncpy(request_line, req, line_end - req);
        printf("Requisicao: %s\n", request_line);
    } else {
        printf("Requisicao: %.50s...\n", req);
    }

    // --- NOVO: Endpoint de configuracao (POST) ---
    if (strstr(req, "POST /config")) {
        char* body = find_http_body(req); // Encontra o inicio do corpo HTTP
        if (body) {
            printf("Corpo da requisicao POST /config: %s\n", body);
            
            // Chamada para parsear o JSON e atualizar a configuracao
            parse_json_config(body, &config);
            
            const char *response_text = "Configuracao atualizada com sucesso!";
            hs->len = snprintf(hs->response, sizeof(hs->response),
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/plain\r\n"
                "Access-Control-Allow-Origin: *\r\n" // Necessario para CORS no fetch JS
                "Content-Length: %d\r\n"
                "Connection: close\r\n"
                "\r\n"
                "%s", (int)strlen(response_text), response_text);
        } else {
            const char *response_text = "Erro: Corpo da requisicao ausente.";
            hs->len = snprintf(hs->response, sizeof(hs->response),
                "HTTP/1.1 400 Bad Request\r\n"
                "Content-Type: text/plain\r\n"
                "Access-Control-Allow-Origin: *\r\n"
                "Content-Length: %d\r\n"
                "Connection: close\r\n"
                "\r\n"
                "%s", (int)strlen(response_text), response_text);
        }
    }
    // --- FIM NOVO: Endpoint de configuracao (POST) ---

    // Endpoint de teste
    else if (strstr(req, "GET /test")) {
        printf("Servindo pagina de teste\n");
        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html\r\n"
            "Content-Length: %d\r\n"
            "Connection: close\r\n"
            "\r\n"
            "%s", (int)strlen(TEST_PAGE), TEST_PAGE);
    }
    // Endpoint para pagina de graficos
    else if (strstr(req, "GET /graphs")) {
        printf("Servindo pagina de graficos\n");
        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html\r\n"
            "Content-Length: %d\r\n"
            "Connection: close\r\n"
            "\r\n"
            "%s", (int)strlen(GRAPHS_PAGE), GRAPHS_PAGE);
    }
    // Endpoint para dados historicos
    else if (strstr(req, "GET /history")) {
        char json_payload[1536];
        int json_len = 0;
        
        // Comecar o JSON
        json_len += snprintf(json_payload + json_len, sizeof(json_payload) - json_len,
            "{\"timestamps\":[");
        
        // Adicionar timestamps
        for (int i = 0; i < history.count; i++) {
            int idx = (history.index - history.count + i + HISTORY_SIZE) % HISTORY_SIZE;
            json_len += snprintf(json_payload + json_len, sizeof(json_payload) - json_len,
                "%s%u", (i > 0) ? "," : "", history.timestamps[idx]);
        }
        
        json_len += snprintf(json_payload + json_len, sizeof(json_payload) - json_len,
            "],\"temp_aht\":[");
        
        // Adicionar temperaturas AHT20
        for (int i = 0; i < history.count; i++) {
            int idx = (history.index - history.count + i + HISTORY_SIZE) % HISTORY_SIZE;
            json_len += snprintf(json_payload + json_len, sizeof(json_payload) - json_len,
                "%s%.1f", (i > 0) ? "," : "", history.temp_aht[idx]);
        }
        
        json_len += snprintf(json_payload + json_len, sizeof(json_payload) - json_len,
            "],\"temp_bmp\":[");
        
        // Adicionar temperaturas BMP280
        for (int i = 0; i < history.count; i++) {
            int idx = (history.index - history.count + i + HISTORY_SIZE) % HISTORY_SIZE;
            json_len += snprintf(json_payload + json_len, sizeof(json_payload) - json_len,
                "%s%.1f", (i > 0) ? "," : "", history.temp_bmp[idx]);
        }
        
        json_len += snprintf(json_payload + json_len, sizeof(json_payload) - json_len,
            "],\"humidity\":[");
        
        // Adicionar umidade
        for (int i = 0; i < history.count; i++) {
            int idx = (history.index - history.count + i + HISTORY_SIZE) % HISTORY_SIZE;
            json_len += snprintf(json_payload + json_len, sizeof(json_payload) - json_len,
                "%s%.1f", (i > 0) ? "," : "", history.humidity[idx]);
        }
        
        json_len += snprintf(json_payload + json_len, sizeof(json_payload) - json_len,
            "],\"pressure\":[");
        
        // Adicionar pressao
        for (int i = 0; i < history.count; i++) {
            int idx = (history.index - history.count + i + HISTORY_SIZE) % HISTORY_SIZE;
            json_len += snprintf(json_payload + json_len, sizeof(json_payload) - json_len,
                "%s%.1f", (i > 0) ? "," : "", history.pressure[idx]);
        }
        
        json_len += snprintf(json_payload + json_len, sizeof(json_payload) - json_len,
            "],\"altitude\":[");
        
        // Adicionar altitude
        for (int i = 0; i < history.count; i++) {
            int idx = (history.index - history.count + i + HISTORY_SIZE) % HISTORY_SIZE;
            json_len += snprintf(json_payload + json_len, sizeof(json_payload) - json_len,
                "%s%.1f", (i > 0) ? "," : "", history.altitude[idx]);
        }
        
        json_len += snprintf(json_payload + json_len, sizeof(json_payload) - json_len,
            "],\"count\":%d}", history.count);

        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Content-Length: %d\r\n"
            "Connection: close\r\n"
            "\r\n"
            "%s", json_len, json_payload);
    }
    // Endpoint para dados dos sensores (MODIFICADO para incluir offsets!)
    else if (strstr(req, "GET /data")) {
        read_sensors(&current_data);
        
        char json_payload[512]; // Certifique-se de que este buffer e grande o suficiente!
        int json_len = snprintf(json_payload, sizeof(json_payload),
            "{"
            "\"temp_aht\":%.2f,"
            "\"temp_bmp\":%.2f,"
            "\"humidity\":%.2f,"
            "\"pressure\":%.2f,"
            "\"altitude\":%.2f,"
            "\"valid_aht\":%s,"
            "\"valid_bmp\":%s,"
            "\"temp_min\":%.1f,"
            "\"temp_max\":%.1f,"
            "\"umid_min\":%.1f,"
            "\"umid_max\":%.1f,"
            "\"press_min\":%.1f,"
            "\"press_max\":%.1f,"
            "\"temp_offset\":%.1f," 
            "\"umid_offset\":%.1f," 
            "\"press_offset\":%.1f"  
            "}",
            current_data.temp_aht, current_data.temp_bmp, current_data.humidity,
            current_data.pressure, current_data.altitude,
            current_data.valid_aht ? "true" : "false",
            current_data.valid_bmp ? "true" : "false",
            config.temp_min, config.temp_max,
            config.umid_min, config.umid_max,
            config.press_min, config.press_max,
            config.temp_offset, config.umid_offset, config.press_offset); 

        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Content-Length: %d\r\n"
            "Connection: close\r\n"
            "\r\n"
            "%s", json_len, json_payload);
    }
    // Reset do historico
    else if (strstr(req, "GET /reset_history")) {
        memset(&history, 0, sizeof(history));
        printf("Historico resetado\n");
        
        const char *response_text = "Historico limpo";
        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/plain\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "Content-Length: %d\r\n"
            "Connection: close\r\n"
            "\r\n"
            "%s", (int)strlen(response_text), response_text);
    }
    // Pagina principal (MODIFICADO para usar as partes HTML_PART1 e HTML_PART2)
    else {
        // Concatenar as partes do HTML
        // O -1 e porque snprintf ja adiciona um NULL terminator, e nao queremos dois se as partes forem coladas diretamente.
        char full_html_body[sizeof(HTML_PART1) + sizeof(HTML_PART2) -1]; 
        snprintf(full_html_body, sizeof(full_html_body), "%s%s", HTML_PART1, HTML_PART2);

        printf("Servindo pagina principal (HTML: %d bytes)\n", (int)strlen(full_html_body));
        
        // Verificar se o HTML cabe no buffer antes de tentar montar a resposta
        int html_size = strlen(full_html_body);
        int header_size = 150; // Tamanho estimado do cabecalho HTTP
        int total_needed = html_size + header_size;
        
        // response para 8192
        if (total_needed >= sizeof(hs->response)) {
            printf("ERRO: HTML muito grande (%d bytes) para buffer (%d bytes) mesmo apos concatenacao!\n",
                   total_needed, (int)sizeof(hs->response));
            
            // Resposta de erro simples
            const char* error_msg = "Erro: Pagina muito grande (concatenacao falhou)";
            hs->len = snprintf(hs->response, sizeof(hs->response),
                "HTTP/1.1 500 Internal Server Error\r\n"
                "Content-Type: text/plain\r\n"
                "Content-Length: %d\r\n"
                "Connection: close\r\n"
                "\r\n"
                "%s", (int)strlen(error_msg), error_msg);
        } else {
            // Montar resposta HTTP normal
            hs->len = snprintf(hs->response, sizeof(hs->response),
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/html; charset=utf-8\r\n"
                "Content-Length: %d\r\n"
                "Connection: close\r\n"
                "\r\n"
                "%s", html_size, full_html_body); // Usamos full_html_body aqui
                
            printf("Resposta HTTP: %d bytes (buffer: %d bytes) - OK\n",
                   (int)hs->len, (int)sizeof(hs->response));
        }
    }

    tcp_arg(tpcb, hs);
    tcp_sent(tpcb, http_sent);

    // Verificar se a resposta foi gerada com sucesso
    if (hs->len == 0 || hs->len >= sizeof(hs->response)) {
        printf("ERRO: Resposta invalida (len=%d, max=%d)\n", (int)hs->len, (int)sizeof(hs->response));
        
        // Gerar resposta de erro de emergencia
        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 500 Internal Server Error\r\n"
            "Content-Type: text/plain\r\n"
            "Content-Length: 13\r\n"
            "Connection: close\r\n"
            "\r\n"
            "Server Error");
    }

    printf("Enviando resposta: %d bytes\n", (int)hs->len);
    
    err_t write_err = tcp_write(tpcb, hs->response, hs->len, TCP_WRITE_FLAG_COPY);
    if (write_err != ERR_OK) {
        printf("Erro ao escrever TCP: %d\n", write_err);
        free(hs);
        tcp_close(tpcb);
        pbuf_free(p);
        return write_err;
    }
    
    tcp_output(tpcb);

    pbuf_free(p);
    return ERR_OK;
}


// Callback para novas conexoes
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err) {
    (void)arg;   // Evitar warning de parametro nao usado
    (void)err;   // Evitar warning de parametro nao usado
    tcp_recv(newpcb, http_recv);
    return ERR_OK;
}

// Iniciar servidor HTTP
static void start_http_server(void) {
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) {
        printf("Erro ao criar PCB TCP\n");
        return;
    }
    if (tcp_bind(pcb, IP_ADDR_ANY, 80) != ERR_OK) {
        printf("Erro ao ligar o servidor na porta 80\n");
        return;
        }
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, connection_callback);
    printf("Servidor HTTP rodando na porta 80...\n");
}

// Funcao de inicializacao dos sensores (AHT20 + BMP280) - CORRIGIDA
bool init_sensors(void) {
    printf("Inicializando sensores...\n");
    
    // I2C para sensores
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    sleep_ms(50); // aguardar estabilizacao do I2C

    // Inicializar BMP280 - IMPLEMENTACAO CORRIGIDA
    printf("Inicializando BMP280...\n");
    bmp280_init(I2C_PORT);
    sleep_ms(50); // aguardar inicializacao
    
    bmp280_get_calib_params(I2C_PORT, &bmp_params);
    printf("BMP280 inicializado e calibrado\n");
    
    // Inicializar AHT20
    printf("Inicializando AHT20...\n");
    aht20_reset(I2C_PORT);
    sleep_ms(20); // aguardar reset
    
    aht20_init(I2C_PORT);
    sleep_ms(30); // aguardar inicializacao
    printf("AHT20 inicializado\n");
    
    // Fazer uma leitura de teste para verificar se os sensores estao funcionando
    printf("Testando sensores...\n");
    
    // Teste do AHT20
    AHT20_Data test_aht;
    bool aht_ok = aht20_read(I2C_PORT, &test_aht);
    
    // Teste do BMP280 - IMPLEMENTACAO CORRIGIDA
    int32_t test_temp, test_press;
    bmp280_read_raw(I2C_PORT, &test_temp, &test_press);
    
    // Converter valores para teste (como no codigo que funciona)
    int32_t conv_temp = bmp280_convert_temp(test_temp, &bmp_params);
    int32_t conv_press = bmp280_convert_pressure(test_press, test_temp, &bmp_params);
    
    bool bmp_ok = (conv_temp != 0 && conv_press != 0);
    
    if (aht_ok) {
        printf("AHT20: OK (Temp: %.1fC, Umid: %.1f%%)\n", test_aht.temperature, test_aht.humidity);
    } else {
        printf("AHT20: FALHA\n");
    }
    
    if (bmp_ok) {
        printf("BMP280: OK (Temp: %.1fC, Press: %.1f hPa)\n", 
               conv_temp/100.0f, conv_press/100.0f);
    } else {
        printf("BMP280: FALHA\n");
    }
    
    sensors_initialized = true;
    
    // Retorna true se pelo menos um sensor esta funcionando
    return (aht_ok || bmp_ok);
}

// Funcao de inicializacao do display
void init_display(ssd1306_t *ssd) {
    i2c_init(I2C_PORT_DISP, 400 * 1000);
    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);
    
    ssd1306_init(ssd, 128, 64, false, endereco, I2C_PORT_DISP);
    ssd1306_config(ssd);
    ssd1306_fill(ssd, false);
    ssd1306_send_data(ssd);
}

// Funcao para atualizar display com dados dos sensores (AHT20 + BMP280)
void update_display(ssd1306_t *ssd, struct sensor_data_t *data, const char *ip_str) {
    char str_temp_aht[10], str_temp_bmp[10], str_umid[10], str_press[10];
    
    if (data->valid_aht) {
        snprintf(str_temp_aht, sizeof(str_temp_aht), "%.1fC", data->temp_aht);
        snprintf(str_umid, sizeof(str_umid), "%.1f%%", data->humidity);
    } else {
        strcpy(str_temp_aht, "ERR");
        strcpy(str_umid, "ERR");
    }
    
    if (data->valid_bmp) {
        snprintf(str_temp_bmp, sizeof(str_temp_bmp), "%.1fC", data->temp_bmp);
        snprintf(str_press, sizeof(str_press), "%.0fhPa", data->pressure);
    } else {
        strcpy(str_temp_bmp, "ERR");
        strcpy(str_press, "ERR");
    }


    ssd1306_fill(ssd, false);
    ssd1306_rect(ssd, 2, 2, 124, 60, true, false);

    if (state_A == 0) { // SENSORES
    // Cabecalho
    //ssd1306_draw_string(ssd, "SENSORES", 35, 5);
    ssd1306_line(ssd, 2, 15, 126, 15, true);
    
    // IP
    ssd1306_draw_string(ssd, "EMBARCATECH", 22, 5);
    //ssd1306_draw_string(ssd, ip_str, 35, 5);
    //ssd1306_draw_string(ssd, ip_str, 5, 18);
    //ssd1306_line(ssd, 2, 28, 126, 28, true);
    
    // Dados dos sensores (compacto)
    ssd1306_draw_string(ssd, "AHT", 5, 20);
    ssd1306_draw_string(ssd, "Temp:", 35, 20);
    ssd1306_draw_string(ssd, "Umid:", 35, 30);
    ssd1306_draw_string(ssd, str_temp_aht, 80, 20);
    ssd1306_draw_string(ssd, str_umid, 80, 30);
    
    ssd1306_draw_string(ssd, "BMP", 5, 40);
    ssd1306_draw_string(ssd, "Temp:", 35, 40);
    ssd1306_draw_string(ssd, "Pres:", 35, 50);
    ssd1306_draw_string(ssd, str_temp_bmp, 80, 40);
    //ssd1306_draw_string(ssd, str_press, 70, 42);
    /*
    // Indicadores de status
    ssd1306_rect(ssd, 5, 52, 6, 6, true, data->valid_aht);
    ssd1306_draw_string(ssd, "A", 15, 53);
    
    ssd1306_rect(ssd, 30, 52, 6, 6, true, data->valid_bmp);
    ssd1306_draw_string(ssd, "B", 40, 53);
    */
    char alt_str[10];
    if (data->valid_bmp) {
        snprintf(alt_str, sizeof(alt_str), "%.0fm", data->altitude);
        ssd1306_draw_string(ssd, alt_str, 80, 50);
    }
    } else { // state_A != 0, Modo: VALORES MAX/MIN
        // Cabecalho
        ssd1306_line(ssd, 2, 15, 126, 15, true);
        ssd1306_draw_string(ssd, "IP:", 5, 5);
        ssd1306_draw_string(ssd, ip_str, 35, 5);

        // Limites de Umidade
        ssd1306_draw_string(ssd, "Umid Min:", 5, 20);
        snprintf(str_press, sizeof(str_press), "%.1f%%", config.umid_min); // Reutilizando str_press
        ssd1306_draw_string(ssd, str_press, 80, 20);

        ssd1306_draw_string(ssd, "Umid Max:", 5, 30);
        snprintf(str_umid, sizeof(str_umid), "%.1f%%", config.umid_max); // Reutilizando str_umid
        ssd1306_draw_string(ssd, str_umid, 80, 30);

        // Limites de Temperatura
        ssd1306_draw_string(ssd, "Temp Min:", 5, 40);
        snprintf(str_temp_bmp, sizeof(str_temp_bmp), "%.1fC", config.temp_min); // Reutilizando str_temp_bmp
        ssd1306_draw_string(ssd, str_temp_bmp, 80, 40);

        ssd1306_draw_string(ssd, "Temp Max:", 5, 50);
        snprintf(str_temp_aht, sizeof(str_temp_aht), "%.1fC", config.temp_max); // Reutilizando str_temp_aht
        ssd1306_draw_string(ssd, str_temp_aht, 80, 50);
    }
    
    
    
    ssd1306_send_data(ssd);
}

// Handler de interrupcoes
void gpio_irq_handler(uint gpio, uint32_t events) {
    absolute_time_t now = get_absolute_time();
    int64_t diff = absolute_time_diff_us(last_interrupt_time, now);
    if (diff < 250000) return; // debounce de 250ms
    last_interrupt_time = now;
    if (gpio == BOTAO_A) {
        printf("Botao A pressionado\n");
        state_A = !state_A;
        
    } else if (gpio == BOTAO_B) {
        printf("Reset via botao B\n");
        reset_usb_boot(0, 0);
    }
}
float temp_media = 0;

// Funcao principal
int main() {

    
    stdio_init_all();
    init_pwm(BUZZER); // inicia buzzer
    npInit(LED_PIN); // matriz

    sleep_ms(2000);
    
    printf("=== Sistema de Monitoramento de Sensores ===\n");
    printf("Iniciando sistema...\n");

    // Configurar botoes
    gpio_init(BOTAO_A);
    gpio_set_dir(BOTAO_A, GPIO_IN);
    gpio_pull_up(BOTAO_A);
    gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    
    gpio_init(BOTAO_B);
    gpio_set_dir(BOTAO_B, GPIO_IN);
    gpio_pull_up(BOTAO_B);
    gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // Configurar LEDs
    gpio_init(LED_RED);
    gpio_init(LED_GREEN);
    gpio_init(LED_BLUE);
    gpio_set_dir(LED_RED, GPIO_OUT);
    gpio_set_dir(LED_GREEN, GPIO_OUT);
    gpio_set_dir(LED_BLUE, GPIO_OUT);

    // Inicializar display
    ssd1306_t ssd;
    init_display(&ssd);
    
    ssd1306_draw_string(&ssd, "Iniciando...", 0, 0);
    ssd1306_draw_string(&ssd, "Wi-Fi...", 0, 15);
    ssd1306_send_data(&ssd);

    // Inicializar Wi-Fi
    printf("Inicializando Wi-Fi...\n");
    if (cyw43_arch_init()) {
        printf("Erro na inicializacao do Wi-Fi\n");
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "WiFi FALHA", 0, 0);
        ssd1306_draw_string(&ssd, "Verifique SSID/PASS", 0, 15);
        ssd1306_send_data(&ssd);
        return 1;
    }

    cyw43_arch_enable_sta_mode();
    printf("Conectando ao Wi-Fi: %s\n", WIFI_SSID);
    
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Erro na conexao Wi-Fi\n");
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "WiFi ERRO", 0, 0);
        ssd1306_draw_string(&ssd, "Verifique SSID/PASS", 0, 15);
        ssd1306_send_data(&ssd);
        return 1;
    }

    // Obter IP
    extern cyw43_t cyw43_state;
    uint8_t *ip = (uint8_t *)&(cyw43_state.netif[0].ip_addr.addr);
    char ip_str[24];
    snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    printf("Wi-Fi conectado! IP: %s\n", ip_str);

    // Inicializar sensores
    if (!init_sensors()) {
        printf("Erro na inicializacao dos sensores\n");
        ssd1306_fill(&ssd, false);
        ssd1306_draw_string(&ssd, "SENSOR ERRO", 0, 0);
        ssd1306_draw_string(&ssd, "Verifique I2C", 0, 15);
        ssd1306_send_data(&ssd);
        // Continuar mesmo com erro nos sensores
    }

    // Iniciar servidor HTTP
    start_http_server();
    printf("Pagina de graficos: http://%s/graphs\n", ip_str);
    printf("Pagina de teste: http://%s/test\n", ip_str);
    printf("API dados: http://%s/data\n", ip_str);
    printf("API historico: http://%s/history\n", ip_str);

    // Loop principal
    absolute_time_t last_sensor_read = get_absolute_time();
    absolute_time_t last_display_update = get_absolute_time();

    
    while (true) {
        temp_media = (current_data.temp_aht + current_data.temp_bmp) / 2;
        // Polling da rede
        cyw43_arch_poll();
        
        // Ler sensores a cada 1 segundo
        absolute_time_t now = get_absolute_time();
        if (sensors_initialized && 
            absolute_time_diff_us(last_sensor_read, now) > 1000000) {
            
            read_sensors(&current_data);
            last_sensor_read = now;
        }
        
        // Atualizar display a cada 1 segundo
        if (absolute_time_diff_us(last_display_update, now) > 1000000) {
            update_display(&ssd, &current_data, ip_str);
            last_display_update = now;
        }

        // Feedback sonoro

        
        // Controlar LEDs baseado no status (AHT20 + BMP280)
        if (sensors_initialized) {
            if (current_data.valid_aht && current_data.valid_bmp) {
                gpio_put(LED_GREEN, true);
                gpio_put(LED_RED, false);
                gpio_put(LED_BLUE, false);
            } else if (current_data.valid_aht || current_data.valid_bmp) {
                gpio_put(LED_GREEN, false);
                gpio_put(LED_RED, false);
                gpio_put(LED_BLUE, true); // azul para sensor parcial
            } else {
                gpio_put(LED_GREEN, false);
                gpio_put(LED_RED, true);
                gpio_put(LED_BLUE, false);
            }
        } else {
            // Sensores nao inicializados
            gpio_put(LED_GREEN, false);
            gpio_put(LED_RED, true);
            gpio_put(LED_BLUE, false);
        }
        
        if (temp_media > config.temp_max) {
            set_buzzer_tone(BUZZER, 395);
            sleep_ms(100);
            stop_buzzer(BUZZER); 
        } else if (temp_media < config.temp_min) {
            set_buzzer_tone(BUZZER, 330);
            sleep_ms(100);
            stop_buzzer(BUZZER);
        } else {
            stop_buzzer(BUZZER);
        }

        sleep_ms(200); // loop responsivo

        if (current_data.humidity > config.umid_max) {
            npDisplayDigit(3);
        } else if (current_data.humidity < config.umid_min) {
            npDisplayDigit(1);
        } else {
            npDisplayDigit(2);
        }
    }

    cyw43_arch_deinit();
    return 0;
}
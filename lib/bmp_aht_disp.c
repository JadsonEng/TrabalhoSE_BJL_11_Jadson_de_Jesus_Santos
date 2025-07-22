#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "aht20.h"
#include "bmp280.h"
#include "ssd1306.h"
#include "font.h"
#include <math.h>
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "ws2818b.pio.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/netif.h"
#include "pico/cyw43_arch.h"


#define I2C_PORT i2c0               // i2c0 pinos 0 e 1, i2c1 pinos 2 e 3
#define I2C_SDA 0                   // 0 ou 2
#define I2C_SCL 1                   // 1 ou 3
#define SEA_LEVEL_PRESSURE 101325.0 // Pressão ao nível do mar em Pa
// Display na I2C
#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define endereco 0x3C
#define BOTAO_A 5
#define LED_RED 13
#define LED_BLUE 12
#define LED_GREEN  11
#define LED_PIN 7
#define LED_COUNT 25
#define BUZZER 21
// configuração da rede Wi-Fi
#define WIFI_SSID "Son"
#define WIFI_PASSWORD "14164881j"

// Variáveis globais para os dados dos sensores
volatile float bmp_temperature = 0.0;
volatile float aht_temperature = 0.0;
volatile float humidity = 0.0;
volatile float altitude = 0.0;

// Estrutura para manter o estado da resposta HTTP
struct http_state {
    char response[4096];
    size_t len;
    size_t sent;
};

// Página HTML adaptada para os sensores
const char HTML_BODY[] =
"<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Monitor de Sensores</title>"
"<style>"
"body{font-family:sans-serif;text-align:center;padding:5px;margin:0;}"
"h2{margin:5px 0;}"
".barra{width:80%; height:15px;background:#ddd;margin:5px auto;}"
".preenchimento{height:100%;background:#2196F3;}"
".botao{background:#2196F3;color:#fff;margin:5px;padding:5px;}"
".reset{background:#FF9800;}"
"input{width:60px;margin:2px;}"
"#confirm{color:green;font-size:small;}"
"</style>"
"<script>"
"function inicializar(){"
"fetch('/estado').then(r=>r.json()).then(data=>{"
"atualizar(data);"
"});"
"}"
"function calcularPercentual(valor,min,max){"
"var percent=((valor-min)/(max-min))*100;"
"if(percent<0)percent=0;"
"if(percent>100)percent=100;"
"return percent;"
"}"
"function atualizar(data){"
"if(!data){"
"fetch('/estado').then(r=>r.json()).then(d=>{"
"document.getElementById('bmp_temp').innerText=d.bmp_temp.toFixed(1);"
"document.getElementById('aht_temp').innerText=d.aht_temp.toFixed(1);"
"document.getElementById('humidity').innerText=d.humidity.toFixed(1);"
"document.getElementById('altitude').innerText=d.altitude.toFixed(0);"
"var percent=calcularPercentual(d.humidity,0,100);"
"document.getElementById('barra_humidity').style.width=percent+'%';"
"});"
"}else{"
"document.getElementById('bmp_temp').innerText=data.bmp_temp.toFixed(1);"
"document.getElementById('aht_temp').innerText=data.aht_temp.toFixed(1);"
"document.getElementById('humidity').innerText=data.humidity.toFixed(1);"
"document.getElementById('altitude').innerText=data.altitude.toFixed(0);"
"var percent=calcularPercentual(data.humidity,0,100);"
"document.getElementById('barra_humidity').style.width=percent+'%';"
"}"
"}"
"setInterval(function(){atualizar();},1000);"
"window.onload=inicializar;"
"</script></head><body>"
"<h2>EMBARCATECH</h2>"
"<p>Temperatura BMP280: <span id='bmp_temp'>--</span> °C</p>"
"<p>Temperatura AHT20: <span id='aht_temp'>--</span> °C</p>"
"<p>Umidade: <span id='humidity'>--</span> %</p>"
"<div class='barra'><div id='barra_humidity' class='preenchimento' style='width:0'></div></div>"
"<p>Altitude: <span id='altitude'>--</span> m</p>"
"<p id='confirm'></p>"
"</body></html>";

uint sm;

// =====     Configurações para o Buzzer     ===== //
#define BUZZER 21
//#define BUZZER2 10
void init_pwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM); // Configura o GPIO como PWM
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_clkdiv(slice_num, 125.0f);     // Define o divisor do clock para 1 MHz
    pwm_set_wrap(slice_num, 1000);        // Define o TOP para frequência de 1 kHz
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), 0); // Razão cíclica inicial
    pwm_set_enabled(slice_num, true);     // Habilita o PWM
}   
void set_buzzer_tone(uint gpio, uint freq) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    uint top = 1000000 / freq;            // Calcula o TOP para a frequência desejada
    pwm_set_wrap(slice_num, top);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), top / 2); // 50% duty cycle
}   
void stop_buzzer(uint gpio) {
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio), 0); // Desliga o PWM
}

// Callback quando parte da resposta foi enviada
static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    struct http_state *hs = (struct http_state *)arg;
    hs->sent += len;
    if (hs->sent >= hs->len) {
        tcp_close(tpcb);
        free(hs);
    }
    return ERR_OK;
}

// Recebe e trata as requisições
static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (!p) {
        tcp_close(tpcb);
        return ERR_OK;
    }

    char *req = (char *)p->payload;
    struct http_state *hs = malloc(sizeof(struct http_state));
    if (!hs) {
        pbuf_free(p);
        tcp_close(tpcb);
        return ERR_MEM;
    }
    hs->sent = 0;

    if (strncmp(req, "GET / ", 6) == 0 || strncmp(req, "GET /index.html", 15) == 0) {
        int content_length = strlen(HTML_BODY);
        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html\r\n"
            "Content-Length: %d\r\n"
            "Connection: close\r\n"
            "\r\n"
            "%s",
            content_length, HTML_BODY);
    } else if (strstr(req, "GET /estado")) {
        char json[256];
        int json_len = snprintf(json, sizeof(json),
            "{\"bmp_temp\":%.1f,\"aht_temp\":%.1f,\"humidity\":%.1f,\"altitude\":%.0f}",
            bmp_temperature, aht_temperature, humidity, altitude);

        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: %d\r\n"
            "Connection: close\r\n"
            "\r\n"
            "%s",
            json_len, json);
    } else {
        const char *resp = "404 Not Found";
        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 404 Not Found\r\n"
            "Content-Type: text/plain\r\n"
            "Content-Length: %d\r\n"
            "Connection: close\r\n"
            "\r\n"
            "%s",
            (int)strlen(resp), resp);
    }

    tcp_arg(tpcb, hs);
    tcp_sent(tpcb, http_sent);
    tcp_write(tpcb, hs->response, hs->len, TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);
    pbuf_free(p);
    return ERR_OK;
}

// Configura nova conexão
static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err) {
    tcp_recv(newpcb, http_recv);
    return ERR_OK;
}

// Inicializa servidor HTTP
static void start_http_server(void) {
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) return;
    if (tcp_bind(pcb, IP_ADDR_ANY, 80) != ERR_OK) return;
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, connection_callback);
}

//////////////////////////////////////////////////////////////////////////////////////////
// =====     Configurações para a Matriz de LEDs     ===== //
// Estrutura para representar um pixel RGB na matriz de LEDs
struct pixel_t {
    uint8_t G, R, B;           // Componentes de cor: verde, vermelho, azul
};
typedef struct pixel_t pixel_t;
typedef pixel_t npLED_t;       // Tipo para LEDs NeoPixel (WS2812B)
npLED_t leds[LED_COUNT];       // Array que armazena o estado de cada LED
PIO np_pio;                    // Instância do PIO para controlar a matriz
void npDisplayDigit(int digit);
// Matrizes que definem os padrões de exibição na matriz de LEDs (5x5 pixels)
const uint8_t digits[4][5][5][3] = {
    // Matriz desligada
    {
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}
    },
    // Atenção Vermelho
    {
        {{0, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {0, 0, 0}}
    },
    // Atenção Verde
    {
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}}
    },
    // Atenção Azul
    {
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 100}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 100}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 100}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 100}, {0, 0, 0}, {0, 0, 0}}
    }
};

// Define as cores de um LED na matriz
void npSetLED(uint index, uint8_t r, uint8_t g, uint8_t b) {
    leds[index].R = r; // Componente vermelho
    leds[index].G = g; // Componente verde
    leds[index].B = b; // Componente azul
}

// Limpa a matriz de LEDs, exibindo o padrão de dígito 4 (padrão para limpar)
void npClear() {
    npDisplayDigit(0);
}

// Inicializa a matriz de LEDs WS2812B usando o PIO
void npInit(uint pin) {
    uint offset = pio_add_program(pio0, &ws2818b_program); // Carrega programa PIO
    np_pio = pio0; // Usa PIO0
    sm = pio_claim_unused_sm(np_pio, true); // Reserva uma máquina de estado
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
    sleep_us(100); // Pequeno atraso para estabilizar a comunicação
}

// Calcula o índice de um LED na matriz com base nas coordenadas (x, y)
int getIndex(int x, int y) {
    if (y % 2 == 0) {
        return 24 - (y * 5 + x); // Linhas pares: ordem direta
    } else {
        return 24 - (y * 5 + (4 - x)); // Linhas ímpares: ordem invertida
    }
}

// Exibe um dígito ou padrão na matriz de LEDs
void npDisplayDigit(int digit) {
    for (int coluna = 0; coluna < 5; coluna++) {
        for (int linha = 0; linha < 5; linha++) {
            int posicao = getIndex(linha, coluna); // Calcula índice do LED
            npSetLED(posicao, digits[digit][coluna][linha][0], // Componente R
                              digits[digit][coluna][linha][1], // Componente G
                              digits[digit][coluna][linha][2]); // Componente B
        }
    }
    npWrite(); // Atualiza a matriz com os novos dados
}
//////////////////////////////////////////////////////////////////////////////////////////

// Função para calcular a altitude a partir da pressão atmosférica
double calculate_altitude(double pressure)
{
    return 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 0.1903));
}

// Trecho para os botões
#include "pico/bootrom.h"
#define botaoB 6
static absolute_time_t last_interrupt_time = 0;
bool state_A = 0;
void gpio_irq_handler(uint gpio, uint32_t events)
{
    absolute_time_t now = get_absolute_time();
    int64_t diff = absolute_time_diff_us(last_interrupt_time, now);
    if (diff < 250000) return; // debounce de 250ms
    last_interrupt_time = now;
    
     if (gpio == BOTAO_A) {
        state_A = !state_A;

     } else if (gpio == botaoB) {
        reset_usb_boot(0, 0);
     } 

}

int main()
{
    // Para ser utilizado o modo BOOTSEL com botão B
    gpio_init(BOTAO_A);
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_set_dir(BOTAO_A, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_pull_up(BOTAO_A);
    gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
   // Fim do trecho para modo BOOTSEL com botão B
   
    stdio_init_all();

    init_pwm(BUZZER); // inicia buzzer

    npInit(LED_PIN); // matriz

    // Iniciando e configurando os LEDs
    gpio_init(LED_RED);
    gpio_init(LED_GREEN);
    gpio_init(LED_BLUE);
    gpio_set_dir(LED_RED, GPIO_OUT);
    gpio_set_dir(LED_GREEN, GPIO_OUT);
    gpio_set_dir(LED_BLUE, GPIO_OUT);
    gpio_put(LED_RED, 0);
    gpio_put(LED_GREEN, 0);
    gpio_put(LED_BLUE, 0);

    // I2C do Display funcionando em 400Khz.
    i2c_init(I2C_PORT_DISP, 400 * 1000);

    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_pull_up(I2C_SDA_DISP);                                        // Pull up the data line
    gpio_pull_up(I2C_SCL_DISP);                                        // Pull up the clock line
    ssd1306_t ssd;                                                     // Inicializa a estrutura do display
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT_DISP); // Inicializa o display
    ssd1306_config(&ssd);                                              // Configura o display
    ssd1306_send_data(&ssd);                                           // Envia os dados para o display

    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    // Inicializa o I2C
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicializa o BMP280
    bmp280_init(I2C_PORT);
    struct bmp280_calib_param params;
    bmp280_get_calib_params(I2C_PORT, &params);

    // Inicializa o AHT20
    aht20_reset(I2C_PORT);
    aht20_init(I2C_PORT);

    // Estrutura para armazenar os dados do sensor
    AHT20_Data data;
    int32_t raw_temp_bmp;
    int32_t raw_pressure;

    char str_tmp1[8];  // Buffer para armazenar a string
    char str_alt[8];  // Buffer para armazenar a string  
    char str_tmp2[8];  // Buffer para armazenar a string
    char str_umi[8];  // Buffer para armazenar a string      

    // Inicializar Wi-Fi
    if (cyw43_arch_init()) {
        ssd1306_fill(&ssd, 0);
        ssd1306_draw_string(&ssd, "Falha Wi-Fi", 0, 0);
        ssd1306_send_data(&ssd);
        return 1;
    }
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 5000)) {
        ssd1306_fill(&ssd, 0);
        ssd1306_draw_string(&ssd, "Falha Wi-Fi", 0, 0);
        ssd1306_send_data(&ssd);
        return 1;
    }

    // Exibir endereço IP no display
    uint32_t ip_addr = cyw43_state.netif[0].ip_addr.addr;
    uint8_t *ip = (uint8_t *)&ip_addr;
    char ip_display[24];
    snprintf(ip_display, sizeof(ip_display), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    ssd1306_fill(&ssd, 0);
    ssd1306_draw_string(&ssd, "Wi-Fi => OK", 0, 0);
    ssd1306_draw_string(&ssd, ip_display, 0, 10);
    ssd1306_send_data(&ssd);

    // Iniciar servidor HTTP
    start_http_server();

    bool cor = true;
    while (1)
    {
        // Processar eventos de rede
        cyw43_arch_poll();

        // Leitura do BMP280
        bmp280_read_raw(I2C_PORT, &raw_temp_bmp, &raw_pressure);
        int32_t temperature = bmp280_convert_temp(raw_temp_bmp, &params);
        int32_t pressure = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &params);

        // Cálculo da altitude
        altitude = calculate_altitude(pressure); // ALTERADO: Armazenar em variável global
        bmp_temperature = temperature / 100.0;   // ALTERADO: Armazenar em variável global

        printf("Pressao = %.3f kPa\n", pressure / 1000.0);
        printf("Temperatura BMP: = %.2f C\n", temperature / 100.0);
        printf("Altitude estimada: %.2f m\n", altitude);

        // Leitura do AHT20
        if (aht20_read(I2C_PORT, &data))
        {
            aht_temperature = data.temperature; // ALTERADO: Armazenar em variável global
            humidity = data.humidity;           // ALTERADO: Armazenar em variável global
            printf("Temperatura AHT: %.2f C\n", data.temperature);
            printf("Umidade: %.2f %%\n\n\n", data.humidity);
        }
        else
        {
            printf("Erro na leitura do AHT10!\n\n\n");
        }

        sprintf(str_tmp1, "%.1fC", temperature / 100.0);  // Converte o inteiro em string
        sprintf(str_alt, "%.0fm", altitude);  // Converte o inteiro em string
        sprintf(str_tmp2, "%.1fC", data.temperature);  // Converte o inteiro em string
        sprintf(str_umi, "%.1f%%", data.humidity);  // Converte o inteiro em string        
    
        //  Atualiza o conteúdo do display com animações
        ssd1306_fill(&ssd, !cor);                           // Limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);       // Desenha um retângulo
        ssd1306_line(&ssd, 3, 25, 123, 25, cor);            // Desenha uma linha
        ssd1306_line(&ssd, 3, 37, 123, 37, cor);            // Desenha uma linha
        ssd1306_draw_string(&ssd, "CEPEDI   TIC37", 8, 6);  // Desenha uma string
        ssd1306_draw_string(&ssd, "EMBARCATECH", 20, 16);   // Desenha uma string
        ssd1306_draw_string(&ssd, "BMP280  AHT10", 10, 28); // Desenha uma string
        ssd1306_line(&ssd, 63, 25, 63, 60, cor);            // Desenha uma linha vertical
        ssd1306_draw_string(&ssd, str_tmp1, 14, 41);             // Desenha uma string
        ssd1306_draw_string(&ssd, str_alt, 14, 52);             // Desenha uma string
        ssd1306_draw_string(&ssd, str_tmp2, 73, 41);             // Desenha uma string
        ssd1306_draw_string(&ssd, str_umi, 73, 52);            // Desenha uma string
        ssd1306_send_data(&ssd);                            // Atualiza o display

        sleep_ms(500);
    }

    // Desinicializar Wi-Fi
    cyw43_arch_deinit();

    return 0;
}
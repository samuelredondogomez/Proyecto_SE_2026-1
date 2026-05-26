/*
 * ================================================================
 *  RiegoAuto — Sistema automático de riego con ESP32
 *  Controla una bomba de agua (relé) y un servomotor para el movimiento
 *  según la humedad del suelo leída por un sensor analógico.
 *  El umbral de humedad se puede ajustar desde el monitor serie con la 
 *  comunicación UART
 * ================================================================
 */

// ── Librerías estándar de C ──────────────────────────────────────
#include <stdio.h>          // printf, fgets, snprintf
#include <string.h>         // strcspn, strlen

// ── Librerías de FreeRTOS (sistema operativo en tiempo real) ─────
#include "freertos/FreeRTOS.h"  // núcleo de FreeRTOS
#include "freertos/task.h"      // xTaskCreate, vTaskDelay

// ── Librerías de drivers del ESP32 ──────────────────────────────
#include "driver/gpio.h"            // control de pines digitales (relé)
#include "driver/ledc.h"            // PWM por hardware (servo)
#include "driver/adc.h"             // ADC legacy (requerido por compatibilidad)
#include "esp_adc/adc_oneshot.h"    // ADC moderno oneshot (lectura del sensor)
#include "driver/i2c.h"             // comunicación I2C (pantalla LCD)

// ================================================================
//  CONFIGURACIÓN — ADC / Sensor de humedad del suelo
// ================================================================
#define HUM_ADC_CHANNEL ADC_CHANNEL_6   // Canal ADC6 = GPIO34 del ESP32
#define ADC_UNIT        ADC_UNIT_1      // Unidad ADC1 (GPIOs 32-39)
#define ADC_SECO        1178            // Valor ADC cuando el sensor está en aire (0% humedad)
#define ADC_MOJADO      617             // Valor ADC cuando el sensor está en agua (100% humedad)

// ================================================================
//  CONFIGURACIÓN — Servo motor (válvula de agua)
//  Controlado por señal PWM de 50Hz usando el periférico LEDC
// ================================================================
#define SERVO_GPIO       GPIO_NUM_22        // Pin de señal del servo
#define SERVO_MODE       LEDC_LOW_SPEED_MODE // Modo de velocidad del PWM
#define SERVO_TIMER      LEDC_TIMER_0       // Timer 0 del LEDC
#define SERVO_CHANNEL    LEDC_CHANNEL_0     // Canal 0 del LEDC
#define SERVO_RESOLUTION LEDC_TIMER_16_BIT  // Resolución de 16 bits (0-65535)
#define SERVO_FREQ_HZ    50                 // Frecuencia PWM: 50Hz (estándar servo)
#define SERVO_MIN_US     500                // Pulso mínimo: 500µs = 0 grados
#define SERVO_MAX_US     2500               // Pulso máximo: 2500µs = 180 grados
#define SERVO_PERIOD_US  20000              // Período total: 20000µs = 50Hz

// ================================================================
//  CONFIGURACIÓN — Relé (bomba de agua)
//  Relé activo en alto: ON=1 enciende la bomba, OFF=0 la apaga
// ================================================================
#define RELE_GPIO  GPIO_NUM_23  // Pin de control del relé
#define RELE_ON    1            // Nivel alto = relé activado = bomba encendida
#define RELE_OFF   0            // Nivel bajo = relé desactivado = bomba apagada

// ================================================================
//  CONFIGURACIÓN — I2C y pantalla LCD 16x2 con módulo PCF8574
//  El PCF8574 es un expansor de puertos I2C que controla el LCD
// ================================================================
#define I2C_MASTER_SCL_IO   18          // Pin del reloj I2C (SCL)
#define I2C_MASTER_SDA_IO   19          // Pin de datos I2C (SDA)
#define I2C_MASTER_NUM      I2C_NUM_0   // Puerto I2C número 0
#define I2C_MASTER_FREQ_HZ  100000      // Velocidad I2C: 100kHz (modo estándar)
#define PCF8574_ADDR        0x27        // Dirección I2C del módulo PCF8574
#define LCD_BACKLIGHT       0x08        // Bit 3 del PCF8574 = luz de fondo del LCD
#define LCD_EN              0x04        // Bit 2 del PCF8574 = pin Enable del LCD
#define LCD_RS              0x01        // Bit 0 del PCF8574 = pin RS del LCD (0=cmd, 1=dato)

// ================================================================
//  VARIABLES GLOBALES
// ================================================================
static float hum_control = 30.0f;      // Umbral de humedad (%). Si humedad < este valor → riega
static char  uart_input[32] = "30";    // Guarda el último valor recibido por monitor serie (para mostrar en LCD)

// ================================================================
//  FUNCIONES LCD — Comunicación con pantalla vía I2C + PCF8574
// ================================================================

/*
 * pcf8574_write — Envía un byte al expansor PCF8574 por I2C.
 * El PCF8574 convierte ese byte en 8 señales digitales que
 * controlan los pines del LCD (RS, EN, backlight y datos D4-D7).
 */
esp_err_t pcf8574_write(uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();   // Crea una transacción I2C
    i2c_master_start(cmd);                           // Condición de inicio I2C
    i2c_master_write_byte(cmd,                       // Envía dirección del PCF8574
                          (PCF8574_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);          // Envía el byte de datos
    i2c_master_stop(cmd);                            // Condición de parada I2C
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                          pdMS_TO_TICKS(1000)); // Ejecuta la transacción
    i2c_cmd_link_delete(cmd);                        // Libera la memoria
    return ret;
}

/*
 * lcd_pulse — Genera un pulso en el pin Enable (EN) del LCD.
 * El LCD HD44780 necesita un pulso EN para leer cada nibble de datos.
 * Secuencia: EN=1 → espera → EN=0
 */
void lcd_pulse(uint8_t data)
{
    pcf8574_write(data | LCD_EN | LCD_BACKLIGHT);           // EN=1, backlight=1
    vTaskDelay(pdMS_TO_TICKS(1));                           // Espera 1ms
    pcf8574_write((data & ~LCD_EN) | LCD_BACKLIGHT);        // EN=0, backlight=1
    vTaskDelay(pdMS_TO_TICKS(1));                           // Espera 1ms
}

/*
 * lcd_write_nibble — Envía 4 bits (nibble) al LCD.
 * El LCD está en modo 4 bits, así que cada byte se envía en dos partes.
 * rs=0 → comando, rs=1 → dato/carácter
 */
void lcd_write_nibble(uint8_t nibble, uint8_t rs)
{
    uint8_t data = (nibble << 4) & 0xF0;    // Coloca los 4 bits en la parte alta del byte
    if (rs) data |= LCD_RS;                 // Si es dato, activa el pin RS
    lcd_pulse(data);                        // Envía con pulso EN
}

/*
 * lcd_write_byte — Envía un byte completo al LCD en dos nibbles.
 * Primero envía los 4 bits más significativos, luego los 4 menos significativos.
 * rs=0 → es un comando (ej: limpiar pantalla), rs=1 → es un carácter ASCII
 */
void lcd_write_byte(uint8_t byte, uint8_t rs)
{
    lcd_write_nibble(byte >> 4, rs);        // Envía nibble alto (bits 7-4)
    lcd_write_nibble(byte & 0x0F, rs);      // Envía nibble bajo (bits 3-0)
}

/*
 * lcd_init — Inicializa el LCD HD44780 en modo 4 bits.
 * Sigue la secuencia de inicialización del datasheet del HD44780:
 * reset por software → modo 4 bits → configuración de pantalla
 */
void lcd_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(50));          // Espera 50ms al encender (estabilización)

    // Reset por software — secuencia requerida por el datasheet
    lcd_write_nibble(0x03, 0); vTaskDelay(pdMS_TO_TICKS(5));  // Reset 1
    lcd_write_nibble(0x03, 0); vTaskDelay(pdMS_TO_TICKS(5));  // Reset 2
    lcd_write_nibble(0x03, 0); vTaskDelay(pdMS_TO_TICKS(1));  // Reset 3
    lcd_write_nibble(0x02, 0);                                 // Activa modo 4 bits

    // Configuración del LCD
    lcd_write_byte(0x28, 0);    // Function Set: 4 bits, 2 líneas, caracteres 5x8
    lcd_write_byte(0x0C, 0);    // Display Control: pantalla ON, cursor OFF, parpadeo OFF
    lcd_write_byte(0x06, 0);    // Entry Mode: cursor avanza a la derecha automáticamente
    lcd_write_byte(0x01, 0);    // Clear Display: borra toda la pantalla
    vTaskDelay(pdMS_TO_TICKS(2)); // Espera 2ms (el comando clear tarda más)
}

/*
 * lcd_set_cursor — Mueve el cursor a una posición específica del LCD.
 * col: columna (0-15), row: fila (0=primera línea, 1=segunda línea)
 * La segunda línea en el HD44780 empieza en la dirección 0x40
 */
void lcd_set_cursor(uint8_t col, uint8_t row)
{
    uint8_t row_offsets[] = {0x00, 0x40};                       // Offsets de cada fila
    lcd_write_byte(0x80 | (col + row_offsets[row]), 0);         // Comando set DDRAM address
}

/*
 * lcd_print — Escribe una cadena de texto en la posición actual del cursor.
 * Envía cada carácter ASCII uno por uno con rs=1 (modo dato)
 */
void lcd_print(const char *str)
{
    while (*str) lcd_write_byte(*str++, 1); // Envía cada carácter hasta el '\0'
}

// ================================================================
//  FUNCIONES SERVO — Control de ángulo por PWM
// ================================================================

/*
 * servo_angle_to_duty — Convierte un ángulo (0-180°) en valor de duty cycle.
 * El servo espera pulsos entre 500µs (0°) y 2500µs (180°) cada 20ms.
 * El duty se invierte porque el hardware tiene la lógica al revés.
 */
uint32_t servo_angle_to_duty(int angle)
{
    if (angle < 0)   angle = 0;     // Límite inferior
    if (angle > 180) angle = 180;   // Límite superior

    // Calcula el ancho de pulso en microsegundos según el ángulo
    uint32_t pulse_us = SERVO_MIN_US +
                        ((SERVO_MAX_US - SERVO_MIN_US) * angle) / 180;

    uint32_t max_duty    = (1 << 16) - 1;                   // Valor máximo con 16 bits = 65535
    uint32_t duty_normal = (pulse_us * max_duty) / SERVO_PERIOD_US; // Duty proporcional

    return max_duty - duty_normal;  // Se invierte porque el hardware lo requiere
}

/*
 * servo_write_angle — Mueve el servo al ángulo indicado.
 * Calcula el duty cycle y lo aplica al canal LEDC configurado.
 */
void servo_write_angle(int angle)
{
    uint32_t duty = servo_angle_to_duty(angle);
    ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, duty);     // Establece el nuevo duty
    ledc_update_duty(SERVO_MODE, SERVO_CHANNEL);        // Aplica el cambio
}

// ================================================================
//  TAREA UART — Recibe el umbral de humedad por monitor serie
// ================================================================

/*
 * uart_monitor_task — Tarea FreeRTOS que corre en paralelo al loop principal.
 * Espera que el usuario escriba un número en el monitor serie (0-100)
 * y actualiza el umbral de humedad hum_control en tiempo real.
 */
void uart_monitor_task(void *arg)
{
    char line[32];
    while (1) {
        // fgets espera una línea completa del teclado (termina con Enter)
        if (fgets(line, sizeof(line), stdin) != NULL) {
            line[strcspn(line, "\r\n")] = 0;    // Elimina el salto de línea del final
            hum_control = atof(line);            // Convierte el texto a número float
            snprintf(uart_input, sizeof(uart_input), "%.1f", hum_control); // Guarda para el LCD
        }
        vTaskDelay(pdMS_TO_TICKS(100));          // Cede el CPU cada 100ms
    }
}

// ================================================================
//  APP_MAIN — Punto de entrada principal del programa
// ================================================================
void app_main(void)
{
    // ── Inicialización ADC ───────────────────────────────────────
    // Configura el GPIO34 para leer voltaje analógico del sensor de humedad
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT };
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_new_unit(&init_config, &adc_handle);    // Inicializa la unidad ADC

    adc_oneshot_chan_cfg_t adc_config = {
        .bitwidth = ADC_BITWIDTH_12,    // Resolución 12 bits (valores 0-4095)
        .atten    = ADC_ATTEN_DB_11,    // Atenuación 11dB → rango 0 a ~3.3V
    };
    adc_oneshot_config_channel(adc_handle, HUM_ADC_CHANNEL, &adc_config);

    // ── Inicialización Servo (LEDC PWM) ──────────────────────────
    // Configura el timer PWM a 50Hz con resolución de 16 bits
    ledc_timer_config_t timer = {
        .speed_mode      = SERVO_MODE,
        .timer_num       = SERVO_TIMER,
        .duty_resolution = SERVO_RESOLUTION,
        .freq_hz         = SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,   // Selección automática de reloj
    };
    ledc_timer_config(&timer);

    // Asigna el canal PWM al GPIO del servo
    ledc_channel_config_t channel = {
        .gpio_num   = SERVO_GPIO,
        .speed_mode = SERVO_MODE,
        .channel    = SERVO_CHANNEL,
        .timer_sel  = SERVO_TIMER,
        .duty       = 0,        // Duty inicial = 0 (servo en reposo)
        .hpoint     = 0,
    };
    ledc_channel_config(&channel);

    // ── Inicialización Relé ──────────────────────────────────────
    // Configura el GPIO23 como salida digital para controlar el relé
    gpio_config_t io_conf = {
        .pin_bit_mask  = (1ULL << RELE_GPIO),   // Máscara del pin
        .mode          = GPIO_MODE_OUTPUT,       // Configurar como salida
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,      // Sin interrupción
    };
    gpio_config(&io_conf);
    gpio_set_level(RELE_GPIO, RELE_OFF);         // Inicia con la bomba apagada

    // ── Inicialización I2C y LCD ─────────────────────────────────
    // Configura el bus I2C para comunicarse con el módulo PCF8574 del LCD
    i2c_config_t i2c_conf = {
        .mode             = I2C_MODE_MASTER,        // ESP32 es el maestro I2C
        .sda_io_num       = I2C_MASTER_SDA_IO,      // GPIO19 = línea de datos
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,      // Resistencia pull-up interna en SDA
        .scl_io_num       = I2C_MASTER_SCL_IO,      // GPIO18 = línea de reloj
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,      // Resistencia pull-up interna en SCL
        .master.clk_speed = I2C_MASTER_FREQ_HZ,     // Velocidad: 100kHz
    };
    i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0);
    lcd_init();     // Inicializa la pantalla LCD

    // ── Crear tarea UART ─────────────────────────────────────────
    // Corre uart_monitor_task en paralelo para escuchar el monitor serie
    xTaskCreate(uart_monitor_task, "uart_monitor", 2048, NULL, 5, NULL);

    // ── Variables del loop principal ─────────────────────────────
    int   adc_raw = 0;      // Valor crudo del ADC (0-4095)
    float humedad = 0.0f;   // Porcentaje de humedad calculado (0-100%)
    char  lcd_msg[64];      // Buffer para el mensaje del LCD

    // ================================================================
    //  LOOP PRINCIPAL — Se ejecuta indefinidamente cada 500ms
    // ================================================================
    while (1) {

        // ── Leer sensor de humedad ───────────────────────────────
        // Lee el voltaje analógico del sensor y lo convierte a porcentaje
        adc_oneshot_read(adc_handle, HUM_ADC_CHANNEL, &adc_raw);

        // Fórmula de conversión: interpola linealmente entre ADC_SECO y ADC_MOJADO
        // Cuando adc_raw = ADC_SECO (1178) → humedad = 0%
        // Cuando adc_raw = ADC_MOJADO (617) → humedad = 100%
        humedad = (adc_raw - ADC_SECO) * (100.0f / (ADC_MOJADO - ADC_SECO));

        // Limita el valor entre 0% y 100% por si hay lecturas fuera de rango
        if (humedad < 0)   humedad = 0;
        if (humedad > 100) humedad = 100;

        // ── Control automático de riego ──────────────────────────
        // Si la humedad está por debajo del umbral → activa el riego
        if (humedad < hum_control) {
            gpio_set_level(RELE_GPIO, RELE_ON);     // Enciende la bomba
            servo_write_angle(180);                 // Abre la válvula (180°)
        } else {
            gpio_set_level(RELE_GPIO, RELE_OFF);    // Apaga la bomba
            servo_write_angle(0);                   // Cierra la válvula (0°)
        }

        // ── Mostrar datos en el LCD ──────────────────────────────
        // Línea 0: muestra la humedad actual y el umbral configurado
        lcd_set_cursor(0, 0);
        snprintf(lcd_msg, sizeof(lcd_msg),
                 "hum: %3.1f%%ctl: %s ", humedad, uart_input);
        lcd_print(lcd_msg);

        // Espera 500ms antes de la siguiente lectura
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

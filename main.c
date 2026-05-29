/*
 * ================================================================
 *  RiegoAuto — Sistema automático de riego con ESP32
 *  Controla una bomba de agua (relé) y un servomotor para el movimiento
 *  según la humedad del suelo leída por un sensor analógico.
 *  El umbral de humedad se puede ajustar desde el monitor serie con la 
 *  comunicación UART
 * ================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/i2c.h"

// ================================================================
//  CONFIGURACIÓN — ADC / Sensor de humedad del suelo
// ================================================================
#define HUM_ADC_CHANNEL ADC_CHANNEL_6
#define ADC_UNIT        ADC_UNIT_1
#define ADC_SECO        1178
#define ADC_MOJADO      617

// ================================================================
//  CONFIGURACIÓN — Servo motor
// ================================================================
#define SERVO_GPIO       GPIO_NUM_22
#define SERVO_MODE       LEDC_LOW_SPEED_MODE
#define SERVO_TIMER      LEDC_TIMER_0
#define SERVO_CHANNEL    LEDC_CHANNEL_0
#define SERVO_RESOLUTION LEDC_TIMER_16_BIT
#define SERVO_FREQ_HZ    50
#define SERVO_MIN_US     500
#define SERVO_MAX_US     2500
#define SERVO_PERIOD_US  20000

// Velocidad del barrido del servo
#define SERVO_STEP_US    10
#define LOOP_DELAY_MS    30

// ================================================================
//  CONFIGURACIÓN — Relé
// ================================================================
#define RELE_GPIO  GPIO_NUM_23
#define RELE_ON    1
#define RELE_OFF   0

// ================================================================
//  CONFIGURACIÓN — I2C y pantalla LCD 16x2 con módulo PCF8574
// ================================================================
#define I2C_MASTER_SCL_IO   18
#define I2C_MASTER_SDA_IO   19
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000
#define PCF8574_ADDR        0x27
#define LCD_BACKLIGHT       0x08
#define LCD_EN              0x04
#define LCD_RS              0x01

// ================================================================
//  VARIABLES GLOBALES
// ================================================================
static float hum_control = 30.0f;
static char  uart_input[32] = "30";

// ================================================================
//  SISTEMA DE LOGGING
// ================================================================
#define LOG_INFO(code, msg)  printf("[INFO ][%lu ms][%s] %s\n",  (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS), code, msg)
#define LOG_WARN(code, msg)  printf("[WARN ][%lu ms][%s] %s\n",  (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS), code, msg)
#define LOG_ERROR(code, msg) printf("[ERROR][%lu ms][%s] %s\n",  (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS), code, msg)

static int last_riego_activo = -1;
static int last_sensor_status_logged = -1;
// ================================================================
//  ESTADOS Y ERRORES DEL SENSOR
// ================================================================
typedef enum {
    SENSOR_OK = 0,
    SENSOR_WARN_FUERA_CALIBRACION,
    SENSOR_ERR_ADC_BAJO,
    SENSOR_ERR_ADC_ALTO
} sensor_status_t;

#define ADC_ERROR_LOW      500
#define ADC_ERROR_HIGH     1300
#define ADC_CAL_MARGIN     100

// ================================================================
//  FUNCIONES LCD
// ================================================================
esp_err_t pcf8574_write(uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCF8574_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));

    i2c_cmd_link_delete(cmd);

    return ret;
}

void lcd_pulse(uint8_t data)
{
    pcf8574_write(data | LCD_EN | LCD_BACKLIGHT);
    vTaskDelay(pdMS_TO_TICKS(1));

    pcf8574_write((data & ~LCD_EN) | LCD_BACKLIGHT);
    vTaskDelay(pdMS_TO_TICKS(1));
}

void lcd_write_nibble(uint8_t nibble, uint8_t rs)
{
    uint8_t data = (nibble << 4) & 0xF0;

    if (rs) {
        data |= LCD_RS;
    }

    lcd_pulse(data);
}

void lcd_write_byte(uint8_t byte, uint8_t rs)
{
    lcd_write_nibble(byte >> 4, rs);
    lcd_write_nibble(byte & 0x0F, rs);
}

void lcd_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));

    pcf8574_write(LCD_BACKLIGHT);
    vTaskDelay(pdMS_TO_TICKS(50));

    lcd_write_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    lcd_write_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    lcd_write_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    lcd_write_nibble(0x02, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    lcd_write_byte(0x28, 0);
    vTaskDelay(pdMS_TO_TICKS(5));

    lcd_write_byte(0x0C, 0);
    vTaskDelay(pdMS_TO_TICKS(5));

    lcd_write_byte(0x06, 0);
    vTaskDelay(pdMS_TO_TICKS(5));

    lcd_write_byte(0x01, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void lcd_set_cursor(uint8_t col, uint8_t row)
{
    uint8_t row_offsets[] = {0x00, 0x40};

    lcd_write_byte(0x80 | (col + row_offsets[row]), 0);
}

void lcd_print(const char *str)
{
    while (*str) {
        lcd_write_byte(*str++, 1);
    }
}
// ================================================================
//  FUNCIONES LCD — Mejoras de visualización
// ================================================================
void lcd_print_line(uint8_t row, const char *text)
{
    char line[17];

    snprintf(line, sizeof(line), "%-16s", text);

    lcd_set_cursor(0, row);
    lcd_print(line);
}

sensor_status_t sensor_check_status(int adc_raw)
{
    if (adc_raw <= ADC_ERROR_LOW) {
        return SENSOR_ERR_ADC_BAJO;
    }

    if (adc_raw >= ADC_ERROR_HIGH) {
        return SENSOR_ERR_ADC_ALTO;
    }

    if (adc_raw < (ADC_MOJADO - ADC_CAL_MARGIN) ||
        adc_raw > (ADC_SECO + ADC_CAL_MARGIN)) {
        return SENSOR_WARN_FUERA_CALIBRACION;
    }

    return SENSOR_OK;
}

void lcd_update_status(float humedad, float hum_control, sensor_status_t sensor_status, int riego_activo)
{
    char line[17];

    if (sensor_status == SENSOR_ERR_ADC_BAJO) {
        lcd_print_line(0, "ERROR SENSOR");
        lcd_print_line(1, "ADC MUY BAJO");
        return;
    }

    if (sensor_status == SENSOR_ERR_ADC_ALTO) {
        lcd_print_line(0, "ERROR SENSOR");
        lcd_print_line(1, "ADC MUY ALTO");
        return;
    }

    if (sensor_status == SENSOR_WARN_FUERA_CALIBRACION) {
        snprintf(line, sizeof(line), "Hum:%5.1f%% WARN", humedad);
        lcd_print_line(0, line);

        snprintf(line, sizeof(line), "Ctl:%5.1f%%", hum_control);
        lcd_print_line(1, line);
        return;
    }

    snprintf(line, sizeof(line), "Hum:%5.1f%% %s", humedad, riego_activo ? "RIE" : "OK ");
    lcd_print_line(0, line);

    snprintf(line, sizeof(line), "Ctl:%5.1f%%", hum_control);
    lcd_print_line(1, line);
}


// ================================================================
//  FUNCIONES SERVO
// ================================================================
uint32_t servo_pulse_to_duty(uint32_t pulse_us)
{
    uint32_t max_duty = (1 << 16) - 1;
    uint32_t duty = (pulse_us * max_duty) / SERVO_PERIOD_US;

    return duty;
}

void servo_write_pulse(uint32_t pulse_us)
{
    uint32_t duty = servo_pulse_to_duty(pulse_us);

    ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, duty);
    ledc_update_duty(SERVO_MODE, SERVO_CHANNEL);
}

void servo_apagar(void)
{
    ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, 0);
    ledc_update_duty(SERVO_MODE, SERVO_CHANNEL);
}

// ================================================================
//  TAREA UART
// ================================================================
void uart_monitor_task(void *arg)
{
    char line[32];

    while (1) {
        if (fgets(line, sizeof(line), stdin) != NULL) {
            line[strcspn(line, "\r\n")] = 0;

            hum_control = atof(line);

            snprintf(uart_input, sizeof(uart_input), "%.1f", hum_control);

            char log_msg[64];
            snprintf(log_msg, sizeof(log_msg), "Nuevo umbral de humedad: %.1f%%", hum_control);
            LOG_INFO("UART_01", log_msg);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
// ================================================================
//  APP_MAIN
// ================================================================
void app_main(void)
{
    // ── Inicialización ADC ───────────────────────────────────────
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT
    };

    adc_oneshot_unit_handle_t adc_handle;

    adc_oneshot_new_unit(&init_config, &adc_handle);

    adc_oneshot_chan_cfg_t adc_config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten    = ADC_ATTEN_DB_11,
    };

    adc_oneshot_config_channel(adc_handle, HUM_ADC_CHANNEL, &adc_config);
    LOG_INFO("ADC_01", "ADC inicializado en GPIO34 / ADC_CHANNEL_6");

    // ── Inicialización Servo ─────────────────────────────────────
    ledc_timer_config_t timer = {
        .speed_mode      = SERVO_MODE,
        .timer_num       = SERVO_TIMER,
        .duty_resolution = SERVO_RESOLUTION,
        .freq_hz         = SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };

    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num   = SERVO_GPIO,
        .speed_mode = SERVO_MODE,
        .channel    = SERVO_CHANNEL,
        .timer_sel  = SERVO_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };

    ledc_channel_config(&channel);

   servo_apagar();
    LOG_INFO("SERVO_01", "Servo inicializado en GPIO22");

    // ── Inicialización Relé ──────────────────────────────────────
    gpio_config_t io_conf = {
        .pin_bit_mask  = (1ULL << RELE_GPIO),
        .mode          = GPIO_MODE_OUTPUT,
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };

    gpio_config(&io_conf);
    gpio_set_level(RELE_GPIO, RELE_OFF);
    LOG_INFO("RELE_01", "Rele inicializado apagado en GPIO23");

    // ── Inicialización I2C y LCD ─────────────────────────────────
    i2c_config_t i2c_conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_MASTER_SDA_IO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_io_num       = I2C_MASTER_SCL_IO,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0);

    lcd_init();
    LOG_INFO("LCD_01", "LCD inicializada por I2C");
    // ── Crear tarea UART ─────────────────────────────────────────
    xTaskCreate(uart_monitor_task, "uart_monitor", 2048, NULL, 5, NULL);

       // ── Crear tarea UART ─────────────────────────────────────────
    xTaskCreate(uart_monitor_task, "uart_monitor", 2048, NULL, 5, NULL);
    LOG_INFO("SYS_01", "Sistema iniciado correctamente");

    // ── Variables del loop principal ─────────────────────────────
    int   adc_raw = 0;
    float humedad = 0.0f;

    uint32_t servo_pulse = SERVO_MIN_US;
    int servo_direction = 1;

    TickType_t last_lcd_update = 0;
    TickType_t lcd_update_interval = pdMS_TO_TICKS(200);

  while (1) {
    // ── Leer sensor de humedad ───────────────────────────────
    adc_oneshot_read(adc_handle, HUM_ADC_CHANNEL, &adc_raw);

    humedad = (adc_raw - ADC_SECO) * (100.0f / (ADC_MOJADO - ADC_SECO));

    if (humedad < 0) {
        humedad = 0;
    }

    if (humedad > 100) {
        humedad = 100;
    }

    // ── Verificar estado del sensor ──────────────────────────
    sensor_status_t sensor_status = sensor_check_status(adc_raw);

    int sensor_error_grave = (sensor_status == SENSOR_ERR_ADC_BAJO ||
                              sensor_status == SENSOR_ERR_ADC_ALTO);

    int riego_activo = 0;

    // ── Control automático de riego ──────────────────────────
    if (sensor_error_grave) {
    gpio_set_level(RELE_GPIO, RELE_OFF);

    servo_apagar();

    servo_pulse = SERVO_MIN_US;
    servo_direction = 1;

    riego_activo = 0;

} else if (humedad < hum_control) {
    gpio_set_level(RELE_GPIO, RELE_ON);

    riego_activo = 1;

    servo_write_pulse(servo_pulse);

    if (servo_direction == 1) {
        servo_pulse += SERVO_STEP_US;

        if (servo_pulse >= SERVO_MAX_US) {
            servo_pulse = SERVO_MAX_US;
            servo_direction = -1;
        }
    } else {
        servo_pulse -= SERVO_STEP_US;

        if (servo_pulse <= SERVO_MIN_US) {
            servo_pulse = SERVO_MIN_US;
            servo_direction = 1;
        }
    }

} else {
    gpio_set_level(RELE_GPIO, RELE_OFF);

    servo_apagar();

    servo_pulse = SERVO_MIN_US;
    servo_direction = 1;

    riego_activo = 0;
}
// Log de cambio de estado de riego
if (riego_activo != last_riego_activo) {
    last_riego_activo = riego_activo;

    if (riego_activo) {
        LOG_INFO("IRR_01", "Riego activado: rele ON y servo en barrido");
    } else {
        LOG_INFO("IRR_02", "Riego desactivado: rele OFF y servo apagado");
    }
}

    // ── Mostrar datos en el LCD ──────────────────────────────
    if ((xTaskGetTickCount() - last_lcd_update) >= lcd_update_interval) {
        last_lcd_update = xTaskGetTickCount();

        lcd_update_status(humedad, hum_control, sensor_status, riego_activo);
    }

    vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));


// Log de cambio de estado del sensor
if (sensor_status != last_sensor_status_logged) {
    last_sensor_status_logged = sensor_status;

    if (sensor_status == SENSOR_OK) {
        LOG_INFO("SENS_00", "Sensor en rango normal");
    } else if (sensor_status == SENSOR_WARN_FUERA_CALIBRACION) {
        LOG_WARN("SENS_01", "Sensor fuera del rango de calibracion");
    } else if (sensor_status == SENSOR_ERR_ADC_BAJO) {
        LOG_ERROR("SENS_02", "Error: ADC demasiado bajo");
    } else if (sensor_status == SENSOR_ERR_ADC_ALTO) {
        LOG_ERROR("SENS_03", "Error: ADC demasiado alto");
    }
}
}
}

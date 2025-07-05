#include <stdio.h>
#include <driver/gpio.h> // GPIO driver for input/output
#include <driver/ledc.h> // LEDC driver for PWM
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

//-------- Define GPIO pins ----------
//Define GPIOs to control a motor driver (L298N)
#define PIN_TURN GPIO_NUM_25
#define PIN_ENB GPIO_NUM_26

// Define GPIOs to receive encoder signals
#define PIN_ENC_A GPIO_NUM_36
#define PIN_ENC_B GPIO_NUM_39

#define TIMER LEDC_TIMER_0 // Timer que usara el PWM
#define TIMER_BIT LEDC_TIMER_12_BIT
#define FREQ_TIMER 10000
#define TIMER_SPEED_MODE LEDC_HIGH_SPEED_MODE

#define CHANNEL_EN LEDC_CHANNEL_0 // Channel for the PWM signal

//Configurar el timer para PWM
void config_timer(){
    //Configuracion del Timer
    ledc_timer_config_t ledcTimer = {};
    ledcTimer.speed_mode = TIMER_SPEED_MODE;
    ledcTimer.timer_num = TIMER;
    ledcTimer.duty_resolution = TIMER_BIT;
    ledcTimer.freq_hz = FREQ_TIMER;
    ledcTimer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledcTimer);
}

extern "C" void app_main(){
    //Configuración de Perifericos
    gpio_config_t motor_turn_config = {};
    motor_turn_config.pin_bit_mask = (1ULL << PIN_TURN);
    motor_turn_config.mode = GPIO_MODE_OUTPUT;
    motor_turn_config.pull_up_en = GPIO_PULLUP_DISABLE;
    motor_turn_config.pull_down_en = GPIO_PULLDOWN_ENABLE;
    motor_turn_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&motor_turn_config);

    gpio_config_t motor_enb_config = {};
    motor_enb_config.pin_bit_mask = (1ULL << PIN_ENB);
    motor_enb_config.mode = GPIO_MODE_OUTPUT;
    motor_enb_config.pull_up_en = GPIO_PULLUP_DISABLE;
    motor_enb_config.pull_down_en = GPIO_PULLDOWN_ENABLE;
    motor_enb_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&motor_enb_config);

    // Configure the LEDC timer for PWM
    config_timer();
    // Configure the LEDC channel for the motor ENB
    ledc_channel_config_t channel_config = {};
    channel_config.gpio_num = PIN_ENB; // GPIO for PWM output
    channel_config.speed_mode = TIMER_SPEED_MODE; // Speed mode for the channel
    channel_config.channel = CHANNEL_EN; // Channel number
    channel_config.timer_sel = TIMER; // Timer to use
    channel_config.duty = 0; // Initial duty cycle (0 means off)
    channel_config.hpoint = 0; // Hpoint value

    ledc_channel_config(&channel_config); // Configure the channel

    //loop
    while(1){
        // Set motor ENB high
        ledc_set_duty(TIMER_SPEED_MODE, CHANNEL_EN, 4095); // Set duty cycle to maximum (100%)
        ledc_update_duty(TIMER_SPEED_MODE, CHANNEL_EN); // Update the duty cycle
        
        // Set motor TURN high (CCW)
        gpio_set_level(PIN_TURN, 1);
        /*
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second
        // Set motor TURN low (CW)
        gpio_set_level(PIN_TURN, 0);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second
        */
    }
}
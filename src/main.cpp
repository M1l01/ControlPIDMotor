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

    //loop
    while(1){
        // Set motor ENB high
        gpio_set_level(PIN_ENB, 1);
        // Set motor TURN high (CCW)
        gpio_set_level(PIN_TURN, 1);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second
        // Set motor TURN low (CW)
        gpio_set_level(PIN_TURN, 0);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second
    }

}
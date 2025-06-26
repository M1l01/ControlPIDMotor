#include <stdio.h>
#include <driver/gpio.h> // GPIO driver for input/output
#include <driver/ledc.h> // LEDC driver for PWM
#include <esp_err.h>

//-------- Define GPIO pins ----------
//Define GPIOs to control a motor driver (L298N)
#define PIN_IN4 GPIO_NUM_33
#define PIN_IN3 GPIO_NUM_25
#define PIN_ENB GPIO_NUM_26

// Define GPIOs to receive encoder signals
#define PIN_ENC_A GPIO_NUM_36
#define PIN_ENC_B GPIO_NUM_39

extern "C" void app_main(){
    //Configuración de Perifericos
    gpio_config_t motor_in4_config = {};
    motor_in4_config.pin_bit_mask = (1ULL << PIN_IN4);
    motor_in4_config.mode = GPIO_MODE_OUTPUT;
    motor_in4_config.pull_up_en = GPIO_PULLUP_DISABLE;
    motor_in4_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    motor_in4_config.intr_type = GPIO_INTR_DISABLE;
    esp_err_t ret = gpio_config(&motor_in4_config);
    if (ret != ESP_OK) {
        printf("Error configuring GPIO %d: %s\n", PIN_IN4, esp_err_to_name(ret));
        return;
    }

    gpio_config_t motor_in3_config = {};
    motor_in3_config.pin_bit_mask = (1ULL << PIN_IN3);
    motor_in3_config.mode = GPIO_MODE_OUTPUT;
    motor_in3_config.pull_up_en = GPIO_PULLUP_DISABLE;
    motor_in3_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    motor_in3_config.intr_type = GPIO_INTR_DISABLE;
    ret = gpio_config(&motor_in3_config);
    if (ret != ESP_OK) {
        printf("Error configuring GPIO %d: %s\n", PIN_IN3, esp_err_to_name(ret));
        return;
    }

    //loop
    while(1){
        // Set motor IN4 high and IN3 low to move forward
        gpio_set_level(PIN_IN4, 1);
        gpio_set_level(PIN_IN3, 0);

    }

}
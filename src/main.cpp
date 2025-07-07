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

// Global Variables

//Variables to handle the encoder detection
int16_t encoder_count = 0; // Variable to count encoder ticks (11 ticks per revolution)
bool encoder_direction = true; // Variable to identify the direction of the motor (true for CCW, false for CW)

//-------- Function Prototypes --------
esp_err_t init_motor_gpio(gpio_num_t control_pin);
void init_timer();
esp_err_t init_irs(void);
void isr_handler(void *arg);

//-------- Main Function --------
extern "C" void app_main(){
    // Configuración de Perifericos

    // Configure the GPIO for motor TURN
    init_motor_gpio(PIN_TURN);

    // Configure the GPIO for motor PWM control (ENB)
    init_motor_gpio(PIN_ENB);

    // Configure the GPIO for encoder channel B
    gpio_config_t enc_b_config = {};
    enc_b_config.pin_bit_mask = (1ULL << PIN_ENC_B);
    enc_b_config.mode = GPIO_MODE_INPUT; // Set as input
    enc_b_config.pull_up_en = GPIO_PULLUP_ENABLE; // Enable pull-up resistor
    enc_b_config.pull_down_en = GPIO_PULLDOWN_DISABLE; // Disable pull-down resistor
    gpio_config(&enc_b_config);

    // Configure the LEDC timer for PWM
    init_timer();

    // Configure the LEDC channel for the motor ENB
    ledc_channel_config_t channel_config = {};
    channel_config.gpio_num = PIN_ENB; // GPIO for PWM output
    channel_config.speed_mode = TIMER_SPEED_MODE; // Speed mode for the channel
    channel_config.channel = CHANNEL_EN; // Channel number
    channel_config.timer_sel = TIMER; // Timer to use
    channel_config.duty = 0; // Initial duty cycle (0 means off)
    channel_config.hpoint = 0; // Hpoint value
    ledc_channel_config(&channel_config); // Configure the channel

    // Initialize the encoder interrupt of channel A
    init_irs();

    //loop
    while(1){
        // Set motor TURN high (CCW)
        gpio_set_level(PIN_TURN, 1);
        
        // Set motor ENB high in 60% duty cycle
        ledc_set_duty(TIMER_SPEED_MODE, CHANNEL_EN, 2457); // Set duty cycle to maximum (60%)
        ledc_update_duty(TIMER_SPEED_MODE, CHANNEL_EN); // Update the duty cycle

        //Print the encoder count and direction
        printf("Encoder Count: %d\n", encoder_count);
        printf("Revoluciones: %d\n", encoder_count / 11);
        printf("Direction: %s\n", encoder_direction ? "CCW" : "CW");

        vTaskDelay(pdMS_TO_TICKS(10)); // Whatchdog delay to prevent task starvation
    }
}

//-------- Function Definitions --------

// Initilize the GPIOs for the control of the motor
esp_err_t init_motor_gpio(gpio_num_t control_pin) {
    gpio_config_t motor_turn_config = {};
    motor_turn_config.pin_bit_mask = (1ULL << control_pin);
    motor_turn_config.mode = GPIO_MODE_OUTPUT;
    motor_turn_config.pull_up_en = GPIO_PULLUP_DISABLE;
    motor_turn_config.pull_down_en = GPIO_PULLDOWN_ENABLE;
    motor_turn_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&motor_turn_config);
    return ESP_OK;
}

//Configurar el timer para PWM
void init_timer(){
    //Configuracion del Timer
    ledc_timer_config_t ledcTimer = {};
    ledcTimer.speed_mode = TIMER_SPEED_MODE; 
    ledcTimer.timer_num = TIMER;
    ledcTimer.duty_resolution = TIMER_BIT;
    ledcTimer.freq_hz = FREQ_TIMER;
    ledcTimer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledcTimer);
}

// Initialize the interrput to lecture of encoders
esp_err_t init_irs(void) {
    gpio_config_t enc_a_config = {};
    enc_a_config.pin_bit_mask = (1ULL << PIN_ENC_A);
    enc_a_config.mode = GPIO_MODE_INPUT;
    enc_a_config.pull_up_en = GPIO_PULLUP_ENABLE;
    enc_a_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    enc_a_config.intr_type = GPIO_INTR_POSEDGE; // Interrupt on rising edge
    gpio_config(&enc_a_config);

    // Installing the interrupts service
    gpio_install_isr_service(0);

    gpio_isr_handler_add(PIN_ENC_A, isr_handler, NULL);

    return ESP_OK;
}

// Function to count encoder tiks and identify direction
// This function will be called in the ISR
void isr_handler(void *arg) {
    //Increment the encoder count
    encoder_count++;
    // Check the state of channel B to determine direction
    int level_b = gpio_get_level(PIN_ENC_B);
    if (level_b == 1) {
        // If channel B is high, the direction is CCW
        encoder_direction = true;
    } else {
        // If channel B is low, the direction is CW
        encoder_direction = false;
    }
}
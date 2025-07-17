#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <driver/gpio.h> // GPIO driver for input/output
#include <driver/ledc.h> // LEDC driver for PWM
#include <driver/uart.h> // UART driver for serial communication
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <stdatomic.h>
#include <esp_log.h>

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

//Constants for the encoder
#define TPR 1496
#define N_SAMPLES 10 // Number of samples to average the velocity
#define SAMPLE_TIME_MS 100 // Time in milliseconds to sample the velocity

//UART Configuration
#define UART_PORT_NUM UART_NUM_0 // UART port number
#define UART_BAUD_RATE 115200 // Baud rate for UART communication
#define UART_DATA_BITS UART_DATA_8_BITS // Data bits configuration
#define UART_PARITY UART_PARITY_DISABLE // Parity configuration
#define UART_STOP_BITS UART_STOP_BITS_1 // Stop bits configuration
#define UART_FLOW_CTRL UART_HW_FLOWCTRL_DISABLE // Flow control configuration
#define UART_SOURCE_CLK UART_SCLK_DEFAULT // Source clock for UART

#define BUF_SIZE 1024

double buffer_velocity[N_SAMPLES] = {0}; // Buffer to store the velocity samples
int buffer_index = 0; // Index to track the current position in the buffer

// Global Variables

// Variables to handle the encoder detection
atomic_uint_fast32_t ticks_count = 0; // Protected variable to count encoder ticks (1496 ticks per revolution)
bool encoder_direction = true; // Variable to identify the direction of the motor (true for CCW, false for CW)

// Variables for simulink communication
volatile int simulink_command = 0; // Command received from Simulink
volatile bool new_command_received = false; // Flag to indicate if a new command has been received

// Variables for PID control
float cv;
float cv_prev1;
float error;
float error_prev1;
float error_prev2;

double kp = 0.186493; // Proportional gain
double ki = 2.105121; // Integral gain
double kd = 0.0; // Derivative gain
float Tm = 0.1; // Sample time in seconds

float setpoint = 300.0; // Desired setpoint for the motor speed in RPM

//-------- Function Prototypes --------
esp_err_t init_motor_gpio(gpio_num_t control_pin);
void init_timer();
esp_err_t init_irs(void);
void isr_handler_a(void *arg);
void isr_handler_b(void *arg);
esp_err_t init_uart(void);
void uart_receive_task(void *arg);
void set_motor_pwm(int activate_motor);

//-------- Main Function --------
extern "C" void app_main(){
    // Configuración de Perifericos

    // Configure the GPIO for motor TURN
    init_motor_gpio(PIN_TURN);

    // Configure the GPIO for motor PWM control (ENB)
    init_motor_gpio(PIN_ENB);

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

    // Initialize the UART for simulink communication
    init_uart();

    // Initialize the encoder interrupt of channel A
    init_irs();

    //Create a task to receive data from UART
    xTaskCreate(uart_receive_task, "uart_receive_task", 4096, NULL, 10, NULL);

    // Local Variables
    // Velocity measure variables
    uint32_t current_ticks = 0;
    uint32_t prev_ticks = 0;
    uint32_t delta_ticks = 0;

    float velocity_rpm; // Variable to store the velocity in RPM

    // Set motor TURN high (CCW)
    gpio_set_level(PIN_TURN, 1);
    set_motor_pwm(0); // Set the motor PWM to 0 (off)
    //loop
    while(1){

        if (new_command_received) {
            new_command_received = false; // Reset the flag
            set_motor_pwm(simulink_command); // Set the motor PWM based on the command received
        }
        
        // Read the current ticks count
        current_ticks = atomic_load(&ticks_count); // Atomically read the current ticks count
        
        // Calculate the difference in ticks since the last measurement
        delta_ticks = current_ticks - prev_ticks; // Calculate the difference in ticks
        prev_ticks = current_ticks; // Update the previous ticks count
        
        // Calculate the velocity in RPM
        double revolutions = (double)delta_ticks / TPR; // Convert ticks to revolutions
        double time_minutes = SAMPLE_TIME_MS / 60000.0; // Convert milliseconds to minutes
        velocity_rpm = revolutions / time_minutes; // Calculate RPM
        
        buffer_velocity[buffer_index] = velocity_rpm; // Store the velocity in the buffer
        buffer_index = (buffer_index + 1) % N_SAMPLES; // Update the buffer index
        
        // Calculate the average velocity from the buffer
        double sum_velocity = 0.0;
        for (int i = 0; i < N_SAMPLES; i++) {
            sum_velocity += buffer_velocity[i]; // Sum all the velocities in the buffer
        }

        float average_velocity = sum_velocity / N_SAMPLES; // Calculate the average velocity

        // PID Control
        // Calculate the Error
        error = setpoint - average_velocity;

        //Calculate the Difference ecuations
        cv = cv_prev1 + (kp + (kd/Tm))*error + (-kp + (ki*Tm) - 2*(kd/Tm))*error_prev1 + (kd/Tm)*error_prev2;
        // recursively update the previous values
        cv_prev1 = cv;
        error_prev2 = error_prev1;
        error_prev1 = error;

        // CV saturation
        if (cv > 500.0) {
            cv = 500.0;
        } else if (cv < 0.0) {
            cv = 0.0;
        }

        // Set the motor PWM based on the PID control output
        ledc_set_duty(TIMER_SPEED_MODE, CHANNEL_EN, (uint32_t)(cv * (4095.0 / 500.0))); // Scale cv to 12-bit duty cycle
        ledc_update_duty(TIMER_SPEED_MODE, CHANNEL_EN); // Update the duty cycle

        printf("Setpoint: %.2f RPM, Average Velocity: %.2f RPM, Control Value: %.2f\n", setpoint, average_velocity, cv);

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_TIME_MS)); // Delay for SAMPLE_TIME_MS milliseconds
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
    // Configure encoder channel A
    gpio_config_t enc_a_config = {};
    enc_a_config.pin_bit_mask = (1ULL << PIN_ENC_A);
    enc_a_config.mode = GPIO_MODE_INPUT;
    enc_a_config.pull_up_en = GPIO_PULLUP_ENABLE;
    enc_a_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    enc_a_config.intr_type = GPIO_INTR_ANYEDGE; // Interrupt on RISING and FALLING edges
    gpio_config(&enc_a_config);

    // Configure encoder channel B
    gpio_config_t enc_b_config = {};
    enc_b_config.pin_bit_mask = (1ULL << PIN_ENC_B);
    enc_b_config.mode = GPIO_MODE_INPUT;
    enc_b_config.pull_up_en = GPIO_PULLUP_ENABLE;
    enc_b_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    enc_b_config.intr_type = GPIO_INTR_ANYEDGE; // Interrupt on RISING and FALLING edges
    gpio_config(&enc_b_config);

    // Installing the interrupts service
    gpio_install_isr_service(0);

    gpio_isr_handler_add(PIN_ENC_A, isr_handler_a, NULL); // Add ISR handler for channel A
    gpio_isr_handler_add(PIN_ENC_B, isr_handler_b, NULL); // Add ISR handler for channel B

    return ESP_OK;
}

// Function to count encoder tiks (channel A)
// This function will be called in the ISR
void isr_handler_a(void *arg) {
    //Increment the encoder count
    atomic_fetch_add(&ticks_count, 1); // Atomically increment the ticks count
}

// Function to count encoder tiks (channel B)
// This function will be called in the ISR
void isr_handler_b(void *arg) {
    // Check the state of channel A to determine direction
    atomic_fetch_add(&ticks_count, 1); // Atomically increment the ticks count
}

esp_err_t init_uart(void) {
    uart_config_t uart_config = {};
    uart_config.baud_rate = UART_BAUD_RATE; // Set the baud rate
    uart_config.data_bits = UART_DATA_BITS; // Set the data bits
    uart_config.parity = UART_PARITY; // Set the parity
    uart_config.stop_bits = UART_STOP_BITS; // Set the stop bits
    uart_config.flow_ctrl = UART_FLOW_CTRL; // Set the flow control
    uart_config.source_clk = UART_SOURCE_CLK; // Set the source clock

    // Install the UART driver
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config)); // Configure the UART parameters
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)); // Set the UART pins
    
    return ESP_OK;
}

// Task to receive data from Simulink via UART
void uart_receive_task(void *arg) {
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE); // Buffer to store received data
    
    while (1) {
        // Read data from UART
        int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);

        if (len > 0) {
            for (int i = 0; i < len; i++) {
                if (data[i] == '\n' || data[i] == '\r') {
                    if (data[i-1] == 0 || data[i-1] == 1){
                        //printf("%d\n", data[i-1]);
                        simulink_command = (data[i-1] == 1) ? 1 : 0;
                        new_command_received = true;
                    }
                }
            }
        }
    }
    free(data);
}

void set_motor_pwm(int activate_motor) {
    // Clamp the speed percentage to the range [0, 100]
    uint32_t duty_cycle = (activate_motor == 1) ? 4095 : 0; // Convert to 12-bit duty cycle
    
    // Set PWM duty cycle
    ledc_set_duty(TIMER_SPEED_MODE, CHANNEL_EN, duty_cycle); // Set the duty cycle
    ledc_update_duty(TIMER_SPEED_MODE, CHANNEL_EN); // Update the duty cycle
}


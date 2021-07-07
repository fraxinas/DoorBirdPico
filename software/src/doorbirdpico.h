#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/structs/pwm.h"

#define UART_ID uart0
#define BAUD_RATE 9600
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define UART_INIT_SEQUENCE "MTT_14_001_001\r\n"

#define PIN_REED_DOOR 2
#define PIN_REED_LETTER 3

#define PIN_A_KEY 6
#define PIN_A_BLUE 7
#define PIN_A_GREEN 8
#define PIN_A_RED 9

#define PIN_B_KEY 11
#define PIN_B_BLUE 12
#define PIN_B_GREEN 13
#define PIN_B_RED 14

#define PIN_C_KEY 22
#define PIN_C_BLUE 17
#define PIN_C_GREEN 16
#define PIN_C_RED 19

#define PIN_RELAY_OUT 28

#define PIN_RELAY_IN1 27
#define PIN_RELAY_IN2 26

#define LONG_PRESS_MS 800
#define DELAY_LEAVING_LOCK_MS 15000
#define DELAY_ACTUATED_LOCK_MS 2500
#define BUZZER_MS 5000

#define PIN_STATUS 25

#define KEY_A_CODE "101#"
#define KEY_B_CODE "102#"
#define LOCK_CODE "103#"
#define UNLOCK_CODE "104#"
#define LETTER_CODE "105#"

#define DEBOUNCE_MS 15

#define MAX_PWM_LEDS 6
#define PWM_LED_INIT_DIV 4.0f
#define PWM_LED_CYCLES 8

#define DO_UNLOCK_ON_OPEN false

#define RS485_ID uart1
#define RS485_TX_PIN 20
#define RS485_RX_PIN 21
#define RS485_DRIVER_ENABLE_PIN 18
#define RS485_BUF_LEN 64
#define RS485_KEY_LEN 10
#define RS485_READ_TIMEOUT 1000000/96*(RS485_BUF_LEN+1)

typedef enum{
    RS485_K_NONE,
    RS485_K_LOCKSTATE,
    RS485_K_LOCKACTION,
    RS485_K_BUZZER,
    RS485_K_DOORREED,
    RS485_K_LETTERREED,
    RS485_K_BRIGHTNESS,
    RS485_K_COLOR,
    RS485_K_LAST,
} rs485_key_t;

static const char *rs485_key_str[] = {
    "",
    " LOCKSTATE",
    "LOCKACTION",
    "    BUZZER",
    "  DOORREED",
    "LETTERREED",
    "BRIGHTNESS",
    "     COLOR"
};

typedef enum
{
    DOOR_UNKNOWN,
    DOOR_CLOSED,
    DOOR_OPEN,
} door_state_t;

typedef enum
{
    LOCK_S_UNKNOWN,
    LOCK_S_LOCKED,
    LOCK_S_UNLOCKED,
    LOCK_S_WAITING_FOR_UNLOCK,
} lock_state_t;

typedef enum
{
    LOCK_A_OFF,
    LOCK_A_LOCKING,
    LOCK_A_LOCKED,
    LOCK_A_LEAVING,
    LOCK_A_UNLOCKING,
    LOCK_A_UNLOCKED,
    LOCK_A_ACTUATED,
    LOCK_A_PRESSED,
} lock_action_t;

static const char *door_state_str[] = {
    "UNKNOWN",
    "CLOSED",
    "OPEN",
};

static const char *lock_state_str[] = {
    "UNKNOWN",
    "LOCKED",
    "UNLOCKED",
    "WAITING_FOR_UNLOCK",
};

static const char *lock_action_str[] = {
    "OFF",
    "LOCKING",
    "LOCKED",
    "LEAVING",
    "UNLOCKING",
    "UNLOCKED",
    "ACTUATED",
    "PRESSED",
};

const uint16_t pwm_gamma[] = {
      0,   662,  1320,  1974,  2623,  3267,  3908,  4543,
   5175,  5802,  6424,  7043,  7657,  8266,  8872,  9473,
  10070, 10662, 11251, 11835, 12415, 12990, 13562, 14129,
  14693, 15252, 15807, 16358, 16905, 17447, 17986, 18521,
  19051, 19578, 20101, 20619, 21134, 21645, 22152, 22654,
  23153, 23649, 24140, 24627, 25111, 25590, 26066, 26539,
  27007, 27471, 27932, 28389, 28843, 29292, 29738, 30181,
  30619, 31054, 31486, 31913, 32338, 32758, 33175, 33589,
  33999, 34405, 34808, 35207, 35603, 35996, 36385, 36770,
  37153, 37531, 37907, 38279, 38647, 39013, 39375, 39733,
  40089, 40441, 40790, 41135, 41478, 41817, 42153, 42486,
  42815, 43142, 43465, 43785, 44102, 44416, 44727, 45035,
  45340, 45641, 45940, 46236, 46529, 46818, 47105, 47389,
  47670, 47948, 48223, 48495, 48764, 49031, 49295, 49555,
  49813, 50069, 50321, 50571, 50818, 51062, 51304, 51542,
  51778, 52012, 52243, 52471, 52696, 52919, 53140, 53357,
  53572, 53785, 53995, 54203, 54408, 54610, 54810, 55008,
  55203, 55396, 55586, 55774, 55960, 56143, 56324, 56503,
  56679, 56853, 57024, 57194, 57361, 57525, 57688, 57848,
  58007, 58163, 58316, 58468, 58618, 58765, 58910, 59053,
  59195, 59334, 59471, 59606, 59739, 59870, 59999, 60126,
  60251, 60374, 60495, 60614, 60732, 60847, 60961, 61072,
  61182, 61290, 61397, 61501, 61604, 61705, 61804, 61902,
  61998, 62092, 62184, 62275, 62364, 62451, 62537, 62621,
  62704, 62785, 62865, 62943, 63019, 63094, 63167, 63239,
  63310, 63379, 63446, 63512, 63577, 63640, 63702, 63763,
  63822, 63880, 63937, 63992, 64046, 64099, 64150, 64200,
  64249, 64297, 64344, 64389, 64433, 64476, 64518, 64559,
  64599, 64637, 64675, 64711, 64747, 64781, 64815, 64847,
  64878, 64909, 64938, 64967, 64995, 65024, 65052, 65081,
  65109, 65138, 65166, 65195, 65223, 65251, 65280, 65308,
  65337, 65365, 65394, 65422, 65451, 65479, 65508, 65535
};

typedef struct pwm_led {
    bool active;
    uint key_in;
    uint led_out;
    uint slice_num;
    uint8_t val_init;
    uint8_t val_end;
    uint8_t fade;
    uint8_t cycle;
} pwm_led_t;

door_state_t door_state;
lock_state_t lock_state;
pwm_config pwm_conf;
pwm_led_t pwm_leds[MAX_PWM_LEDS];
bool pwm_fading;
uint8_t global_brightness;

// received data on UART0 or RS485
void on_uart_rx();

// process data from Doorbird
void on_doorbird_rx();

// Send data to Doorbird via UART
void uart_send_code(char *code);

// Get key from RS485 command
rs485_key_t rs485_key_from_str (char *msg);

// process data from knxadapter
void on_rs485_rx();

// Send data to knxadapter via RS485
void rs485_send_msg(rs485_key_t key, const char* val);

// Return amount of active PWM LEDs
uint8_t get_active_led_count();

// Button pressed
void key_callback(uint gpio);

// PWM wrap callback, de/in-crement & set RGB values
void on_pwm_wrap();

// get brightness-adjusted 16-bit PWM gamma value from 8-bit component
uint16_t get_gamma(uint8_t value);

// Set color of inside button
void set_key_c_color(lock_action_t action);

// Turn off buzzer callback
int64_t buzzer_alarm_cb(alarm_id_t id, void *user_data);

// function when lock state relais 2 changed
int64_t actuated_lock_alarm_cb(alarm_id_t id, void *user_data);

// function called after leaving lock delay
int64_t delayed_lock_alarm_cb(alarm_id_t id, void *user_data);

// function which sends (un-)locking codes to UART and RS485
void send_locking_action(lock_action_t action);

// check whether inside key was pressed short or long and lock accordingly
int64_t lock_key_pressed_cb(alarm_id_t id, void *user_data);

// interrupt service routine, debouncer and function router for all inputs
void sensor_input(uint gpio, uint32_t events);

// Setup function for interrupts
int setup_irq();

// Setup function for Inputs and Outputs
int setup_gpio();

// Setup function for RGB PWM
int setup_pwm();

// Setup function for UART communication
int setup_uart(bool enable_rs485);

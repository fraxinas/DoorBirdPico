#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/structs/pwm.h"

#define UART_ID uart1
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
#define PIN_C_BLUE 21
#define PIN_C_GREEN 20
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

#define RS485_ID uart0
#define RS485_TX_PIN 16
#define RS485_RX_PIN 17
#define RS485_BUF_LEN 64
#define RS485_KEY_LEN 10
#define RS485_READ_TIMEOUT 1000000/96*(RS485_BUF_LEN+1)

typedef enum{
    RS485_K_NONE,
    RS485_K_LOCKSTATE,
    RS485_K_LOCKACTION,
    RS485_K_BUZZER,
    RS485_K_BRIGHTNESS,
    RS485_K_DOORREED,
    RS485_K_LETTERREED,
    RS485_K_LAST,
} rs485_key_t;

static const char *rs485_key_str[] = {
    "",
    " LOCKSTATE",
    "LOCKACTION",
    "    BUZZER",
    "BRIGHTNESS",
    "  DOORREED",
    "LETTERREED"
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
  65535,    65508,    65479,    65451,    65422,    65394,    65365,    65337,
  65308,    65280,    65251,    65223,    65195,    65166,    65138,    65109,
  65081,    65052,    65024,    64995,    64967,    64938,    64909,    64878,
  64847,    64815,    64781,    64747,    64711,    64675,    64637,    64599,
  64559,    64518,    64476,    64433,    64389,    64344,    64297,    64249,
  64200,    64150,    64099,    64046,    63992,    63937,    63880,    63822,
  63763,    63702,    63640,    63577,    63512,    63446,    63379,    63310,
  63239,    63167,    63094,    63019,    62943,    62865,    62785,    62704,
  62621,    62537,    62451,    62364,    62275,    62184,    62092,    61998,
  61902,    61804,    61705,    61604,    61501,    61397,    61290,    61182,
  61072,    60961,    60847,    60732,    60614,    60495,    60374,    60251,
  60126,    59999,    59870,    59739,    59606,    59471,    59334,    59195,
  59053,    58910,    58765,    58618,    58468,    58316,    58163,    58007,
  57848,    57688,    57525,    57361,    57194,    57024,    56853,    56679,
  56503,    56324,    56143,    55960,    55774,    55586,    55396,    55203,
  55008,    54810,    54610,    54408,    54203,    53995,    53785,    53572,
  53357,    53140,    52919,    52696,    52471,    52243,    52012,    51778,
  51542,    51304,    51062,    50818,    50571,    50321,    50069,    49813,
  49555,    49295,    49031,    48764,    48495,    48223,    47948,    47670,
  47389,    47105,    46818,    46529,    46236,    45940,    45641,    45340,
  45035,    44727,    44416,    44102,    43785,    43465,    43142,    42815,
  42486,    42153,    41817,    41478,    41135,    40790,    40441,    40089,
  39733,    39375,    39013,    38647,    38279,    37907,    37531,    37153,
  36770,    36385,    35996,    35603,    35207,    34808,    34405,    33999,
  33589,    33175,    32758,    32338,    31913,    31486,    31054,    30619,
  30181,    29738,    29292,    28843,    28389,    27932,    27471,    27007,
  26539,    26066,    25590,    25111,    24627,    24140,    23649,    23153,
  22654,    22152,    21645,    21134,    20619,    20101,    19578,    19051,
  18521,    17986,    17447,    16905,    16358,    15807,    15252,    14693,
  14129,    13562,    12990,    12415,    11835,    11251,    10662,    10070,
   9473,    8872,    8266,    7657,    7043,    6424,    5802,    5175,
   4543,    3908,    3267,    2623,    1974,    1320,    662,    0
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
bool pwm_running;

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

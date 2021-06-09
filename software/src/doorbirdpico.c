#include "pico/stdlib.h"
#include <stdio.h>
#include "pico/time.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"

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

#define DO_UNLOCK_ON_OPEN false

typedef enum
{
    DOOR_UNKNOWN,
    DOOR_CLOSED,
    DOOR_OPEN
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

static const char *lock_state_str[] = {
    "UNKNOWN",
    "LOCKED",
    "UNLOCKED",
    "WAITING_FOR_UNLOCK",
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
    uint8_t idle_value;
    uint8_t fade;
    uint8_t count;
} pwm_led_t;

door_state_t door_state;
lock_state_t lock_state;
pwm_config pwm_conf;
pwm_led_t pwm_leds[MAX_PWM_LEDS];

void on_uart_rx()
{
    printf("on_uart_rx... ");
    while (uart_is_readable(UART_ID))
    {
        uint8_t ch = uart_getc(UART_ID);
        printf("0x%02X ", ch);
        if (ch == 0x02)
        {
            ch = uart_getc(UART_ID);
            printf("%02X\n", ch);
            if (ch == 0x01)
            {
                if (uart_is_writable(UART_ID))
                {
                    printf("received startup bytes from Doorbird. Sending UART_INIT_SEQUENCE...\n");
                    uart_puts(uart0, UART_INIT_SEQUENCE);
                    printf("'%s' sent successfully!\n", UART_INIT_SEQUENCE);
                    return;
                }
            }
        }
    }
    printf("UART error occured!\n");
}

void uart_send_code(char *code)
{
    uart_puts(uart0, code);
    uart_puts(uart0, "\r\n");
}

void key_callback(uint gpio)
{
    float divider = 2.;
    uint8_t i = 0;

    printf("key_callback(%d) ", gpio);
    switch (gpio)
    {
        case PIN_A_KEY:
            uart_send_code(KEY_A_CODE);
            printf("Key A rang\r\n");
            break;
        case PIN_B_KEY:
            uart_send_code(KEY_B_CODE);
            printf("Key B rang\r\n");
            break;
    }

    // Load the configuration into our PWM slice, and set it running.
    for (i; i < MAX_PWM_LEDS; i++)
    {
        pwm_led_t p = pwm_leds[i];
        if (p.key_in == gpio)
        {
            printf("activate LED on pin %d", p.led_out);
            pwm_leds[i].active = true;
            divider *= 2.;
        }
    }

    printf("start pwm divider %.2f", divider);

    // Set divider, reduces counter clock to sysclock/this value
    pwm_config_set_clkdiv(&pwm_conf, divider);

    for (i = 0; i < MAX_PWM_LEDS; i++)
    {
        pwm_led_t p = pwm_leds[i];
        if (p.key_in == gpio)
        {
            uint slice_num = pwm_gpio_to_slice_num(p.led_out);
            printf("adding LED on pin %d to pwm slice %d", p.led_out, slice_num);
            pwm_init(slice_num, &pwm_conf, true);
        }
    }
}

void on_pwm_wrap()
{
    uint8_t i = 0;
    float divider = 2.;
    uint8_t num_active = 0;

    printf("on_pwm_wrap() ");

    for (i; i < MAX_PWM_LEDS; i++)
    {
        printf("{%d}=", i);
        if (pwm_leds[i].active)
        {
            printf("ACTIVE ");
            num_active++;
            divider *= 2.;
        }
        else {
            printf("OFF ");
        }
    }
    for (i = 0; i < MAX_PWM_LEDS; i++)
    {
        pwm_led_t p = pwm_leds[i];
        printf("[%d]=", i);

        if (!p.led_out)
            continue;

        // printf(" checking whether active\r\n");

        // printf("=%i ", p.active);

        uint slice_num = pwm_gpio_to_slice_num(p.led_out);
        printf("slice_num=%i ", slice_num);

        // Clear the interrupt flag that brought us here
        pwm_clear_irq(slice_num);

        if (p.active) {
            // printf("getting slice from gpio ");
            // printf("%d ", p.led_out);



            printf("count=%d fade=%d ", pwm_leds[i].count, pwm_leds[i].fade);

            if (pwm_leds[i].count % 2)
            {
                pwm_leds[i].fade += 1;
                if (pwm_leds[i].fade >= 255)
                {
                    pwm_leds[i].count += 1;
                }
            }
            else
            {
                pwm_leds[i].fade -= 1;
                if (pwm_leds[i].fade <= 0)
                {
                    pwm_leds[i].count += 1;
                }
            }
            if (pwm_leds[i].count == 6)
            {
                printf(" COUNT==6 i=%d num_active=%u", i, num_active);
                pwm_leds[i].count = 0;
                pwm_leds[i].active = false;
                num_active--;

                for (uint8_t j = 0; j < MAX_PWM_LEDS; j++)
                {
                    pwm_led_t p = pwm_leds[j];
                    if (p.active) {
                        printf("setting new divider %.2f for gpio %d\n", divider, p.led_out);
                        uint slice = pwm_gpio_to_slice_num(p.led_out);
                        pwm_config_set_clkdiv(&pwm_conf, divider);
                        pwm_init(slice, &pwm_conf, true);
                    }
                }
                pwm_set_gpio_level(p.led_out, pwm_gamma[p.idle_value]);
                continue;
            }
            pwm_set_gpio_level(p.led_out, pwm_gamma[pwm_leds[i].fade]);
            printf("%hu\r\n", pwm_gamma[pwm_leds[i].fade]);
        }
        if (num_active == 0)
        {
            // when all fades finished, stop pwm
            printf("stop pwn\n");
            pwm_init(slice_num, &pwm_conf, false);
        }
    }
}

void set_key_c_color(lock_action_t action)
{
    bool r, g, b = 1;

    switch (action)
    {
    case LOCK_A_OFF: // black
        r = 1;
        g = 1;
        b = 1;
        break;
    case LOCK_A_LOCKING: // purple
        r = 0;
        g = 1;
        b = 0;
        break;
    case LOCK_A_LOCKED: // red
        r = 0;
        g = 1;
        b = 1;
        break;
    case LOCK_A_LEAVING: // blue
        r = 1;
        g = 1;
        b = 0;
        break;
    case LOCK_A_UNLOCKING: // cyan
        r = 1;
        g = 0;
        b = 0;
        break;
    case LOCK_A_UNLOCKED: // green
        r = 1;
        g = 0;
        b = 1;
        break;
    case LOCK_A_ACTUATED: // yellow
        r = 0;
        g = 0;
        b = 1;
        break;
    case LOCK_A_PRESSED: // white
        r = 0;
        g = 0;
        b = 0;
        break;
    default:
        break;
    }
    gpio_put(PIN_C_RED, r);
    gpio_put(PIN_C_GREEN, g);
    gpio_put(PIN_C_BLUE, b);
}

int64_t buzzer_alarm_cb(alarm_id_t id, void *user_data)
{
    printf("buzzer_alarm_cb -> Buzzer off!\r\n");
    gpio_put(PIN_RELAY_OUT, 0);
    return 0;
}

int64_t actuated_lock_alarm_cb(alarm_id_t id, void *user_data)
{
    lock_state_t new_state = gpio_get(PIN_RELAY_IN2) ? LOCK_S_LOCKED : LOCK_S_UNLOCKED;
    printf("actuated_lock_alarm_cb -> new_state: %s\r\n", lock_state_str[new_state]);
    if (new_state == LOCK_S_LOCKED)
    {
        set_key_c_color(LOCK_A_LOCKED);
    }
    else if (new_state == LOCK_S_UNLOCKED)
    {
        set_key_c_color(LOCK_A_UNLOCKED);
        if (gpio_get(PIN_RELAY_IN1))
        {
            printf("Door got unlocked & Relay 1 still on -> Buzzer on!\r\n");
            gpio_put(PIN_RELAY_OUT, 1);
        }
        if (lock_state == LOCK_S_WAITING_FOR_UNLOCK)
        {
            printf("Door got unlocked while waiting -> Buzzer on for %d ms!\r\n", BUZZER_MS);
            gpio_put(PIN_RELAY_OUT, 1);
            add_alarm_in_ms(BUZZER_MS, buzzer_alarm_cb, NULL, false);
        }
    }
    lock_state = new_state;
    return 0;
}

int64_t delayed_lock_alarm_cb(alarm_id_t id, void *user_data)
{
    printf("...delayed_lock_alarm_cb(%d) send %s\r\n", (int)id, LOCK_CODE);
    set_key_c_color(LOCK_A_OFF);
    uart_send_code(LOCK_CODE);
    return 0;
}

int64_t lock_key_pressed_cb(alarm_id_t id, void *user_data)
{
    printf("...lock_key_pressed_cb(%d) ", (int)id);

    bool long_pressed = gpio_get(PIN_C_KEY);

    if (long_pressed)
    {
        printf("LONG pressed -> LOCK immediately (send %s)\r\n", LOCK_CODE);
        uart_send_code(LOCK_CODE);
        set_key_c_color(LOCK_A_LOCKING);
    }
    else
    {
        printf("short pressed -> LOCK delayed ");
        set_key_c_color(LOCK_A_LEAVING);
        add_alarm_in_ms(DELAY_LEAVING_LOCK_MS, delayed_lock_alarm_cb, NULL, false);
    }

    return 0;
}

void sensor_input(uint gpio, uint32_t events)
{
    static uint32_t last_irq_ts = 0;
    uint32_t now = time_us_32();

    if (now - last_irq_ts > DEBOUNCE_MS * 1000)
    {
        printf("sensor_input(%d, %02X) ...", gpio, events);
        busy_wait_us(1000);
        bool RisingEdge = gpio_get(gpio);
        bool FallingEdge = !RisingEdge;
        printf(" is %s\t", RisingEdge ? "RisingEdge" : "FallingEdge");

        switch (gpio)
        {
        case PIN_A_KEY:
        case PIN_B_KEY:
            if (RisingEdge)
                key_callback(gpio);
            break;
        case PIN_C_KEY:
            printf("lock toggle pressed in state %s ", lock_state_str[lock_state]);
            if (lock_state != LOCK_S_UNLOCKED)
            {
                printf(" -> UNLOCK (send %s)\r\n", UNLOCK_CODE);
                set_key_c_color(LOCK_A_UNLOCKING);
                uart_send_code(UNLOCK_CODE);
            }
            else
            {
                set_key_c_color(LOCK_A_PRESSED);
                add_alarm_in_ms(LONG_PRESS_MS, lock_key_pressed_cb, NULL, false);
            }
            break;
        case PIN_REED_LETTER:
            if (FallingEdge)
            {
                printf("Letterbox opened (send %s)\r\n", LETTER_CODE);
                uart_send_code(LETTER_CODE);
            }
            break;
        case PIN_REED_DOOR:
            door_state = RisingEdge ? DOOR_CLOSED : DOOR_OPEN;
            printf("Reed door toggle -> door_state = %s\r\n", door_state == DOOR_OPEN ? "OPENED" : "CLOSED");
            break;
        case PIN_RELAY_IN1:
            if (FallingEdge)
            {
                printf("Relay 1 falling -> Buzzer off!\r\n");
                gpio_put(PIN_RELAY_OUT, 0);
            }
            else
            {
                if (lock_state == LOCK_S_UNLOCKED)
                {
                    printf("Relay 1 rising -> Buzzer on!\r\n");
                    gpio_put(PIN_RELAY_OUT, 1);
                }
                else
                {
                    printf("Relay 1 rising while in LOCKED ");
                    if (DO_UNLOCK_ON_OPEN)
                    {
                        printf(" -> UNLOCK (send %s)...\r\n", UNLOCK_CODE);
                        set_key_c_color(LOCK_A_UNLOCKING);
                        uart_send_code(UNLOCK_CODE);
                    }
                    else
                    {
                        printf(" -> waiting for unlocked...\r\n");
                        lock_state = LOCK_S_WAITING_FOR_UNLOCK;
                        set_key_c_color(LOCK_A_LEAVING);
                    }
                }
            }
            break;
        case PIN_RELAY_IN2:
            printf("Relay 2 toggle (previous state: %s) -> ", lock_state_str[lock_state]);
            set_key_c_color(LOCK_A_ACTUATED);
            add_alarm_in_ms(DELAY_ACTUATED_LOCK_MS, actuated_lock_alarm_cb, NULL, false);
            break;
        }
    }
    last_irq_ts = now;
}

int setup_irq()
{
    stdio_init_all();
    printf("Hello GPIO IRQ\n");
    gpio_set_irq_enabled_with_callback(PIN_A_KEY, GPIO_IRQ_EDGE_RISE, true, &sensor_input);
    gpio_set_irq_enabled_with_callback(PIN_B_KEY, GPIO_IRQ_EDGE_RISE, true, &sensor_input);
    gpio_set_irq_enabled_with_callback(PIN_C_KEY, GPIO_IRQ_EDGE_RISE, true, &sensor_input);
    gpio_set_irq_enabled_with_callback(PIN_REED_LETTER, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &sensor_input);
    gpio_set_irq_enabled_with_callback(PIN_REED_DOOR, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &sensor_input);
    gpio_set_irq_enabled_with_callback(PIN_RELAY_IN1, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &sensor_input);
    gpio_set_irq_enabled_with_callback(PIN_RELAY_IN2, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &sensor_input);
}

int setup_gpio()
{
    gpio_init_mask(
        1 << PIN_A_BLUE |
        1 << PIN_A_GREEN |
        1 << PIN_A_RED |
        1 << PIN_B_BLUE |
        1 << PIN_B_GREEN |
        1 << PIN_B_RED |
        1 << PIN_C_BLUE |
        1 << PIN_C_GREEN |
        1 << PIN_C_RED |
        1 << PIN_RELAY_OUT |
        1 << PIN_STATUS);
    gpio_set_dir(PIN_RELAY_OUT, GPIO_OUT);
    gpio_set_dir(PIN_A_BLUE, GPIO_OUT);
    gpio_set_dir(PIN_A_GREEN, GPIO_OUT);
    gpio_set_dir(PIN_A_RED, GPIO_OUT);
    gpio_set_dir(PIN_B_BLUE, GPIO_OUT);
    gpio_set_dir(PIN_B_GREEN, GPIO_OUT);
    gpio_set_dir(PIN_B_RED, GPIO_OUT);
    gpio_set_dir(PIN_C_BLUE, GPIO_OUT);
    gpio_set_dir(PIN_C_GREEN, GPIO_OUT);
    gpio_set_dir(PIN_C_RED, GPIO_OUT);
    gpio_set_dir(PIN_STATUS, GPIO_OUT);

    gpio_put(PIN_A_BLUE, 1);
    gpio_put(PIN_A_GREEN, 1);
    gpio_put(PIN_A_RED, 1);
    gpio_put(PIN_B_BLUE, 1);
    gpio_put(PIN_B_GREEN, 1);
    gpio_put(PIN_B_RED, 1);

    set_key_c_color(LOCK_A_OFF);

    // GPIO26 & 27 are analog by default, need to be set to input explicitely in order for IRQ to work
    gpio_set_input_enabled(PIN_RELAY_IN1, true);
    gpio_set_input_enabled(PIN_RELAY_IN2, true);

    gpio_pull_down(PIN_RELAY_IN1);
    gpio_pull_down(PIN_RELAY_IN2);
    gpio_pull_down(PIN_REED_DOOR);
    gpio_pull_down(PIN_REED_LETTER);

    gpio_put(PIN_RELAY_OUT, 0);
    door_state = DOOR_UNKNOWN;
    lock_state = LOCK_S_UNKNOWN;
}

int setup_pwm()
{
    for (uint8_t i = 0; i < MAX_PWM_LEDS; i++)
    {
      pwm_led_t p = pwm_leds[i];
      pwm_leds[i].active = false;
      pwm_leds[i].key_in = pwm_leds[i].led_out = 0;
      pwm_leds[i].fade = pwm_leds[i].count = 0;
    }

    pwm_leds[0].key_in = PIN_A_KEY;
    pwm_leds[0].led_out = PIN_A_GREEN;
    pwm_leds[0].idle_value = 200;

    pwm_leds[1].key_in = PIN_B_KEY;
    pwm_leds[1].led_out = PIN_B_BLUE;
    pwm_leds[1].idle_value = 200;

    pwm_leds[2].key_in = PIN_B_KEY;
    pwm_leds[2].led_out = PIN_B_RED;
    pwm_leds[2].idle_value = 200;

    uint slice_num;

    for (uint8_t i = 0; i < MAX_PWM_LEDS; i++)
    {
        pwm_led_t p = pwm_leds[i];

        if (p.led_out == 0)
            continue;

        // Tell the LED pin that the PWM is in charge of its value.
        gpio_set_function(p.led_out, GPIO_FUNC_PWM);
        pwm_set_gpio_level(p.led_out, pwm_gamma[p.idle_value]);

        // Figure out which slice we just connected to the LED pin
        slice_num = pwm_gpio_to_slice_num(p.led_out);
        printf("led[%d] setting idle val %d = %u, slice_num=%u\r\n", p.led_out, p.idle_value, pwm_gamma[p.idle_value], slice_num);
        pwm_clear_irq(slice_num);
        pwm_set_irq_enabled(slice_num, true);
    }
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    pwm_conf = pwm_get_default_config();
}

int setup_uart()
{
    // Initialise UART 0
    uart_init(UART_ID, BAUD_RATE);

    // Set the GPIO pin mux to the UART - 0 is TX, 1 is RX
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);
}

int main()
{
    setup_gpio();
    setup_irq();
    setup_uart();

    sleep_ms(2000);
    setup_pwm();

    gpio_put(PIN_STATUS, 1);

    // Everything after this point happens in the interrupt handlers,
    // so we can chill here
    while (1)
    {
        gpio_put(PIN_STATUS, 1);
        sleep_ms(250);
        gpio_put(PIN_STATUS, 0);
        sleep_ms(250);
        tight_loop_contents();
    }
}

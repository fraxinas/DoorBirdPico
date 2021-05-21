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

#define PIN_A_KEY 5
#define PIN_A_ANODE 6
#define PIN_A_BLUE 7
#define PIN_A_GREEN 8
#define PIN_A_RED 9

#define PIN_B_KEY 10
#define PIN_B_ANODE 11
#define PIN_B_BLUE 12
#define PIN_B_GREEN 13
#define PIN_B_RED 14

pwm_config pwm_conf;
int fade_pins[2] = {-1, -1};

static const char *gpio_irq_str[] = {
    "LEVEL_LOW",  // 0x1
    "LEVEL_HIGH", // 0x2
    "EDGE_FALL",  // 0x4
    "EDGE_RISE"   // 0x8
};

static char event_str[128];

void on_uart_rx() {
    printf("on_uart_rx... ");
    while (uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);
        printf("0x%02X ", ch);
        if (ch == 0x02) {
            ch = uart_getc(UART_ID);
            printf("%02X\n", ch);
            if (ch == 0x01) {
                if (uart_is_writable(UART_ID)) {
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

void gpio_event_string(char *buf, uint32_t events)
{
    for (uint i = 0; i < 4; i++)
    {
        uint mask = (1 << i);
        if (events & mask)
        {
            // Copy this event string into the user string
            const char *event_str = gpio_irq_str[i];
            while (*event_str != '\0')
            {
                *buf++ = *event_str++;
            }
            events &= ~mask;

            // If more events add ", "
            if (events)
            {
                *buf++ = ',';
                *buf++ = ' ';
            }
        }
    }
    *buf++ = '\0';
}

void key_callback(uint gpio, uint32_t events)
{
    gpio_event_string(event_str, events);
    printf("GPIO KEY %d %s ", gpio, event_str);
    if (events & 0x8)
    {
        // Load the configuration into our PWM slice, and set it running.
        int i, irq_pin;

        switch (gpio)
        {
          case PIN_A_KEY:
              irq_pin = PIN_A_GREEN;
              uart_puts(uart0, "101#\r\n");
              i = 0;
              break;
          case PIN_B_KEY:
              irq_pin = PIN_B_BLUE;
              uart_puts(uart0, "102#\r\n");
              printf("sent 102#\\r\\n");
              i = 1;
              break;
        }
        fade_pins[i] = irq_pin;
        printf("i=%d pin=%d",i, fade_pins[i]);

        float divider = 4.;

        if (fade_pins[0] > 0 && fade_pins[1] > 0)
        {
          // half time if both pins are fading
           divider = 8.;
        }

        printf(" start pwm divider %f", divider);
        // Set divider, reduces counter clock to sysclock/this value
        pwm_config_set_clkdiv(&pwm_conf, divider);

        for (int i=0; i<2; i++)
        {
          if (fade_pins[i] > 0)
          {
             pwm_init(pwm_gpio_to_slice_num(fade_pins[i]), &pwm_conf, true);
          }
        }
    }
}

int setup_irq()
{
    stdio_init_all();
    printf("Hello GPIO IRQ\n");
    gpio_set_irq_enabled_with_callback(PIN_A_KEY, GPIO_IRQ_EDGE_RISE, true, &key_callback);
    gpio_set_irq_enabled_with_callback(PIN_B_KEY, GPIO_IRQ_EDGE_RISE, true, &key_callback);
}

void on_pwm_wrap()
{
    static int fade = 0;
    static bool going_up = true;
    static int count = 0;

    for (int i=0; i<2; i++)
    {
      printf("w[%d]",i);
      int irq_pin = fade_pins[i];
      printf("=",irq_pin);
      if (irq_pin < 0)
        continue;
      uint slice = pwm_gpio_to_slice_num(irq_pin);
      // Clear the interrupt flag that brought us here
      pwm_clear_irq(slice);

      if (going_up)
      {
          ++fade;
          if (fade > 255)
          {
              fade = 255;
              going_up = false;
              printf("switch to down\n");
          }
      }
      else
      {
          --fade;
          if (fade < 0)
          {
              fade = 0;
              going_up = true;
              printf("switch to up, count=%d\n", count);
              if (++count == 5)
              {
                  count = 0;
                  pwm_set_gpio_level(irq_pin, 0x2000);
                  fade_pins[i] = -1;
                  if (fade_pins[0] == -1 && fade_pins[0] == -1)
                  {
                     // when all fades finished, stop pwm
                     printf("stop pwn\n");
                     pwm_init(slice, &pwm_conf, false);
                  }
                  else if (fade_pins[0] == -1 || fade_pins[1] == -1)
                  {
                    slice = (i==0) ? pwm_gpio_to_slice_num(fade_pins[1]) : pwm_gpio_to_slice_num(fade_pins[0]);
                    // regular speed if only one pin is fading
                    pwm_config_set_clkdiv(&pwm_conf, 4.0f);
                    pwm_init(slice, &pwm_conf, true);
                  }
              }
          }
      }
      // Square the fade value to make the LED's brightness appear more linear
      // Note this range matches with the wrap value
      pwm_set_gpio_level(irq_pin, fade * fade);
    }
}

int setup_pwm()
{
    gpio_init(PIN_A_ANODE);
    gpio_set_dir(PIN_A_ANODE, GPIO_OUT);
    gpio_put(PIN_A_ANODE, 1);

    gpio_init(PIN_B_ANODE);
    gpio_set_dir(PIN_B_ANODE, GPIO_OUT);
    gpio_put(PIN_B_ANODE, 1);

    // Tell the LED pin that the PWM is in charge of its value.
    gpio_set_function(PIN_A_GREEN, GPIO_FUNC_PWM);
    pwm_set_gpio_level(PIN_A_GREEN, 0xE000);

    gpio_set_function(PIN_B_BLUE, GPIO_FUNC_PWM);
    pwm_set_gpio_level(PIN_B_BLUE, 0xE000);

    uint slice_num;

    // Figure out which slice we just connected to the LED pin
    slice_num = pwm_gpio_to_slice_num(PIN_A_GREEN);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);

    slice_num = pwm_gpio_to_slice_num(PIN_B_BLUE);
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);

    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // Get some sensible defaults for the slice configuration. By default, the
    // counter is allowed to wrap over its maximum range (0 to 2**16-1)
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
    setup_irq();
    setup_pwm();
    setup_uart();

    // Everything after this point happens in the PWM interrupt handler, so we
    // can twiddle our thumbs
    while (1)
        tight_loop_contents();
}
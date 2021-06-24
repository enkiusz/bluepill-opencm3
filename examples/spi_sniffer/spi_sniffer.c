#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <string.h>

void init_clocks(void);
void init_io(void);
void init_usart(void);
void init_exti(void);
int main(void);

void init_clocks(void) {
  // internal oscillator (HSI) at 48MHz
  rcc_clock_setup_in_hsi_out_48mhz();
  rcc_periph_clock_enable(RCC_GPIOA);  
	rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_USART2);
}

void init_io(void) {

  // /CS  -> PB10 and PB13 (to properly detect both edges)
  // SCK  -> PB11
  // MOSI -> PB12
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO10 | GPIO11 | GPIO12 | GPIO13);

}

void init_usart(void) {

  //Setup USART2 TX on PA2
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);

  usart_set_baudrate(USART2, 115200);
  usart_set_databits(USART2, 8);
  usart_set_stopbits(USART2, USART_STOPBITS_1);
  usart_set_mode(USART2, USART_MODE_TX_RX);
  usart_set_parity(USART2, USART_PARITY_NONE);
  usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	usart_enable(USART2);
}

void init_exti(void)
{
	rcc_periph_clock_enable(RCC_AFIO);
	nvic_enable_irq(NVIC_EXTI15_10_IRQ);

	exti_select_source(EXTI10, GPIOB);
	exti_select_source(EXTI11, GPIOB);
	exti_select_source(EXTI13, GPIOB);

  exti_set_trigger(EXTI10, EXTI_TRIGGER_FALLING);
  exti_set_trigger(EXTI11, EXTI_TRIGGER_RISING);
  exti_set_trigger(EXTI13, EXTI_TRIGGER_RISING);

}

volatile char buf[512];
volatile unsigned int bit_count;
volatile bool flag = false;

#define BIT_WRITE(word, n, bit) word = (word & ~(1U << (n) )) | (bit << (n) )

void exti15_10_isr(void) {

  if (exti_get_flag_status(EXTI11)) {
    exti_reset_request(EXTI11);

    // sample and store MOSI bit in buffer
    bool bit = gpio_get(GPIOB, GPIO12);
    BIT_WRITE(buf[bit_count / 8], 7-(bit_count % 8), bit);
    bit_count += 1;
  }

  if (exti_get_flag_status(EXTI10)) {
    exti_reset_request(EXTI10);

    exti_disable_request(EXTI10);
    exti_enable_request(EXTI13);

    exti_enable_request(EXTI11);
  }

  if (exti_get_flag_status(EXTI13)) {
    exti_reset_request(EXTI13);

  	exti_disable_request(EXTI10);
  	exti_disable_request(EXTI11);
  	exti_disable_request(EXTI13);

    flag = true;

  }
}

#define BYTECOUNT(bits) ( ((bits) & 0x07) ? ((bits) / 8) + 1 : ((bits) / 8) )

int main() {
  init_clocks();
  init_io();
  init_usart();
  init_exti();
  
	exti_enable_request(EXTI10);
  exti_disable_request(EXTI13);

  while(1) {
    if (! flag) {
      continue;
    }

    /* Send packet with SPI transaction data */
    bit_count = bit_count & 0xFF;
    usart_send_blocking(USART2, 0xF0);
    usart_send_blocking(USART2, bit_count);
    for(unsigned int i=0; i < BYTECOUNT(bit_count); i++) {
      usart_send_blocking(USART2, buf[i]);
    }
    usart_send_blocking(USART2, 0x0F);

    memset(buf, 0, sizeof(buf));
    bit_count = 0;
    flag = false;

  	exti_enable_request(EXTI10);
  	exti_disable_request(EXTI13);
  }

  return 0;
}

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <string.h>

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

/*
 * Hardware revision differences:
 * 
 * * REV 0 - Prototype using the STM32 Bluepill
 * LCD connections:
 * /CS             -> PB10 and PB13 (to properly detect both edges)
 * SCK             -> PB11
 * DATA (aka MOSI) -> PB12
 * 
 * * REV 1 - First dedicated board
 * Did not make into production, was replaced by REV 2.
 * 
 * * REV 2 - Second dedicated board
 * LCD connections:
 * /CS             -> PB14 and PB15 (to properly detect both edges)
 * SCK             -> PA9
 * DATA (aka MOSI) -> PA10
 * 
 * * REV 3 - Third dedicated board
 * /CS             -> PA9 and PA10 (to properly detect both edges)
 * SCK             -> PA8
 * DATA (aka MOSI) -> PB15
 * 
 * B1 voltage              -> PA1 (ADC12_IN1)
 * B2 voltage              -> PA2 (ADC12_IN2)
 * B3 voltage              -> PA3 (ADC12_IN3)
 * B4 voltage              -> PA4 (ADC12_IN4)
 * 
 * 1WIRE/I2C (temperature) -> PB6 and PB7
 * USART3 (debug USART)    -> PB10 and PB11
 * 
 */

// Default HW REV for build
#define HW_REV 2

#define FW_VER 0.8

#define PROTOCOL_CODE 0

#define DUMP_PACKETS_TO_USART

// the fuck is this shit
// Reference: http://www.decompile.com/cpp/faq/file_and_line_error_string.htm
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define FW_BANNER(hw_rev, fw_ver, serialno) "# SDKK-LII-500 HW rev " TOSTRING(hw_rev) " FW ver " TOSTRING(fw_ver) " usb serial " serialno " proto " TOSTRING(PROTOCOL_CODE) "\r\n"

#include "serial_number.h"

void init_clocks(void);
void init_io(void);
void init_exti(void);
void init_debug_usart(void);
int main(void);
void debug_puts(const char *p);

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = USB_CLASS_CDC,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0x5740,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

/*
 * This notification endpoint isn't implemented. According to CDC spec its
 * optional, but its absence causes a NULL pointer dereference in Linux
 * cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}};

static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt = {
		.bFunctionLength = 
			sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = 1,
	},
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 0,
	},
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = 0,
		.bSubordinateInterface0 = 1, 
	 }
};

static const struct usb_interface_descriptor comm_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
	.iInterface = 0,

	.endpoint = comm_endp,

	.extra = &cdcacm_functional_descriptors,
	.extralen = sizeof(cdcacm_functional_descriptors)
}};

static const struct usb_interface_descriptor data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = comm_iface,
}, {
	.num_altsetting = 1,
	.altsetting = data_iface,
}};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"Black Sphere Technologies",
	"CDC-ACM Demo",
	USB_SERIALNO,
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req)) {

	(void)complete;
	(void)buf;
	(void)usbd_dev;

	switch(req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
		/*
		 * This Linux cdc_acm driver requires this to be implemented
		 * even though it's optional in the CDC spec, and we don't
		 * advertise it in the ACM functional descriptor.
		 */
		char local_buf[10];
		struct usb_cdc_notification *notif = (void *)local_buf;

		/* We echo signals back to host as notification. */
		notif->bmRequestType = 0xA1;
		notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
		notif->wValue = 0;
		notif->wIndex = 0;
		notif->wLength = 2;
		local_buf[8] = req->wValue & 3;
		local_buf[9] = 0;
		// usbd_ep_write_packet(0x83, buf, 10);
		return USBD_REQ_HANDLED;
		}
	case USB_CDC_REQ_SET_LINE_CODING: 
		if(*len < sizeof(struct usb_cdc_line_coding))
			return USBD_REQ_NOTSUPP;

		return USBD_REQ_HANDLED;
	}
	return USBD_REQ_NOTSUPP;
}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep) {

	(void)ep;
    char buf[64];
	usbd_ep_read_packet(usbd_dev, 0x01, buf, sizeof(buf));
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue) {

	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);
}

void init_clocks(void) {
  rcc_clock_setup_in_hsi_out_48mhz(); // internal oscillator (HSI) at 48MHz
}

void init_io(void) {

#if HW_REV == 0

  rcc_periph_clock_enable(RCC_GPIOB);
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO10 | GPIO11 | GPIO12 | GPIO13);

#elif HW_REV == 2

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO9 | GPIO10);
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO14 | GPIO15);

#elif HW_REV == 3

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO8 | GPIO9 | GPIO10);
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO15);

#else
#error There is no IO initialization code for the selected HW_REV value
#endif

}

void init_exti(void) {

#if HW_REV == 0

  rcc_periph_clock_enable(RCC_AFIO);
  nvic_enable_irq(NVIC_EXTI15_10_IRQ);

  exti_select_source(EXTI10, GPIOB); // /CS on PB10 (start transfer)
  exti_set_trigger(EXTI10, EXTI_TRIGGER_FALLING);

  exti_select_source(EXTI13, GPIOB); // /CS on PB13 (end transfer)
  exti_set_trigger(EXTI13, EXTI_TRIGGER_RISING);

  exti_select_source(EXTI11, GPIOB); // SCK on PB11
  exti_set_trigger(EXTI11, EXTI_TRIGGER_RISING);

#elif HW_REV == 2

  rcc_periph_clock_enable(RCC_AFIO);
  nvic_enable_irq(NVIC_EXTI9_5_IRQ);
  nvic_enable_irq(NVIC_EXTI15_10_IRQ);

  exti_select_source(EXTI14, GPIOB); // /CS on PB14 (start transfer)
  exti_set_trigger(EXTI14, EXTI_TRIGGER_FALLING);

  exti_select_source(EXTI15, GPIOB); // /CS on PB15 (end transfer)
  exti_set_trigger(EXTI15, EXTI_TRIGGER_RISING);

  exti_select_source(EXTI9, GPIOA); // SCK on PA9
  exti_set_trigger(EXTI9, EXTI_TRIGGER_RISING);

#else
#error There is no EXTI initialization code for the selected HW_REV value
#endif

}

void init_debug_usart(void) {

#if HW_REV == 2 || HW_REV == 3
  rcc_periph_clock_enable(RCC_AFIO);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_USART3);

  //Configure USART3 TX on PB10
  //Configure USART3 RX on PB11
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART3_RX);

  usart_set_baudrate(USART3, 115200);
  usart_set_databits(USART3, 8);
  usart_set_stopbits(USART3, USART_STOPBITS_1);
  usart_set_mode(USART3, USART_MODE_TX_RX);
  usart_set_parity(USART3, USART_PARITY_NONE);
  usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

  usart_enable(USART3);
#else
#warning No debug USART setup code for hardware revision
#endif

}

void debug_puts(const char *p) {
#if HW_REV == 2 || HW_REV == 3
	while (*p) { usart_send_blocking(USART3, *p); p++; }
#endif
}


volatile char buf[512];
volatile unsigned int bit_count;
volatile bool flag = false;

#define BIT_WRITE(word, n, bit) word = (word & ~(1U << (n) )) | (bit << (n) )

#if HW_REV == 0

void exti15_10_isr(void) {

  // SCK on PB11
  if (exti_get_flag_status(EXTI11)) {
    exti_reset_request(EXTI11);

    // sample and store MOSI bit in buffer
    bool bit = gpio_get(GPIOB, GPIO12);
	// 2 bytes at the beginning of the buffer are reserved for the header
    BIT_WRITE(buf[2 + bit_count / 8], 7-(bit_count % 8), bit);
    bit_count += 1;
  }

  // /CS on PB10 (start transfer)
  if (exti_get_flag_status(EXTI10)) {
    exti_reset_request(EXTI10);

    exti_disable_request(EXTI10); // /CS on PB10 (start transfer)
    exti_enable_request(EXTI13);  // /CS on PB13 (end transfer)
    exti_enable_request(EXTI11);  // SCK on PB11
  }

  // /CS on PB13 (end transfer)
  if (exti_get_flag_status(EXTI13)) {
    exti_reset_request(EXTI13);

	exti_disable_request(EXTI10); // /CS on PB10 (start transfer)
	exti_disable_request(EXTI13); // /CS on PB13 (end transfer)
	exti_disable_request(EXTI11); // SCK on PB11

    flag = true;

  }
}

#elif HW_REV == 2

void exti9_5_isr(void) {

  // SCK on PA9
  if (exti_get_flag_status(EXTI9)) {
    exti_reset_request(EXTI9);

    // sample and store MOSI bit in buffer
    bool bit = gpio_get(GPIOA, GPIO10);
	// 2 bytes at the beginning of the buffer are reserved for the header
    BIT_WRITE(buf[2 + bit_count / 8], 7-(bit_count % 8), bit);
    bit_count += 1;
  }

}


void exti15_10_isr(void) {

  // /CS on PB14 (start transfer)
  if (exti_get_flag_status(EXTI14)) {
    exti_reset_request(EXTI14);

    exti_disable_request(EXTI14); // /CS on PB14 (start transfer)
    exti_enable_request(EXTI15);  // /CS on PB15 (end transfer)
    exti_enable_request(EXTI9);   // SCK on PA9
  }

  // /CS on PB15 (end transfer)
  if (exti_get_flag_status(EXTI15)) {
    exti_reset_request(EXTI15);

	exti_disable_request(EXTI14); // /CS on PB14 (start transfer)
	exti_disable_request(EXTI15); // /CS on PB15 (end transfer)
	exti_disable_request(EXTI9);  // SCK on PA9

    flag = true;
  }

}

#else
  #error No interrupt handler code for for the selected HW_REV value
#endif

#define BYTECOUNT(bits) ( ((bits) & 0x07) ? ((bits) / 8) + 1 : ((bits) / 8) )

int main() {
  usbd_device *usbd_dev;

  init_clocks();
  init_io();
  init_exti();
  init_debug_usart();
  debug_puts( FW_BANNER(HW_REV, FW_VER, USB_SERIALNO) );

  usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
  usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);

#if HW_REV == 0

  exti_enable_request(EXTI10);  // /CS on PB10 (start transfer)
  exti_disable_request(EXTI13); // /CS on PB13 (end transfer)

#elif HW_REV == 2

  exti_enable_request(EXTI14);  // /CS on PB14 (start transfer)
  exti_disable_request(EXTI15); // /CS on PB15 (end transfer)

#else
#error No interrupt handler code for for the selected HW_REV value
#endif

  while(1) {
		usbd_poll(usbd_dev);

    if (! flag) {
      continue;
    }

    /* Send packet with SPI transaction data */
    bit_count = bit_count & 0xFF;
    buf[0] = 0xF0;
    buf[1] = bit_count;
    buf[2 + BYTECOUNT(bit_count)] = 0x0F;
    usbd_ep_write_packet(usbd_dev, 0x82, buf, 3 + BYTECOUNT(bit_count));

	#ifdef DUMP_PACKETS_TO_USART
		for(unsigned int i = 0; i < 3 + BYTECOUNT(bit_count); i++) usart_send_blocking(USART3, buf[i]);
	#endif

    memset(buf, 0, sizeof(buf));
    
    bit_count = 0;
    flag = false;

#if HW_REV == 0

	exti_enable_request(EXTI10);  // /CS on PB10 (start transfer)
	exti_disable_request(EXTI13); // /CS on PB13 (end transfer)

#elif HW_REV == 2

	exti_enable_request(EXTI14);  // /CS on PB14 (start transfer)
	exti_disable_request(EXTI15); // /CS on PB15 (end transfer)

#else
  #error No interrupt handler code for for the selected HW_REV value
#endif

  }

  return 0;
}

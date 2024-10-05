
#ifndef __APPS_CUSTOM_APPS_CUBUS_APP_GPIO_DEFINITIONS_H
#define __APPS_CUSTOM_APPS_CUBUS_APP_GPIO_DEFINITIONS_H

#define ETX_LED_DRIVER_PATH "/dev/gpio_rw"    // LED Driver path

/* Each port bit of the general-purpose I/O (GPIO) ports can be
 * individually configured by software in several modes:
 *
 *  - Input floating
 *  - Input pull-up
 *  - Input-pull-down
 *  - Output open-drain with pull-up or pull-down capability
 *  - Output push-pull with pull-up or pull-down capability
 *  - Alternate function push-pull with pull-up or pull-down capability
 *  - Alternate function open-drain with pull-up or pull-down capability
 *  - Analog
 *
 * 20-bit Encoding:       1111 1111 1100 0000 0000
 *                        9876 5432 1098 7654 3210
 *                        ---- ---- ---- ---- ----
 * Inputs:                MMUU .... ...X PPPP BBBB
 * Outputs:               MMUU .... FFOV PPPP BBBB
 * Alternate Functions:   MMUU AAAA FFO. PPPP BBBB
 * Analog:                MM.. .... .... PPPP BBBB
 */

/* Mode:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * MM.. .... .... .... ....
 */

#define GPIO_MODE_SHIFT               (18)                       /* Bits 18-19: GPIO port mode */
#define GPIO_MODE_MASK                (3 << GPIO_MODE_SHIFT)
#  define GPIO_INPUT                  (0 << GPIO_MODE_SHIFT)     /* Input mode */
#  define GPIO_OUTPUT                 (1 << GPIO_MODE_SHIFT)     /* General purpose output mode */
#  define GPIO_ALT                    (2 << GPIO_MODE_SHIFT)     /* Alternate function mode */
#  define GPIO_ANALOG                 (3 << GPIO_MODE_SHIFT)     /* Analog mode */

/* Input/output pull-ups/downs (not used with analog):
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * ..UU .... .... .... ....
 */

#define GPIO_PUPD_SHIFT               (16)                       /* Bits 16-17: Pull-up/pull down */
#define GPIO_PUPD_MASK                (3 << GPIO_PUPD_SHIFT)
#  define GPIO_FLOAT                  (0 << GPIO_PUPD_SHIFT)     /* No pull-up, pull-down */
#  define GPIO_PULLUP                 (1 << GPIO_PUPD_SHIFT)     /* Pull-up */
#  define GPIO_PULLDOWN               (2 << GPIO_PUPD_SHIFT)     /* Pull-down */

/* Alternate Functions:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... AAAA .... .... ....
 */

#define GPIO_AF_SHIFT                 (12)                       /* Bits 12-15: Alternate function */
#define GPIO_AF_MASK                  (15 << GPIO_AF_SHIFT)
#  define GPIO_AF(n)                  ((n) << GPIO_AF_SHIFT)
#  define GPIO_AF0                    (0 << GPIO_AF_SHIFT)
#  define GPIO_AF1                    (1 << GPIO_AF_SHIFT)
#  define GPIO_AF2                    (2 << GPIO_AF_SHIFT)
#  define GPIO_AF3                    (3 << GPIO_AF_SHIFT)
#  define GPIO_AF4                    (4 << GPIO_AF_SHIFT)
#  define GPIO_AF5                    (5 << GPIO_AF_SHIFT)
#  define GPIO_AF6                    (6 << GPIO_AF_SHIFT)
#  define GPIO_AF7                    (7 << GPIO_AF_SHIFT)
#  define GPIO_AF8                    (8 << GPIO_AF_SHIFT)
#  define GPIO_AF9                    (9 << GPIO_AF_SHIFT)
#  define GPIO_AF10                   (10 << GPIO_AF_SHIFT)
#  define GPIO_AF11                   (11 << GPIO_AF_SHIFT)
#  define GPIO_AF12                   (12 << GPIO_AF_SHIFT)
#  define GPIO_AF13                   (13 << GPIO_AF_SHIFT)
#  define GPIO_AF14                   (14 << GPIO_AF_SHIFT)
#  define GPIO_AF15                   (15 << GPIO_AF_SHIFT)

/* Output/Alt function frequency selection:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... FF.. .... ....
 */

#define GPIO_SPEED_SHIFT              (10)                       /* Bits 10-11: GPIO frequency selection */
#define GPIO_SPEED_MASK               (3 << GPIO_SPEED_SHIFT)

#  define GPIO_SPEED_2MHz             (0 << GPIO_SPEED_SHIFT)    /* 2 MHz Low speed output */
#  define GPIO_SPEED_25MHz            (1 << GPIO_SPEED_SHIFT)    /* 25 MHz Medium speed output */
#  define GPIO_SPEED_50MHz            (2 << GPIO_SPEED_SHIFT)    /* 50 MHz Fast speed output  */


/* Output/Alt function type selection:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... ..O. .... ....
 */

#define GPIO_OPENDRAIN                (1 << 9)                   /* Bit9: 1=Open-drain output */
#define GPIO_PUSHPULL                 (0)                        /* Bit9: 0=Push-pull output */

/* If the pin is a GPIO digital output, then this identifies the initial
 * output value.  If the pin is an input, this bit is overloaded to
 * provide the qualifier to distinguish input pull-up and -down:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... ...V .... ....
 */

#define GPIO_OUTPUT_SET               (1 << 8)                   /* Bit 8: If output, initial value of output */
#define GPIO_OUTPUT_CLEAR             (0)

/* External interrupt selection (GPIO inputs only):
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... ...X .... ....
 */

#define GPIO_EXTI                     (1 << 8)                    /* Bit 8: Configure as EXTI interrupt */

/* This identifies the GPIO port:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... .... PPPP ....
 */

#define GPIO_PORT_SHIFT               (4)                        /* Bit 4-7:  Port number */
#define GPIO_PORT_MASK                (15 << GPIO_PORT_SHIFT)
#  define GPIO_PORTA                  (0 << GPIO_PORT_SHIFT)     /*   GPIOA */
#  define GPIO_PORTB                  (1 << GPIO_PORT_SHIFT)     /*   GPIOB */
#  define GPIO_PORTC                  (2 << GPIO_PORT_SHIFT)     /*   GPIOC */
#  define GPIO_PORTD                  (3 << GPIO_PORT_SHIFT)     /*   GPIOD */
#  define GPIO_PORTE                  (4 << GPIO_PORT_SHIFT)     /*   GPIOE */

#  define GPIO_PORTF                  (5 << GPIO_PORT_SHIFT)     /*   GPIOF */
#  define GPIO_PORTG                  (6 << GPIO_PORT_SHIFT)     /*   GPIOG */
#  define GPIO_PORTH                  (7 << GPIO_PORT_SHIFT)     /*   GPIOH */
#  define GPIO_PORTI                  (8 << GPIO_PORT_SHIFT)     /*   GPIOI */
#  define GPIO_PORTJ                  (9 << GPIO_PORT_SHIFT)     /*   GPIOJ */
#  define GPIO_PORTK                  (10 << GPIO_PORT_SHIFT)    /*   GPIOK */


/* This identifies the bit in the port:
 *
 * 1111 1111 1100 0000 0000
 * 9876 5432 1098 7654 3210
 * ---- ---- ---- ---- ----
 * .... .... .... .... BBBB
 */

#define GPIO_PIN_SHIFT                (0)                        /* Bits 0-3: GPIO number: 0-15 */
#define GPIO_PIN_MASK                 (15 << GPIO_PIN_SHIFT)
#  define GPIO_PIN0                   (0 << GPIO_PIN_SHIFT)
#  define GPIO_PIN1                   (1 << GPIO_PIN_SHIFT)
#  define GPIO_PIN2                   (2 << GPIO_PIN_SHIFT)
#  define GPIO_PIN3                   (3 << GPIO_PIN_SHIFT)
#  define GPIO_PIN4                   (4 << GPIO_PIN_SHIFT)
#  define GPIO_PIN5                   (5 << GPIO_PIN_SHIFT)
#  define GPIO_PIN6                   (6 << GPIO_PIN_SHIFT)
#  define GPIO_PIN7                   (7 << GPIO_PIN_SHIFT)
#  define GPIO_PIN8                   (8 << GPIO_PIN_SHIFT)
#  define GPIO_PIN9                   (9 << GPIO_PIN_SHIFT)
#  define GPIO_PIN10                  (10 << GPIO_PIN_SHIFT)
#  define GPIO_PIN11                  (11 << GPIO_PIN_SHIFT)
#  define GPIO_PIN12                  (12 << GPIO_PIN_SHIFT)
#  define GPIO_PIN13                  (13 << GPIO_PIN_SHIFT)
#  define GPIO_PIN14                  (14 << GPIO_PIN_SHIFT)
#  define GPIO_PIN15                  (15 << GPIO_PIN_SHIFT)

/* KILL SWITCH control and monitoring GPIOs*/
#define GPIO_KILL_SW_EN		    	(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15)


#define GPIO_KILL_SW1_NEG	    	(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN12)
#define GPIO_KILL_SW1_POS	    	(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN2)


#define GPIO_KILL_SW2_NEG	    	(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN4)
#define GPIO_KILL_SW2_POS	    	(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN5)

// #define GPIO_COM_4V_EN			    (GPIO_OUTPUT|GPIO_SPEED_2MHz|GPIO_PORTH|GPIO_PIN12)
// #define GPIO_3V3_COM_EN			    (GPIO_OUTPUT|GPIO_SPEED_2MHz|GPIO_PORTH|GPIO_PIN12)//todo h11/H12

/* Power supply control GPIOs. */
#define GPIO_DCDC_MSN_3V3_2_EN		(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN14)
#define GPIO_DCDC_4V_EN			    (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTI|GPIO_PIN4)
#define GPIO_DCDC_5V_EN			    (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN3)

#define GPIO_3V3_COM_EN			    (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN11)
#define GPIO_MSN_3V3_EN			    (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN5)
#define GPIO_COM_4V_EN			    (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN1)//h12//a1//TODO
#define GPIO_MSN_5V_EN			    (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN12)
#define GPIO_BURNER_EN			    (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN4)
#define GPIO_UNREG_EN			    (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN1)

/* MSN Enable GPIOs */
#define GPIO_MSN1_EN			    (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN3)		//GPIO_1 On Board
#define GPIO_MSN2_EN			    (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN8)		//GPIO_2 On Board
#define GPIO_MSN3_EN			    (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN10)	//GPIO_3 On Board

/* Flash Memory MUX Control GPIOs */
#define GPIO_SFM_MODE		        (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN7)
#define GPIO_SFM_CS		        (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTI|GPIO_PIN6)

#define GPIO_MUX_EN		           	(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN6)
#define GPIO_MUX_EN_EM		           	(GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTI|GPIO_PIN0)

typedef struct{
    uint8_t gpio_val;
    uint32_t gpio_num;
    // void *data;
}gpio_config_s;


#endif 

#ifndef CONFIG_F405AIO_H
#define CONFIG_F405AIO_H

#define BOARD_F405AIO

/**
 * ChibiOS board file
 */
#include "board.h"

/**
 * PPRZ definitions
 */

/*
 * AHB_CLK
 */
#define AHB_CLK STM32_HCLK

/*
 * Concat macro
 */
#define _CONCAT_BOARD_PARAM(_s1, _s2) _s1 ## _s2
#define CONCAT_BOARD_PARAM(_s1, _s2) _CONCAT_BOARD_PARAM(_s1, _s2)

/*
 * LEDs
 */
/* color, on PB02, 1 on LED_ON, 0 on LED_OFF */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO PAL_PORT(LINE_LED1)
#define LED_1_GPIO_PIN PAL_PAD(LINE_LED1)
#define LED_1_GPIO_ON gpio_set
#define LED_1_GPIO_OFF gpio_clear

// #ifndef USE_LED_2
// #define USE_LED_2 1
// #endif
// #define LED_2_GPIO PAL_PORT(LINE_LED2)
// #define LED_2_GPIO_PIN PAL_PAD(LINE_LED2)
// #define LED_2_GPIO_ON gpio_set
// #define LED_2_GPIO_OFF gpio_clear

// #ifndef USE_LED_3
// #define USE_LED_3 1
// #endif
// #define LED_3_GPIO PAL_PORT(LINE_LED3)
// #define LED_3_GPIO_PIN PAL_PAD(LINE_LED3)
// #define LED_3_GPIO_ON gpio_set
// #define LED_3_GPIO_OFF gpio_clear


/*
 * ADCs
 */

// Internal ADC for battery enabled by default
#ifndef USE_ADC_1
#define USE_ADC_1 1
#endif
#if USE_ADC_1
#define AD1_1_CHANNEL CONCAT_BOARD_PARAM(ADC_CHANNEL_IN, VBAT_MEAS_ADC_IN)
#define ADC_1 AD1_1
#define ADC_1_GPIO_PORT PAL_PORT(LINE_VBAT_MEAS)
#define ADC_1_GPIO_PIN PAL_PAD(LINE_VBAT_MEAS)
#endif

#ifndef USE_ADC_2
#define USE_ADC_2 1
#endif

#if USE_ADC_2
#define AD1_2_CHANNEL CONCAT_BOARD_PARAM(ADC_CHANNEL_IN, ADC_CURR_ADC_IN)
#define ADC_2 AD1_2
#define ADC_2_GPIO_PORT PAL_PORT(LINE_ADC_CURR)
#define ADC_2_GPIO_PIN PAL_PAD(LINE_ADC_CURR)
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_1
#endif

#define DefaultVoltageOfAdc(adc) ((3.3f/4096.0f)*11.08866*adc)

#ifndef ADC_CHANNEL_CURRENT
#define ADC_CHANNEL_CURRENT ADC_2
#endif

#define MilliAmpereOfAdc(adc) (3.3f / 4096.0f)*1095.18629*((float)adc) 



/*
 * PWM defines
 */



// motors,  PWM mode disabled by default (DShot is enabled by default) 

#ifndef USE_PWM1
#define USE_PWM1 0
#endif
#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_GPIO PAL_PORT(LINE_MOTOR_1)
#define PWM_SERVO_1_PIN PAL_PAD(LINE_MOTOR_1)
#define PWM_SERVO_1_AF AF_MOTOR_1
#define PWM_SERVO_1_DRIVER CONCAT_BOARD_PARAM(PWMD, MOTOR_1_TIM)
#define PWM_SERVO_1_CHANNEL (MOTOR_1_TIM_CH-1)
#define PWM_SERVO_1_CONF CONCAT_BOARD_PARAM(pwmcfg, MOTOR_1_TIM)
#endif

#ifndef USE_PWM2
#define USE_PWM2 0
#endif
#if USE_PWM2
#define PWM_SERVO_2 2
#define PWM_SERVO_2_GPIO PAL_PORT(LINE_MOTOR_2)
#define PWM_SERVO_2_PIN PAL_PAD(LINE_MOTOR_2)
#define PWM_SERVO_2_AF AF_MOTOR_2
#define PWM_SERVO_2_DRIVER CONCAT_BOARD_PARAM(PWMD, MOTOR_2_TIM)
#define PWM_SERVO_2_CHANNEL (MOTOR_2_TIM_CH-1)
#define PWM_SERVO_2_CONF CONCAT_BOARD_PARAM(pwmcfg, MOTOR_2_TIM)
#endif

#ifndef USE_PWM3
#define USE_PWM3 0
#endif
#if USE_PWM3
#define PWM_SERVO_3 3
#define PWM_SERVO_3_GPIO PAL_PORT(LINE_MOTOR_3)
#define PWM_SERVO_3_PIN PAL_PAD(LINE_MOTOR_3)
#define PWM_SERVO_3_AF AF_MOTOR_3
#define PWM_SERVO_3_DRIVER CONCAT_BOARD_PARAM(PWMD, MOTOR_3_TIM)
#define PWM_SERVO_3_CHANNEL (MOTOR_3_TIM_CH-1)
#define PWM_SERVO_3_CONF CONCAT_BOARD_PARAM(pwmcfg, MOTOR_3_TIM)
#endif

#ifndef USE_PWM4
#define USE_PWM4 0
#endif
#if USE_PWM4
#define PWM_SERVO_4 4
#define PWM_SERVO_4_GPIO PAL_PORT(LINE_MOTOR_4)
#define PWM_SERVO_4_PIN PAL_PAD(LINE_MOTOR_4)
#define PWM_SERVO_4_AF AF_MOTOR_4
#define PWM_SERVO_4_DRIVER CONCAT_BOARD_PARAM(PWMD, MOTOR_4_TIM)
#define PWM_SERVO_4_CHANNEL (MOTOR_4_TIM_CH-1)
#define PWM_SERVO_4_CONF CONCAT_BOARD_PARAM(pwmcfg, MOTOR_4_TIM)
#endif



// servo index starting at 1 + regular servos + aux servos
// so NB = 1+4
#define ACTUATORS_PWM_NB 5


/**
 * DSHOT
 */



#define DSHOT_TIM2_TELEMETRY_DEV NULL


#ifndef USE_DSHOT_TIM2
#define USE_DSHOT_TIM2 1 // MOTOR_1 MOTOR_2 MOTOR_3 MOTOR_4
#endif



#if USE_DSHOT_TIM2 // MOTOR_1 MOTOR_2 MOTOR_3 MOTOR_4 on TIM3

#define DSHOT_SERVO_1 1
#define DSHOT_SERVO_1_GPIO PAL_PORT(LINE_MOTOR_1)
#define DSHOT_SERVO_1_PIN PAL_PAD(LINE_MOTOR_1)
#define DSHOT_SERVO_1_AF AF_MOTOR_1
#define DSHOT_SERVO_1_DRIVER CONCAT_BOARD_PARAM(DSHOTD, MOTOR_1_TIM)
#define DSHOT_SERVO_1_CHANNEL MOTOR_1_TIM_CH

#define DSHOT_SERVO_2 2
#define DSHOT_SERVO_2_GPIO PAL_PORT(LINE_MOTOR_2)
#define DSHOT_SERVO_2_PIN PAL_PAD(LINE_MOTOR_2)
#define DSHOT_SERVO_2_AF AF_MOTOR_2
#define DSHOT_SERVO_2_DRIVER CONCAT_BOARD_PARAM(DSHOTD, MOTOR_2_TIM)
#define DSHOT_SERVO_2_CHANNEL MOTOR_2_TIM_CH

#define DSHOT_SERVO_3 3
#define DSHOT_SERVO_3_GPIO PAL_PORT(LINE_MOTOR_3)
#define DSHOT_SERVO_3_PIN PAL_PAD(LINE_MOTOR_3)
#define DSHOT_SERVO_3_AF AF_MOTOR_3
#define DSHOT_SERVO_3_DRIVER CONCAT_BOARD_PARAM(DSHOTD, MOTOR_3_TIM)
#define DSHOT_SERVO_3_CHANNEL MOTOR_3_TIM_CH

#define DSHOT_SERVO_4 4
#define DSHOT_SERVO_4_GPIO PAL_PORT(LINE_MOTOR_4)
#define DSHOT_SERVO_4_PIN PAL_PAD(LINE_MOTOR_4)
#define DSHOT_SERVO_4_AF AF_MOTOR_4
#define DSHOT_SERVO_4_DRIVER CONCAT_BOARD_PARAM(DSHOTD, MOTOR_4_TIM)
#define DSHOT_SERVO_4_CHANNEL MOTOR_4_TIM_CH


#define DSHOT_CONF_TIM2 1
#define DSHOT_CONF2_DEF { \
  .dma_stream = STM32_PWM2_UP_DMA_STREAM,   \
  .dma_channel = STM32_PWM2_UP_DMA_CHANNEL, \
  .pwmp = &PWMD2,                           \
  .tlm_sd = DSHOT_TIM2_TELEMETRY_DEV,       \
  .dma_buf = &dshot2DmaBuffer,              \
  .dcache_memory_in_use = false             \
}

#endif




/**
 * UART1, UART3, UART4, UART5, UART6
 * 
 */

#define UART1_GPIO_PORT_TX  PAL_PORT(LINE_UART1_TX)
#define UART1_GPIO_TX       PAL_PAD(LINE_UART1_TX)
#define UART1_GPIO_PORT_RX  PAL_PORT(LINE_UART1_RX)
#define UART1_GPIO_RX       PAL_PAD(LINE_UART1_RX)
#define UART1_GPIO_AF       AF_UART1_TX

#define UART3_GPIO_PORT_TX  PAL_PORT(LINE_UART3_TX)
#define UART3_GPIO_TX       PAL_PAD(LINE_UART3_TX)
#define UART3_GPIO_PORT_RX  PAL_PORT(LINE_UART3_RX)
#define UART3_GPIO_RX       PAL_PAD(LINE_UART3_RX)
#define UART3_GPIO_AF       AF_UART3_TX

#define UART4_GPIO_PORT_TX  PAL_PORT(LINE_UART4_TX)
#define UART4_GPIO_TX       PAL_PAD(LINE_UART4_TX)
#define UART4_GPIO_PORT_RX  PAL_PORT(LINE_UART4_RX)
#define UART4_GPIO_RX       PAL_PAD(LINE_UART4_RX)
#define UART4_GPIO_AF       AF_UART4_TX

#define UART4_GPIO_PORT_TX  PAL_PORT(LINE_UART4_TX)
#define UART4_GPIO_TX       PAL_PAD(LINE_UART4_TX)
#define UART4_GPIO_PORT_RX  PAL_PORT(LINE_UART4_RX)
#define UART4_GPIO_RX       PAL_PAD(LINE_UART4_RX)
#define UART4_GPIO_AF       AF_UART4_TX

#define UART4_GPIO_PORT_TX  PAL_PORT(LINE_UART4_TX)
#define UART4_GPIO_TX       PAL_PAD(LINE_UART4_TX)
#define UART4_GPIO_PORT_RX  PAL_PORT(LINE_UART4_RX)
#define UART4_GPIO_RX       PAL_PAD(LINE_UART4_RX)
#define UART4_GPIO_AF       AF_UART4_TX



/**
 * SBUS / Spektrum port UART2
 *
 */

// In case, do dynamic config of UARTs
#ifndef USE_UART2_RX
#define USE_UART2_RX TRUE
#endif
#ifndef USE_UART2_TX // may be used in half duplex mode
#define USE_UART2_TX FALSE
#endif
// Tx and Rx are configured on the same pin, only one of them should be used
#define UART2_GPIO_PORT_TX  PAL_PORT(LINE_RC1)
#define UART2_GPIO_TX       PAL_PAD(LINE_RC1)
#define UART2_GPIO_PORT_RX  PAL_PORT(LINE_RC1)
#define UART2_GPIO_RX       PAL_PAD(LINE_RC1)
#define UART2_GPIO_AF       RC1_USART_AF

// no wait with chibios as the RTC oscillator takes longer to stabilize
#define SPEKTRUM_BIND_WAIT 30000

/**
 * I2C defines
 */

// Digital noise filter: 0 disabled, [0x1 - 0xF] enable up to n t_I2CCLK
#define STM32_CR1_DNF(n)          ((n & 0x0f) << 8)
// Timing register
#define I2C_FAST_400KHZ_DNF0_100NS_PCLK54MHZ_TIMINGR  (STM32_TIMINGR_PRESC(0U) | \
    STM32_TIMINGR_SCLDEL(10U) | STM32_TIMINGR_SDADEL(0U) | \
    STM32_TIMINGR_SCLH(34U)  | STM32_TIMINGR_SCLL(86U))
#define I2C_STD_100KHZ_DNF0_100NS_PCLK54MHZ_TIMINGR  (STM32_TIMINGR_PRESC(1U) | \
    STM32_TIMINGR_SCLDEL(9U) | STM32_TIMINGR_SDADEL(0U) | \
    STM32_TIMINGR_SCLH(105U)  | STM32_TIMINGR_SCLL(153U))


// Internal I2C (IMU, baro)

#ifndef I2C1_CLOCK_SPEED
#define I2C1_CLOCK_SPEED 400000
#endif
#if I2C1_CLOCK_SPEED == 400000
#define I2C1_DUTY_CYCLE FAST_DUTY_CYCLE_2
#elif I2C1_CLOCK_SPEED == 100000
#define I2C1_DUTY_CYCLE STD_DUTY_CYCLE
#else
#error "Invalid I2C1 clock speed"
#endif
#define I2C1_CFG_DEF {        \
           OPMODE_I2C,        \
           I2C1_CLOCK_SPEED,  \
           I2C1_DUTY_CYCLE,   \
           }

// External I2C

#ifndef I2C2_CLOCK_SPEED
#define I2C2_CLOCK_SPEED 400000
#endif
#if I2C2_CLOCK_SPEED == 400000
#define I2C2_DUTY_CYCLE FAST_DUTY_CYCLE_2
#elif I2C2_CLOCK_SPEED == 100000
#define I2C2_DUTY_CYCLE STD_DUTY_CYCLE
#else
#error "Invalid I2C2 clock speed"
#endif
#define I2C2_CFG_DEF {        \
           OPMODE_I2C,        \
           I2C2_CLOCK_SPEED,  \
           I2C2_DUTY_CYCLE,   \
           }



/**
 * SPI Config
 */

#ifndef USE_SPI1
#define USE_SPI1 TRUE
#endif

#ifndef USE_SPI3
#define USE_SPI3 FALSE
#endif


// SPI 1
#define SPI1_GPIO_AF          AF_SPI1_CLK
#define SPI1_GPIO_PORT_MISO   PAL_PORT(LINE_SPI1_MISO)
#define SPI1_GPIO_MISO        PAL_PAD( LINE_SPI1_MISO)
#define SPI1_GPIO_PORT_MOSI   PAL_PORT(LINE_SPI1_MOSI)
#define SPI1_GPIO_MOSI        PAL_PAD( LINE_SPI1_MOSI)
#define SPI1_GPIO_PORT_SCK    PAL_PORT(LINE_SPI1_CLK)
#define SPI1_GPIO_SCK         PAL_PAD( LINE_SPI1_CLK)

// SPI 2
#define SPI2_GPIO_AF          AF_SPI2_CLK
#define SPI2_GPIO_PORT_MISO   PAL_PORT(LINE_SPI2_MISO)
#define SPI2_GPIO_MISO        PAL_PAD(LINE_SPI2_MISO)
#define SPI2_GPIO_PORT_MOSI   PAL_PORT(LINE_SPI2_MOSI)
#define SPI2_GPIO_MOSI        PAL_PAD(LINE_SPI2_MOSI)
#define SPI2_GPIO_PORT_SCK    PAL_PORT(LINE_SPI2_CLK)
#define SPI2_GPIO_SCK         PAL_PAD(LINE_SPI2_CLK)

// SPI 3
#define SPI3_GPIO_AF          AF_SPI3_CLK
#define SPI3_GPIO_PORT_MISO   PAL_PORT(LINE_SPI3_MISO)
#define SPI3_GPIO_MISO        PAL_PAD( LINE_SPI3_MISO)
#define SPI3_GPIO_PORT_MOSI   PAL_PORT(LINE_SPI3_MOSI)
#define SPI3_GPIO_MOSI        PAL_PAD( LINE_SPI3_MOSI)
#define SPI3_GPIO_PORT_SCK    PAL_PORT(LINE_SPI3_CLK)
#define SPI3_GPIO_SCK         PAL_PAD( LINE_SPI3_CLK)


// GYRO on PA04
#define SPI_SELECT_SLAVE0_PORT  PAL_PORT(LINE_GYRO_CS)
#define SPI_SELECT_SLAVE0_PIN   PAL_PAD(LINE_GYRO_CS)
// OSD_CS on PD05
#define SPI_SELECT_SLAVE1_PORT  PAL_PORT(LINE_OSD_CS)
#define SPI_SELECT_SLAVE1_PIN   PAL_PAD(LINE_OSD_CS)
// FLASH_CS on PB12
#define SPI_SELECT_SLAVE2_PORT  PAL_PORT(LINE_FLASH_CS)
#define SPI_SELECT_SLAVE2_PIN   PAL_PAD(LINE_FLASH_CS)



// bat monitoring for file closing
#define SDLOG_BAT_ADC CONCAT_BOARD_PARAM(ADCD, VBAT_MEAS_ADC)
#define SDLOG_BAT_CHAN CONCAT_BOARD_PARAM(ADC_CHANNEL_IN, VBAT_MEAS_ADC_IN)


/*
 * Actuators for fixedwing
 */
 /* Default actuators driver */
#define DEFAULT_ACTUATORS "modules/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()




#endif /* CONFIG_AIOF7TMOTOR_H */


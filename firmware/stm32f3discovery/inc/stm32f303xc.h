// badprog.com
#ifndef STM32F303XC_H
#define STM32F303XC_H

#include <stdint.h>

typedef volatile uint32_t reg32_t;

// ---------------------------------------------------------------------------
// Bus base addresses
// ---------------------------------------------------------------------------
#define PERIPH_BASE         (0x40000000UL)
#define APB1PERIPH_BASE     (PERIPH_BASE + 0x00000000UL)
#define APB2PERIPH_BASE     (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE     (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE     (PERIPH_BASE + 0x08000000UL)

// ---------------------------------------------------------------------------
// RCC - Reset and Clock Control
// ---------------------------------------------------------------------------
#define RCC_BASE            (AHB1PERIPH_BASE + 0x00001000UL)

typedef struct {
    reg32_t CR;
    reg32_t CFGR;
    reg32_t CIR;
    reg32_t APB2RSTR;
    reg32_t APB1RSTR;
    reg32_t AHBENR;
    reg32_t APB2ENR;
    reg32_t APB1ENR;
    reg32_t BDCR;
    reg32_t CSR;
    reg32_t AHBRSTR;
    reg32_t CFGR2;
    reg32_t CFGR3;
} RCC_TypeDef;

#define RCC                     ((RCC_TypeDef *) RCC_BASE)

// AHB peripheral clock enable bits
#define RCC_AHBENR_IOPAEN       (1UL << 17)
#define RCC_AHBENR_IOPBEN       (1UL << 18)
#define RCC_AHBENR_IOPEEN       (1UL << 21)

// APB2 peripheral clock enable bits
#define RCC_APB2ENR_USART1EN    (1UL << 14)
#define RCC_APB2ENR_SYSCFGEN    (1UL << 0)
#define RCC_APB2ENR_SPI1EN      (1UL << 12)

// APB1 peripheral clock enable bits
#define RCC_APB1ENR_I2C1EN      (1UL << 21)
#define RCC_APB1ENR_WWDGEN      (1UL << 11)

// ---------------------------------------------------------------------------
// GPIO
// ---------------------------------------------------------------------------
#define GPIOA_BASE          (AHB2PERIPH_BASE + 0x00000000UL)
#define GPIOB_BASE          (AHB2PERIPH_BASE + 0x00000400UL)
#define GPIOE_BASE          (AHB2PERIPH_BASE + 0x00001000UL)

typedef struct {
    reg32_t MODER;
    reg32_t OTYPER;
    reg32_t OSPEEDR;
    reg32_t PUPDR;
    reg32_t IDR;
    reg32_t ODR;
    reg32_t BSRR;
    reg32_t LCKR;
    reg32_t AFR[2];
    reg32_t BRR;
} GPIO_TypeDef;

#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)

#define GPIO_MODER_INPUT    (0x0UL)
#define GPIO_MODER_OUTPUT   (0x1UL)
#define GPIO_MODER_AF       (0x2UL)
#define GPIO_MODER_ANALOG   (0x3UL)
#define GPIO_OTYPER_PP      (0x0UL)
#define GPIO_OTYPER_OD      (0x1UL)
#define GPIO_OSPEEDR_LOW    (0x0UL)
#define GPIO_OSPEEDR_MED    (0x1UL)
#define GPIO_OSPEEDR_HIGH   (0x2UL)
#define GPIO_OSPEEDR_VHIGH  (0x3UL)
#define GPIO_PUPDR_NONE     (0x0UL)
#define GPIO_PUPDR_UP       (0x1UL)
#define GPIO_PUPDR_DOWN     (0x2UL)

#define LED_ALL_PINS_OFF    (0xFFUL << 24)

// ---------------------------------------------------------------------------
// SYSCFG
// ---------------------------------------------------------------------------
#define SYSCFG_BASE         (APB2PERIPH_BASE + 0x00000000UL)

typedef struct {
    reg32_t CFGR1;
    reg32_t RCR;
    reg32_t EXTICR[4];
    reg32_t CFGR2;
    reg32_t RESERVED[12];
    reg32_t CFGR3;
} SYSCFG_TypeDef;

#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)

// ---------------------------------------------------------------------------
// EXTI
// ---------------------------------------------------------------------------
#define EXTI_BASE           (APB2PERIPH_BASE + 0x00000400UL)

typedef struct {
    reg32_t IMR;
    reg32_t EMR;
    reg32_t RTSR;
    reg32_t FTSR;
    reg32_t SWIER;
    reg32_t PR;
} EXTI_TypeDef;

#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)

// ---------------------------------------------------------------------------
// NVIC
// ---------------------------------------------------------------------------
typedef enum {
    EXTI0_IRQn  = 6,
} IRQn_Type;

#define NVIC_BASE           (0xE000E100UL)

typedef struct {
    reg32_t ISER[8];
    reg32_t RESERVED0[24];
    reg32_t ICER[8];
    reg32_t RESERVED1[24];
    reg32_t ISPR[8];
    reg32_t RESERVED2[24];
    reg32_t ICPR[8];
    reg32_t RESERVED3[24];
    reg32_t IABR[8];
    reg32_t RESERVED4[56];
    reg32_t IP[240];
} NVIC_TypeDef;

#define NVIC                ((NVIC_TypeDef *) NVIC_BASE)

static inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
    NVIC->IP[(uint32_t)IRQn] = (uint32_t)(priority << 4);
}

static inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
    NVIC->ISER[(uint32_t)IRQn >> 5] = (1UL << ((uint32_t)IRQn & 0x1FUL));
}

// ---------------------------------------------------------------------------
// USART1
// ---------------------------------------------------------------------------
#define USART1_BASE         (APB2PERIPH_BASE + 0x00003800UL)

typedef struct {
    reg32_t CR1;
    reg32_t CR2;
    reg32_t CR3;
    reg32_t BRR;
    reg32_t GTPR;
    reg32_t RTOR;
    reg32_t RQR;
    reg32_t ISR;
    reg32_t ICR;
    reg32_t RDR;
    reg32_t TDR;
} USART_TypeDef;

#define USART1              ((USART_TypeDef *) USART1_BASE)

#define USART_CR1_UE        (1UL << 0)
#define USART_CR1_TE        (1UL << 3)
#define USART_CR1_RE        (1UL << 2)
#define USART_ISR_TXE       (1UL << 7)

// ---------------------------------------------------------------------------
// SPI1 - PA5=SCK (AF5), PA6=MISO (AF5), PA7=MOSI (AF5), PE3=CS
// Used for L3GD20 gyroscope
// ---------------------------------------------------------------------------
#define SPI1_BASE           (APB2PERIPH_BASE + 0x00003000UL)

typedef struct {
    reg32_t CR1;
    reg32_t CR2;
    reg32_t SR;
    reg32_t DR;
    reg32_t CRCPR;
    reg32_t RXCRCR;
    reg32_t TXCRCR;
    reg32_t I2SCFGR;
    reg32_t I2SPR;
} SPI_TypeDef;

#define SPI1                ((SPI_TypeDef *) SPI1_BASE)

// CR1 bits
#define SPI_CR1_CPHA        (1UL << 0)     // Clock phase
#define SPI_CR1_CPOL        (1UL << 1)     // Clock polarity
#define SPI_CR1_MSTR        (1UL << 2)     // Master mode
#define SPI_CR1_BR_DIV8     (0x2UL << 3)   // Baud rate: fPCLK/8 = 1 MHz
#define SPI_CR1_SPE         (1UL << 6)     // SPI enable
#define SPI_CR1_SSI         (1UL << 8)     // Internal NSS high
#define SPI_CR1_SSM         (1UL << 9)     // Software NSS management

// SR bits
#define SPI_SR_RXNE         (1UL << 0)     // Receive buffer not empty
#define SPI_SR_TXE          (1UL << 1)     // Transmit buffer empty
#define SPI_SR_BSY          (1UL << 7)     // SPI busy

// CS pin: PE3
#define L3GD20_CS_PIN       3UL
#define L3GD20_CS_LOW()     (GPIOE->BSRR = (1UL << (L3GD20_CS_PIN + 16)))
#define L3GD20_CS_HIGH()    (GPIOE->BSRR = (1UL << L3GD20_CS_PIN))

// ---------------------------------------------------------------------------
// I2C1 - PB6=SCL (AF4), PB7=SDA (AF4)
// Used for LSM303DLHC accelerometer
// ---------------------------------------------------------------------------
#define I2C1_BASE           (APB1PERIPH_BASE + 0x00005400UL)

typedef struct {
    reg32_t CR1;
    reg32_t CR2;
    reg32_t OAR1;
    reg32_t OAR2;
    reg32_t TIMINGR;
    reg32_t TIMEOUTR;
    reg32_t ISR;
    reg32_t ICR;
    reg32_t PECR;
    reg32_t RXDR;
    reg32_t TXDR;
} I2C_TypeDef;

#define I2C1                ((I2C_TypeDef *) I2C1_BASE)

#define I2C_CR1_PE          (1UL << 0)
#define I2C_CR2_RD_WRN      (1UL << 10)
#define I2C_CR2_START       (1UL << 13)
#define I2C_CR2_STOP        (1UL << 14)
#define I2C_CR2_AUTOEND     (1UL << 25)
#define I2C_CR2_NBYTES_Pos  16
#define I2C_ISR_TXE         (1UL << 0)
#define I2C_ISR_TXIS        (1UL << 1)
#define I2C_ISR_RXNE        (1UL << 2)
#define I2C_ISR_TC          (1UL << 6)
#define I2C_ISR_BUSY        (1UL << 15)

// ---------------------------------------------------------------------------
// IWDG - Independent Watchdog
//
// The IWDG is clocked by the LSI oscillator (~40 kHz), independent of the
// main CPU clock and the RTOS. If the firmware does not reload the counter
// before it reaches zero, the IWDG resets the MCU.
//
// Key registers:
//   KR   : Key register. Write 0xCCCC to start, 0xAAAA to reload (pet),
//          0x5555 to unlock PR and RLR for writing.
//   PR   : Prescaler register. Divides the LSI clock.
//          0 = /4, 1 = /8, 2 = /16, 3 = /32, 4 = /64, 5 = /128, 6 = /256
//   RLR  : Reload register. Counter reloads to this value on each pet.
//          12-bit value (0..4095).
//   SR   : Status register. Bits 0 and 1 indicate update in progress.
//
// Timeout calculation:
//   timeout_ms = (RLR + 1) * prescaler / LSI_freq * 1000
//   With PR=4 (div64), RLR=624, LSI=40000 Hz:
//   timeout = 625 * 64 / 40000 * 1000 = 1000 ms
//   We pet the watchdog every 500ms so we have a 2x safety margin.
// ---------------------------------------------------------------------------
#define IWDG_BASE           (0x40003000UL)

typedef struct {
    reg32_t KR;     // Key register
    reg32_t PR;     // Prescaler register
    reg32_t RLR;    // Reload register
    reg32_t SR;     // Status register
    reg32_t WINR;   // Window register
} IWDG_TypeDef;

#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)

#define IWDG_KR_START       0xCCCCUL   // Start IWDG
#define IWDG_KR_RELOAD      0xAAAAUL   // Pet the watchdog (reload counter)
#define IWDG_KR_UNLOCK      0x5555UL   // Unlock PR and RLR for writing
#define IWDG_PR_DIV64       0x4UL      // Prescaler: LSI / 64
#define IWDG_SR_PVU         (1UL << 0) // Prescaler update in progress
#define IWDG_SR_RVU         (1UL << 1) // Reload value update in progress

// ---------------------------------------------------------------------------
// L3GD20 gyroscope registers
// ---------------------------------------------------------------------------
#define L3GD20_WHO_AM_I       0x0FU
#define L3GD20_WHO_AM_I_ID    0xD4U
#define L3GD20H_WHO_AM_I_ID   0xD7U
#define L3GD20_CTRL_REG1      0x20U
#define L3GD20_CTRL_REG4      0x23U
#define L3GD20_OUT_X_L        0x28U
#define L3GD20_READ           (1UL << 7)
#define L3GD20_MULTI          (1UL << 6)

// ---------------------------------------------------------------------------
// LSM303DLHC accelerometer registers
// ---------------------------------------------------------------------------
#define LSM303_ACCEL_ADDR       0x19U
#define LSM303_CTRL_REG1_A      0x20U
#define LSM303_CTRL_REG1_A_VAL  0x57U
#define LSM303_CTRL_REG4_A      0x23U
#define LSM303_CTRL_REG4_A_VAL  0x08U
#define LSM303_OUT_X_L_A        0x28U

// ---------------------------------------------------------------------------
// Cortex-M4 intrinsics
// ---------------------------------------------------------------------------
static inline void __NOP(void) { __asm volatile ("nop"); }

#endif // STM32F303XC_H
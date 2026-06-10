// badprog.com
/**
 * @file    main.c
 * @brief   FreeRTOS multi-device SPI+I2C with vTaskDelayUntil, stack watermark
 *          and IWDG watchdog on STM32F3Discovery
 *
 * Bare metal C with FreeRTOS. No HAL, no CubeMX.
 *
 * This exercise combines everything seen in previous exercises and adds
 * three new mechanisms:
 *
 *   1. vTaskDelayUntil
 *      Replaces vTaskDelay for strict periodic sensor reads. Guarantees
 *      the gyroscope is read every 50ms and the accelerometer every 100ms
 *      regardless of how long each transaction takes.
 *
 *   2. Stack watermark
 *      stats_task periodically calls uxTaskGetStackHighWaterMark() on each
 *      task and prints the remaining free stack words on UART. Used to size
 *      stacks correctly and detect potential overflows before they happen.
 *
 *   3. IWDG hardware watchdog
 *      watchdog_task runs at the highest priority and pets the IWDG every
 *      500ms. The IWDG timeout is set to 1000ms. If any task monopolises
 *      the CPU and prevents watchdog_task from running, the MCU resets.
 *      This is the last line of defence in production firmware.
 *
 * Tasks:
 *   watchdog_task   (priority 4) : pets IWDG every 500ms via vTaskDelayUntil
 *   gyro_task       (priority 3) : reads L3GD20 via SPI every 50ms
 *   accel_task      (priority 3) : reads LSM303DLHC via I2C every 100ms
 *   display_task    (priority 2) : receives from queues, prints via UART
 *   stats_task      (priority 1) : prints stack watermarks every 5s
 *   led_task        (priority 1) : rotates compass LEDs
 *
 * Mutexes:
 *   xSpiMutex  : protects SPI1 bus (used by gyro_task only here, but
 *                correct practice for a shared bus)
 *   xI2cMutex  : protects I2C1 bus (used by accel_task only here)
 *   xUartMutex : protects USART1 (shared by display_task and stats_task)
 *
 * Hardware:
 *   PA5  : SPI1 SCK  (AF5)
 *   PA6  : SPI1 MISO (AF5)
 *   PA7  : SPI1 MOSI (AF5)
 *   PE3  : L3GD20 CS (GPIO output)
 *   PB6  : I2C1 SCL  (AF4)
 *   PB7  : I2C1 SDA  (AF4)
 *   PA9  : USART1 TX (AF7, 115200 baud)
 *   PE8..PE15: compass LEDs
 *
 * CPU clock: 8 MHz (default HSI)
 *
 * -------------------------------------------------------------------------
 * NEW FREERTOS AND HARDWARE CONCEPTS
 * -------------------------------------------------------------------------
 *
 * vTaskDelayUntil(pxPreviousWakeTime, xTimeIncrement):
 *   Suspends the calling task until an absolute time is reached, calculated
 *   from the previous wake time. This makes the period independent of the
 *   task execution time.
 *   Contrast with vTaskDelay(100):
 *     period = execution_time + 100ms  (drifts over time)
 *   With vTaskDelayUntil(&t, pdMS_TO_TICKS(100)):
 *     period = exactly 100ms  (constant, no drift)
 *   The pxPreviousWakeTime variable must be initialised to xTaskGetTickCount()
 *   before the first call. FreeRTOS updates it automatically on each call.
 *
 * uxTaskGetStackHighWaterMark(handle):
 *   Returns the minimum number of free words (4 bytes each) ever observed
 *   in the task's stack since it was created. FreeRTOS fills each stack
 *   with 0xA5 at creation time and scans from the bottom to find the
 *   highest point ever reached.
 *   A value close to 0 means the stack almost overflowed.
 *   A large value means the stack is oversized and can be reduced.
 *   Pass NULL to query the calling task's own stack.
 *   Pass a TaskHandle_t to query any specific task.
 *
 * IWDG (Independent Watchdog):
 *   A hardware timer clocked by the LSI oscillator (~40 kHz), completely
 *   independent of the CPU, RTOS, and main clock. Once started it cannot
 *   be stopped. If the firmware does not write 0xAAAA to IWDG_KR before
 *   the counter reaches zero, the MCU resets unconditionally.
 *   Sequence to start:
 *     1. Write 0x5555 to KR  (unlock PR and RLR)
 *     2. Write prescaler to PR
 *     3. Write reload value to RLR
 *     4. Write 0xCCCC to KR  (start the watchdog)
 *     5. Write 0xAAAA to KR  periodically (pet the watchdog)
 *   Once started, the IWDG cannot be stopped until the next reset.
 * -------------------------------------------------------------------------
 */

#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f303xc.h"

// ---------------------------------------------------------------------------
// Shared FreeRTOS objects
// ---------------------------------------------------------------------------

#define GYRO_QUEUE_LENGTH   5
#define ACCEL_QUEUE_LENGTH  5

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} GyroData;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} AccelData;

static QueueHandle_t    xGyroQueue   = NULL;
static QueueHandle_t    xAccelQueue  = NULL;
static SemaphoreHandle_t xSpiMutex   = NULL;
static SemaphoreHandle_t xI2cMutex   = NULL;
static SemaphoreHandle_t xUartMutex  = NULL;

// Task handles needed for stack watermark queries
static TaskHandle_t xWatchdogTask = NULL;
static TaskHandle_t xGyroTask     = NULL;
static TaskHandle_t xAccelTask    = NULL;
static TaskHandle_t xDisplayTask  = NULL;
static TaskHandle_t xStatsTask    = NULL;
static TaskHandle_t xLedTask      = NULL;

// ---------------------------------------------------------------------------
// Task stack sizes (words)
// ---------------------------------------------------------------------------

#define WATCHDOG_TASK_STACK 128
#define GYRO_TASK_STACK     256
#define ACCEL_TASK_STACK    256
#define DISPLAY_TASK_STACK  384
#define STATS_TASK_STACK    256
#define LED_TASK_STACK      128

// ---------------------------------------------------------------------------
// UART: USART1 on PA9, 115200 baud
// ---------------------------------------------------------------------------

static void uart_init(void)
{
    RCC->AHBENR  |= RCC_AHBENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    GPIOA->MODER  &= ~(0x3UL << 18);
    GPIOA->MODER  |=  (0x2UL << 18);
    GPIOA->AFR[1] &= ~(0xFUL << 4);
    GPIOA->AFR[1] |=  (0x7UL << 4);

    USART1->BRR = 69;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE;
}

static void uart_print_safe(const char *str)
{
    xSemaphoreTake(xUartMutex, portMAX_DELAY);
    const char *p = str;
    while (*p) {
        while (!(USART1->ISR & USART_ISR_TXE));
        USART1->TDR = (uint32_t)(*p++);
    }
    xSemaphoreGive(xUartMutex);
}

// ---------------------------------------------------------------------------
// LEDs: PE8..PE15
// ---------------------------------------------------------------------------

static void leds_init(void)
{
    RCC->AHBENR |= RCC_AHBENR_IOPEEN;
    GPIOE->MODER &= ~(0xFFFFUL << 16);
    GPIOE->MODER |=  (0x5555UL << 16);
    GPIOE->BSRR   = LED_ALL_PINS_OFF;
}

// ---------------------------------------------------------------------------
// SPI1: PA5=SCK, PA6=MISO, PA7=MOSI (AF5), PE3=CS
// Mode 3 (CPOL=1, CPHA=1), 1 MHz, 8-bit frames
// ---------------------------------------------------------------------------

static void spi1_init(void)
{
    RCC->AHBENR  |= RCC_AHBENR_IOPAEN | RCC_AHBENR_IOPEEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // PA5 (SCK), PA6 (MISO), PA7 (MOSI) as AF5
    GPIOA->MODER &= ~((0x3UL << 10) | (0x3UL << 12) | (0x3UL << 14));
    GPIOA->MODER |=  (0x2UL << 10) | (0x2UL << 12) | (0x2UL << 14);
    GPIOA->OSPEEDR |= (0x3UL << 10) | (0x3UL << 12) | (0x3UL << 14);
    GPIOA->AFR[0] &= ~((0xFUL << 20) | (0xFUL << 24) | (0xFUL << 28));
    GPIOA->AFR[0] |=  (0x5UL << 20) | (0x5UL << 24) | (0x5UL << 28);

    // PE3 as CS output, deselected
    GPIOE->MODER &= ~(0x3UL << (L3GD20_CS_PIN * 2));
    GPIOE->MODER |=  (0x1UL << (L3GD20_CS_PIN * 2));
    L3GD20_CS_HIGH();

    // SPI1: master, mode 3 (CPOL=1 CPHA=1), fPCLK/8=1MHz, SSM+SSI
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_CPHA |
                SPI_CR1_BR_DIV8 | SPI_CR1_SSM | SPI_CR1_SSI;

    // 8-bit data size, RXNE fires at 1 byte
    SPI1->CR2 = (0x7UL << 8) | (1UL << 12);

    SPI1->CR1 |= SPI_CR1_SPE;
}

static uint8_t spi1_transfer(uint8_t data)
{
    while (!(SPI1->SR & SPI_SR_TXE));
    *((volatile uint8_t *)&SPI1->DR) = data;
    while (!(SPI1->SR & SPI_SR_RXNE));
    return *((volatile uint8_t *)&SPI1->DR);
}

// ---------------------------------------------------------------------------
// I2C1: PB6=SCL, PB7=SDA (AF4), 100 kHz
// ---------------------------------------------------------------------------

static void i2c1_init(void)
{
    RCC->AHBENR  |= RCC_AHBENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER   &= ~((0x3UL << 12) | (0x3UL << 14));
    GPIOB->MODER   |=  (0x2UL << 12) | (0x2UL << 14);
    GPIOB->OTYPER  |=  (1UL << 6) | (1UL << 7);
    GPIOB->OSPEEDR |=  (0x3UL << 12) | (0x3UL << 14);
    GPIOB->PUPDR   &= ~((0x3UL << 12) | (0x3UL << 14));
    GPIOB->PUPDR   |=  (0x1UL << 12) | (0x1UL << 14);
    GPIOB->AFR[0]  &= ~((0xFUL << 24) | (0xFUL << 28));
    GPIOB->AFR[0]  |=  (0x4UL << 24) | (0x4UL << 28);

    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->TIMINGR = 0x10420F13U;
    I2C1->CR1 |= I2C_CR1_PE;
}

static void i2c_wait_busy(void)  { while (I2C1->ISR & I2C_ISR_BUSY); }
static void i2c_wait_tc(void)    { while (!(I2C1->ISR & I2C_ISR_TC)); }
static void i2c_write_byte(uint8_t b) { while (!(I2C1->ISR & I2C_ISR_TXIS)); I2C1->TXDR = b; }
static uint8_t i2c_read_byte(void)   { while (!(I2C1->ISR & I2C_ISR_RXNE)); return (uint8_t)I2C1->RXDR; }

static void i2c_start_write(uint8_t addr, uint8_t n)
{
    I2C1->CR2 = ((uint32_t)(addr << 1)) | ((uint32_t)n << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
}

static void i2c_start_read(uint8_t addr, uint8_t n)
{
    I2C1->CR2 = ((uint32_t)(addr << 1)) | I2C_CR2_RD_WRN
              | ((uint32_t)n << I2C_CR2_NBYTES_Pos) | I2C_CR2_START | I2C_CR2_AUTOEND;
}

static void i2c_stop(void) { I2C1->CR2 |= I2C_CR2_STOP; while (I2C1->ISR & I2C_ISR_BUSY); }

// ---------------------------------------------------------------------------
// L3GD20 gyroscope
// ---------------------------------------------------------------------------

static void l3gd20_init(void)
{
    L3GD20_CS_LOW();
    spi1_transfer(L3GD20_CTRL_REG1 & ~L3GD20_READ);
    spi1_transfer(0x0FU);   // ODR=95Hz, all axes on
    L3GD20_CS_HIGH();

    L3GD20_CS_LOW();
    spi1_transfer(L3GD20_CTRL_REG4 & ~L3GD20_READ);
    spi1_transfer(0x00U);   // +/-250 dps, little-endian
    L3GD20_CS_HIGH();
}

static void l3gd20_read_xyz(GyroData *data)
{
    L3GD20_CS_LOW();
    spi1_transfer(L3GD20_OUT_X_L | L3GD20_READ | L3GD20_MULTI);
    uint8_t xl = spi1_transfer(0x00);
    uint8_t xh = spi1_transfer(0x00);
    uint8_t yl = spi1_transfer(0x00);
    uint8_t yh = spi1_transfer(0x00);
    uint8_t zl = spi1_transfer(0x00);
    uint8_t zh = spi1_transfer(0x00);
    L3GD20_CS_HIGH();

    data->x = (int16_t)((xh << 8) | xl);
    data->y = (int16_t)((yh << 8) | yl);
    data->z = (int16_t)((zh << 8) | zl);
}

// ---------------------------------------------------------------------------
// LSM303DLHC accelerometer
// ---------------------------------------------------------------------------

static void lsm303_init(void)
{
    i2c_wait_busy();
    i2c_start_write(LSM303_ACCEL_ADDR, 2);
    i2c_write_byte(LSM303_CTRL_REG1_A);
    i2c_write_byte(LSM303_CTRL_REG1_A_VAL);
    i2c_wait_tc();
    i2c_stop();

    i2c_wait_busy();
    i2c_start_write(LSM303_ACCEL_ADDR, 2);
    i2c_write_byte(LSM303_CTRL_REG4_A);
    i2c_write_byte(LSM303_CTRL_REG4_A_VAL);
    i2c_wait_tc();
    i2c_stop();
}

static void lsm303_read_xyz(AccelData *data)
{
    uint8_t buf[6];
    i2c_wait_busy();
    i2c_start_write(LSM303_ACCEL_ADDR, 1);
    i2c_write_byte(LSM303_OUT_X_L_A | 0x80U);
    i2c_wait_tc();
    i2c_start_read(LSM303_ACCEL_ADDR, 6);
    for (uint8_t i = 0; i < 6; i++) { buf[i] = i2c_read_byte(); }
    data->x = (int16_t)((uint16_t)(buf[1] << 8) | buf[0]) >> 4;
    data->y = (int16_t)((uint16_t)(buf[3] << 8) | buf[2]) >> 4;
    data->z = (int16_t)((uint16_t)(buf[5] << 8) | buf[4]) >> 4;
}

// ---------------------------------------------------------------------------
// IWDG: Independent Watchdog
// Timeout: ~1000ms (PR=div64, RLR=624, LSI=40kHz)
// watchdog_task pets it every 500ms (2x safety margin)
// ---------------------------------------------------------------------------

static void iwdg_init(void)
{
    // Unlock PR and RLR for writing
    IWDG->KR = IWDG_KR_UNLOCK;

    // Wait for any pending prescaler update
    while (IWDG->SR & IWDG_SR_PVU);

    // Set prescaler to /64: effective tick = 64/40000 = 1.6 ms
    IWDG->PR = IWDG_PR_DIV64;

    // Wait for any pending reload update
    while (IWDG->SR & IWDG_SR_RVU);

    // Set reload to 624: timeout = (624+1) * 1.6ms = 1000ms
    IWDG->RLR = 624;

    // Start the watchdog
    // Once started, it cannot be stopped until the next MCU reset
    IWDG->KR = IWDG_KR_START;

    // Pet immediately to load the reload value
    IWDG->KR = IWDG_KR_RELOAD;
}

// ---------------------------------------------------------------------------
// Simple integer to string helper (avoids printf overhead)
// ---------------------------------------------------------------------------

static int int16_to_str(char *buf, int16_t v)
{
    int n = 0;
    buf[n++] = (v < 0) ? '-': ' ';
    if (v < 0) v = -v;
    uint8_t started = 0;
    for (int16_t div = 1000; div >= 1; div /= 10) {
        int16_t digit = v / div;
        v %= div;
        if (digit || started || div == 1) {
            buf[n++] = '0' + (char)digit;
            started = 1;
        } else {
            buf[n++] = ' ';
        }
    }
    return n;
}

// ---------------------------------------------------------------------------
// Task 1: watchdog_task (priority 4 = highest)
//
// Pets the IWDG every 500ms using vTaskDelayUntil for strict timing.
// If this task stops running (blocked by a runaway task), the IWDG
// expires after 1000ms and resets the MCU.
// ---------------------------------------------------------------------------

static void watchdog_task(void *pvParameters)
{
    (void)pvParameters;

    // Initialise the wake time to the current tick count.
    // vTaskDelayUntil will update this automatically on each call.
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // Pet the watchdog: write the magic key to KR
        IWDG->KR = IWDG_KR_RELOAD;

        // Sleep until exactly 500ms after the last wake time.
        // Even if this task takes a few microseconds to execute,
        // the next wake will be at xLastWakeTime + 500ms, not
        // (now + 500ms). No drift.
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
    }
}

// ---------------------------------------------------------------------------
// Task 2: gyro_task (priority 3)
//
// Reads L3GD20 every 50ms using vTaskDelayUntil.
// Takes the SPI mutex before accessing the bus, releases after.
// ---------------------------------------------------------------------------

static void gyro_task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    GyroData data;

    while (1) {
        xSemaphoreTake(xSpiMutex, portMAX_DELAY);
        l3gd20_read_xyz(&data);
        xSemaphoreGive(xSpiMutex);

        xQueueSend(xGyroQueue, &data, 0);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

// ---------------------------------------------------------------------------
// Task 3: accel_task (priority 3)
//
// Reads LSM303DLHC every 100ms using vTaskDelayUntil.
// Takes the I2C mutex before accessing the bus, releases after.
// ---------------------------------------------------------------------------

static void accel_task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    AccelData data;

    while (1) {
        xSemaphoreTake(xI2cMutex, portMAX_DELAY);
        lsm303_read_xyz(&data);
        xSemaphoreGive(xI2cMutex);

        xQueueSend(xAccelQueue, &data, 0);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

// ---------------------------------------------------------------------------
// Task 4: display_task (priority 2)
//
// Drains both queues and prints a combined line on UART.
// Gyro runs at 50ms, accel at 100ms, so display runs at 50ms driven
// by the gyro queue. Every other cycle also has accel data.
// ---------------------------------------------------------------------------

static void display_task(void *pvParameters)
{
    (void)pvParameters;

    uart_print_safe("FreeRTOS multi-device SPI+I2C\r\n");
    uart_print_safe("---\r\n");
    uart_print_safe("  GX      GY      GZ    |   AX     AY     AZ\r\n");

    GyroData  gdata;
    AccelData adata;
    char buf[80];
    int n;

    while (1) {
        // Block on gyro queue (50ms period)
        if (xQueueReceive(xGyroQueue, &gdata, portMAX_DELAY) == pdTRUE) {
            n = 0;
            n += int16_to_str(buf + n, gdata.x); buf[n++] = ' ';
            n += int16_to_str(buf + n, gdata.y); buf[n++] = ' ';
            n += int16_to_str(buf + n, gdata.z);
            buf[n++] = ' '; buf[n++] = '|'; buf[n++] = ' ';

            // Try to get accel data if available (non-blocking)
            if (xQueueReceive(xAccelQueue, &adata, 0) == pdTRUE) {
                n += int16_to_str(buf + n, adata.x); buf[n++] = ' ';
                n += int16_to_str(buf + n, adata.y); buf[n++] = ' ';
                n += int16_to_str(buf + n, adata.z);
            } else {
                // No new accel data this cycle
                const char *na = "  ---    ---    ---";
                int i = 0;
                while (na[i]) buf[n++] = na[i++];
            }

            buf[n++] = '\r'; buf[n++] = '\n'; buf[n] = '\0';
            uart_print_safe(buf);
        }
    }
}

// ---------------------------------------------------------------------------
// Task 5: stats_task (priority 1)
//
// Every 5 seconds, prints the stack high watermark of every task.
// The watermark is the minimum number of free words ever observed.
// A low value (< 20) means the stack is dangerously close to overflow.
// ---------------------------------------------------------------------------

static void stats_task(void *pvParameters)
{
    (void)pvParameters;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    char buf[64];
    int n;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5000));

        uart_print_safe("--- stack watermarks (words free) ---\r\n");

        // Helper macro to print one task's watermark
        // Format: "taskname: NNN\r\n"
        #define PRINT_WM(name_str, handle) \
            do { \
                UBaseType_t wm = uxTaskGetStackHighWaterMark(handle); \
                n = 0; \
                const char *nm = name_str; \
                while (*nm) buf[n++] = *nm++; \
                buf[n++] = ':'; buf[n++] = ' '; \
                uint16_t w = (uint16_t)wm; \
                uint8_t s2 = 0; \
                for (uint16_t d = 1000; d >= 1; d /= 10) { \
                    uint16_t dg = w / d; w %= d; \
                    if (dg || s2 || d == 1) { buf[n++] = '0' + dg; s2 = 1; } \
                    else buf[n++] = ' '; \
                } \
                buf[n++] = '\r'; buf[n++] = '\n'; buf[n] = '\0'; \
                uart_print_safe(buf); \
            } while (0)

        PRINT_WM("watchdog", xWatchdogTask);
        PRINT_WM("gyro    ", xGyroTask);
        PRINT_WM("accel   ", xAccelTask);
        PRINT_WM("display ", xDisplayTask);
        PRINT_WM("stats   ", xStatsTask);
        PRINT_WM("leds    ", xLedTask);

        uart_print_safe("---\r\n");
    }
}

// ---------------------------------------------------------------------------
// Task 6: led_task (priority 1)
// ---------------------------------------------------------------------------

static void led_task(void *pvParameters)
{
    (void)pvParameters;
    uint8_t led = 8;

    while (1) {
        GPIOE->BSRR = LED_ALL_PINS_OFF;
        GPIOE->BSRR = (1UL << led);
        led++;
        if (led > 15) led = 8;
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(void)
{
    uart_init();
    leds_init();
    spi1_init();
    i2c1_init();
    l3gd20_init();
    lsm303_init();

    // Start IWDG before creating tasks.
    // Once started it cannot be stopped. watchdog_task must pet it
    // within 1000ms or the MCU resets.
    iwdg_init();

    xGyroQueue  = xQueueCreate(GYRO_QUEUE_LENGTH,  sizeof(GyroData));
    xAccelQueue = xQueueCreate(ACCEL_QUEUE_LENGTH, sizeof(AccelData));
    xSpiMutex   = xSemaphoreCreateMutex();
    xI2cMutex   = xSemaphoreCreateMutex();
    xUartMutex  = xSemaphoreCreateMutex();

    if (!xGyroQueue || !xAccelQueue || !xSpiMutex || !xI2cMutex || !xUartMutex) {
        while (1);
    }

    xTaskCreate(watchdog_task, "watchdog", WATCHDOG_TASK_STACK, NULL, 4, &xWatchdogTask);
    xTaskCreate(gyro_task,     "gyro",     GYRO_TASK_STACK,     NULL, 3, &xGyroTask);
    xTaskCreate(accel_task,    "accel",    ACCEL_TASK_STACK,    NULL, 3, &xAccelTask);
    xTaskCreate(display_task,  "display",  DISPLAY_TASK_STACK,  NULL, 2, &xDisplayTask);
    xTaskCreate(stats_task,    "stats",    STATS_TASK_STACK,    NULL, 1, &xStatsTask);
    xTaskCreate(led_task,      "leds",     LED_TASK_STACK,      NULL, 1, &xLedTask);

    vTaskStartScheduler();

    while (1);
    return 0;
}
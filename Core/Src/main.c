#include "main.h"
#include <math.h>

/*****************************************************************************
 *
 *  Robot v1: reactive obstacle avoidance
 *  MPU6050 heading + safety, HC-SR04 distance, two DC motors (L298N)
 *
 *  Target : STM32 Nucleo-F446RE, bare-metal (register level)
 *  Clock  : HSI 16 MHz, APB1 = 16 MHz (no SystemClock_Config)
 *
 *  Pins
 *  Motor A (left)  : PA6 = ENA (TIM3_CH1, AF2), PA8 = IN1, PA9 = IN2
 *  Motor B (right) : PA7 = ENB (TIM3_CH2, AF2), PB4 = IN3, PB5 = IN4
 *  MPU6050         : PB6 = SCL, PB7 = SDA (I2C1, AF4), 100 kHz, addr 0x68
 *  HC-SR04         : PB0 = TRIG (out), PB1 = ECHO (EXTI1, via divider)
 *  User button B1  : PC13 (active low, start / stop / clear fault)
 *  USART2          : PA2 = TX, PA3 = RX (AF7), 115200 8N1 via ST-Link VCP
 *
 *  Timers
 *  SysTick : 1 ms tick (ms_ticks), scheduler and timeouts
 *  TIM2    : free-running 32-bit counter at 1 MHz (1 us per tick)
 *  TIM3    : motor PWM, PSC = 89, ARR = 999 -> ~178 Hz
 *
 ****************************************************************************/

/* ===========================================================================
 *
 *  CONFIGURATION
 *  Addresses, register map, timing, tuning and safety thresholds
 *
 * ======================================================================== */

/* MPU6050 I2C addresses */
#define MPU6050_ADDR_WRITE  0xD0    // 0x68 << 1
#define MPU6050_ADDR_READ   0xD1    // (0x68 << 1) | 1

/* MPU6050 registers */
#define SMPLRT_DIV          0x19
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define ACCEL_XOUT_H        0x3B    // 0x3B..0x48: accel(6) temp(2) gyro(6)
#define PWR_MGMT_1          0x6B
#define WHO_AM_I            0x75

/* Scheduler periods */
#define IMU_PERIOD_MS       5       // 200 Hz IMU read + heading
#define US_PERIOD_MS        60      // HC-SR04 trigger (>= 60 ms between pings)
#define CTRL_PERIOD_MS      20      // 50 Hz state machine
#define PRINT_PERIOD_MS     100     // 10 Hz UART output
#define US_STARTUP_MS       2000    // HC-SR04 needs time after power-up

/* I2C / IMU */
#define I2C_TIMEOUT_MS      3
#define CAL_SAMPLES         200     // ~1 s at 5 ms
#define CAL_MAX_PP          300     // max gz peak-to-peak during calibration
#define CAL_MIN_AZ_G        0.8f    // flat: az ~1.0 g (this chip reads ~1.13 g)
#define GYRO_LSB_PER_DPS    131.0f  // +-250 dps
#define ACCEL_LSB_PER_G     16384.0f// +-2 g

/* Motors (TIM3: duty 0..PWM_MAX) */
#define PWM_MAX             999
#define DUTY_FORWARD        600
#define DUTY_TURN           550

/* Behaviour */
#define OBSTACLE_CM         25      // stop below this
#define CLEAR_CM            35      // drive on above this (hysteresis)
#define TURN_DEG            90.0f   // turn angle per avoidance
#define SETTLE_MS           300     // stand still before deciding
#define TURN_TIMEOUT_MS     4000    // no 90 deg in 4 s -> fault
#define US_MAX_ECHO_US      25000   // longer echo = nothing in range
#define US_FAR_CM           400
#define US_FRESH_MS         250     // distance older = not usable (clone echo ~200 ms)
#define US_STALE_MS         1000    // no echo at all for 1 s -> fault

/* Safety */
#define COLLISION_G         0.6f    // horizontal accel spike
#define COLLISION_SAMPLES   2       // consecutive samples over threshold
#define BLANK_MS            300     // ignore spikes after motor command change
#define TIP_AZ_G            0.5f    // az below this = tipped / lifted sideways
#define TIP_SAMPLES         20      // 100 ms
#define IMU_FAIL_LIMIT      20      // 100 ms of failed IMU reads -> fault

/* ===========================================================================
 *
 *  TYPES AND STATE
 *
 * ======================================================================== */

typedef enum
{
    ST_IDLE,        // motors off, waiting for B1
    ST_SETTLE,      // motors off, waiting for a fresh distance
    ST_FORWARD,     // driving straight
    ST_TURN,        // turning in place until TURN_DEG
    ST_FAULT        // motors off until B1
} state_t;

typedef enum
{
    F_NONE,
    F_COLLISION,
    F_TIPPED,
    F_IMU,
    F_ULTRASONIC,
    F_TURN_TIMEOUT
} fault_t;

static const char *const state_name[] = { "IDLE", "SETTLE", "FWD", "TURN", "FAULT" };
static const char *const fault_name[] = { "", "collision", "tipped", "imu no data", "ultrasonic", "turn timeout" };

/* IMU */
static float    heading;            // deg since boot, + = counterclockwise
static float    gz_off, ax_off, ay_off;
static float    accel_xy_g;         // horizontal (X/Y) accel of the latest sample, g
static float    accel_xy_max_g;     // largest accel_xy_g since the last print, g
static uint32_t imu_last_ok;        // time of last good sample (dt base)
static uint32_t imu_read_errors;    // failed IMU reads since boot (never reset)
static uint32_t imu_fail_streak;    // consecutive failed reads (0 after a good read)
static uint8_t  collision_flag;
static uint8_t  tipped_flag;

/* HC-SR04 (written in EXTI1 ISR) */
static volatile uint32_t echo_t_rise;
static volatile uint32_t echo_width_us;
static volatile uint8_t  echo_high;
static volatile uint8_t  echo_new;
static uint32_t dist_cm = US_FAR_CM;
static uint32_t dist_ms;            // time of last valid measurement

/* Motors */
static int16_t  mot_left, mot_right;
static uint32_t motor_change_ms;    // last command change (collision blanking)

/* State machine */
static state_t  state = ST_IDLE;
static fault_t  fault = F_NONE;
static uint32_t state_ms;
static float    turn_start;

/* ===========================================================================
 *
 *  PROTOTYPES
 *
 * ======================================================================== */

/* Time base */
static void systick_init(void);
static void delay_ms(uint32_t ms);
static void TIM2_Init(void);
static void delay_us(uint32_t us);

/* Tasks */
static void IMU_Calibrate(void);
static void IMU_Task(uint32_t now);
static void HCSR04_Trigger(void);
static void HCSR04_Process(uint32_t now);
static void Control_Task(uint32_t now);
static void Print_Task(void);

/* State machine helpers */
static void enter_state(state_t s, uint32_t now);
static void enter_fault(fault_t f, uint32_t now);
static uint8_t is_running(void);

/* Drivers */
static void Motor_Init(void);
static void Motor_Set(int16_t left, int16_t right);
static void HCSR04_Init(void);
static void Button_Init(void);
static uint8_t Button_Pressed(void);

/* I2C1 low-level primitives (return 0 = OK, -1 = error/timeout) */
static void I2C1_Init(void);
static void I2C1_Recover(void);
static int  I2C1_WaitSR1(uint32_t flag);
static int  I2C1_Start(void);
static void I2C1_Stop(void);
static int  I2C1_WriteAddr(uint8_t addr);
static int  I2C1_WriteData(uint8_t data);

/* MPU6050 driver */
static int  MPU6050_Init(void);
static int  MPU6050_WriteReg(uint8_t reg, uint8_t value);
static int  MPU6050_ReadReg(uint8_t reg, uint8_t *value);
static int  MPU6050_ReadBurst(uint8_t reg, uint8_t *buf, uint8_t n);
static int  MPU6050_ReadAll(int16_t *ax, int16_t *ay, int16_t *az,
                            int16_t *gx, int16_t *gy, int16_t *gz);

/* UART2 output */
static void UART2_Init(void);
static void UART2_SendChar(char c);
static void UART2_SendString(const char *str);
static void UART2_SendInt(int32_t num);
static void UART2_SendHex8(uint8_t v);
static void UART2_SendFixed2(float v);

/* ===========================================================================
 *
 *  TIME BASE
 *  SysTick 1 ms tick (ms_ticks), delay_ms(); TIM2 1 us counter, delay_us()
 *
 * ======================================================================== */

volatile uint32_t ms_ticks = 0;

/* SysTick_Handler in stm32f4xx_it.c is commented out, so it lives here */
void SysTick_Handler(void)
{
    ms_ticks++;
}

static void systick_init(void)
{
    SysTick->LOAD = 16000 - 1;                  // 16 MHz / 16000 = 1 kHz
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk  // processor clock
                  | SysTick_CTRL_TICKINT_Msk    // interrupt every tick
                  | SysTick_CTRL_ENABLE_Msk;
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = ms_ticks;
    while ((ms_ticks - start) < ms);
}

/* TIM2: 32-bit, 16 MHz / (15 + 1) = 1 MHz, free running, wraps after ~71 min.
 * Unsigned differences (now - then) stay correct across the wrap. */
static void TIM2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->CR1 = 0;
    TIM2->PSC = 15;                     // 1 us per tick
    TIM2->ARR = 0xFFFFFFFFU;
    TIM2->CNT = 0;
    TIM2->EGR = TIM_EGR_UG;             // load PSC now (PSC is buffered)
    TIM2->CR1 = TIM_CR1_CEN;
}

static void delay_us(uint32_t us)
{
    uint32_t start = TIM2->CNT;
    while ((TIM2->CNT - start) < us);
}

/* ===========================================================================
 *
 *  MAIN
 *  Init, IMU calibration, cooperative scheduler on ms_ticks
 *
 * ======================================================================== */

int main(void)
{
    systick_init();         // first: I2C timeouts use ms_ticks
    UART2_Init();
    TIM2_Init();
    Motor_Init();           // early: motor pins defined low = motors off
    HCSR04_Init();
    Button_Init();
    I2C1_Init();

    delay_ms(100);          // MPU6050 power-up time

    UART2_SendString("\r\nRobot v1  build 2026-11-17c (imu_err naming)\r\n");

    /* --- Boot diagnostics: bus lines and BUSY flag ------------------------ */
    UART2_SendString("SCL=");  UART2_SendInt((GPIOB->IDR >> 6) & 1);
    UART2_SendString(" SDA="); UART2_SendInt((GPIOB->IDR >> 7) & 1);
    UART2_SendString(" BUSY="); UART2_SendInt((I2C1->SR2 & I2C_SR2_BUSY) ? 1 : 0);
    UART2_SendString("\r\n");

    if (MPU6050_Init() != 0)
        UART2_SendString("MPU6050 init failed\r\n");

    uint8_t who = 0;
    if (MPU6050_ReadReg(WHO_AM_I, &who) == 0)
    {
        UART2_SendString("WHO_AM_I: 0x");
        UART2_SendHex8(who);                    // expect 0x68
        UART2_SendString("\r\n");
    }
    else
    {
        UART2_SendString("WHO_AM_I read failed\r\n");
    }

    IMU_Calibrate();        // robot flat and still

    /* --- HC-SR04 startup time -------------------------------------------- */
    while (ms_ticks < US_STARTUP_MS);

    UART2_SendString("Ready. Press B1 to start / stop.\r\n");

    /* --- Scheduler -------------------------------------------------------- */
    uint32_t t_imu   = ms_ticks;
    uint32_t t_us    = t_imu;
    uint32_t t_ctrl  = t_imu;
    uint32_t t_print = t_imu;

    imu_last_ok = t_imu;
    dist_ms     = t_imu;
    state_ms    = t_imu;

    while (1)
    {
        uint32_t now = ms_ticks;

        if ((now - t_imu) >= IMU_PERIOD_MS)
        {
            t_imu = now;
            IMU_Task(now);
        }

        if ((now - t_us) >= US_PERIOD_MS)
        {
            t_us = now;
            HCSR04_Trigger();
        }

        HCSR04_Process(now);                    // picks up echo from the ISR

        if ((now - t_ctrl) >= CTRL_PERIOD_MS)
        {
            t_ctrl = now;
            Control_Task(now);
        }

        if ((now - t_print) >= PRINT_PERIOD_MS)
        {
            t_print = now;
            Print_Task();                       // blocking ~8 ms, dt is measured
        }
    }
}

/* ===========================================================================
 *
 *  IMU TASK
 *  Startup calibration, heading integration, collision and tip detection
 *
 * ======================================================================== */

/* Average gz, ax, ay while the robot is flat and still. Retries until:
 *  - all CAL_SAMPLES reads succeeded (I2C OK)
 *  - gz peak-to-peak below CAL_MAX_PP (not moving)
 *  - az above CAL_MIN_AZ_G (Z points up: board mounted flat, chip up) */
static void IMU_Calibrate(void)
{
    int16_t ax, ay, az, gx, gy, gz;

    for (;;)
    {
        int32_t sgz = 0, sax = 0, say = 0, saz = 0;
        int16_t gmin = 32767, gmax = -32768;
        int     n = 0;

        UART2_SendString("Calibrating IMU, keep still...\r\n");
        for (int i = 0; i < CAL_SAMPLES; i++)
        {
            if (MPU6050_ReadAll(&ax, &ay, &az, &gx, &gy, &gz) == 0)
            {
                sgz += gz;
                sax += ax;
                say += ay;
                saz += az;
                if (gz < gmin) gmin = gz;
                if (gz > gmax) gmax = gz;
                n++;
            }
            else
            {
                I2C1_Recover();
            }
            delay_ms(IMU_PERIOD_MS);
        }

        /* --- Check the three conditions ---------------------------------- */
        uint8_t reads_ok = (n == CAL_SAMPLES);
        uint8_t still    = reads_ok && ((gmax - gmin) < CAL_MAX_PP);
        float   az_g     = (n > 0) ? ((float)saz / n) / ACCEL_LSB_PER_G : 0.0f;
        uint8_t flat     = reads_ok && (az_g > CAL_MIN_AZ_G);

        if (reads_ok && still && flat)
        {
            gz_off = (float)sgz / n;
            ax_off = (float)sax / n;
            ay_off = (float)say / n;
            break;
        }

        /* --- Report why, then retry --------------------------------------- */
        if (!reads_ok)
        {
            UART2_SendString("Calibration failed: only ");
            UART2_SendInt(n);
            UART2_SendString(" of ");
            UART2_SendInt(CAL_SAMPLES);
            UART2_SendString(" reads OK (check I2C wiring)\r\n");
        }
        else if (!still)
        {
            UART2_SendString("Calibration failed: moving (gz p-p ");
            UART2_SendInt(gmax - gmin);
            UART2_SendString("), keep still\r\n");
        }
        else
        {
            UART2_SendString("Calibration failed: not flat (az = ");
            UART2_SendFixed2(az_g);
            UART2_SendString(" g, Z must point up)\r\n");
        }
    }

    UART2_SendString("gz_off: ");    UART2_SendFixed2(gz_off);
    UART2_SendString("  ax_off: ");  UART2_SendInt((int32_t)ax_off);
    UART2_SendString("  ay_off: ");  UART2_SendInt((int32_t)ay_off);
    UART2_SendString("\r\n");
}

static void IMU_Task(uint32_t now)
{
    static uint8_t coll_n = 0;
    static uint8_t tip_n  = 0;
    int16_t ax, ay, az, gx, gy, gz;

    if (MPU6050_ReadAll(&ax, &ay, &az, &gx, &gy, &gz) != 0)
    {
        /* imu_last_ok unchanged: the next good sample integrates the gap */
        imu_read_errors++;
        imu_fail_streak++;
        I2C1_Recover();
        return;
    }
    imu_fail_streak = 0;

    /* --- Heading ---------------------------------------------------------- */
    float dt = (now - imu_last_ok) / 1000.0f;   // measured, not assumed
    imu_last_ok = now;
    heading += ((gz - gz_off) / GYRO_LSB_PER_DPS) * dt;

    /* --- Bias tracking while parked (zero-rate update) --------------------
     * IDLE, motors off for > 1 s and gyro quiet: the robot is not rotating,
     * so any gz left is bias. Low-pass it into gz_off (tau ~2.5 s at 200 Hz). */
    if (state == ST_IDLE && (now - motor_change_ms) > 1000 &&
        fabsf(gz - gz_off) < 200.0f)                    // < ~1.5 deg/s
    {
        gz_off += 0.002f * (gz - gz_off);
    }

    /* --- Accel baseline tracking while parked ----------------------------
     * Same conditions: whatever X/Y shows is tilt, not acceleration. Follow
     * it, so a robot put down differently after boot starts with axy ~0
     * (tau ~0.5 s at 200 Hz). Tipping is still caught by the raw az check. */
    if (state == ST_IDLE && (now - motor_change_ms) > 1000 &&
        fabsf(gz - gz_off) < 200.0f)
    {
        ax_off += 0.01f * (ax - ax_off);
        ay_off += 0.01f * (ay - ay_off);
    }

    /* --- Collision: horizontal spike, ignored right after motor changes --- */
    float dx = (ax - ax_off) / ACCEL_LSB_PER_G;
    float dy = (ay - ay_off) / ACCEL_LSB_PER_G;
    accel_xy_g = sqrtf(dx * dx + dy * dy);
    if (accel_xy_g > accel_xy_max_g)
        accel_xy_max_g = accel_xy_g;

    if (accel_xy_g > COLLISION_G && (now - motor_change_ms) > BLANK_MS)
    {
        if (++coll_n >= COLLISION_SAMPLES)
            collision_flag = 1;
    }
    else
    {
        coll_n = 0;
    }

    /* --- Tipped: gravity no longer on +Z ---------------------------------- */
    if ((az / ACCEL_LSB_PER_G) < TIP_AZ_G)
    {
        if (tip_n < 255) tip_n++;
        if (tip_n >= TIP_SAMPLES)
            tipped_flag = 1;
    }
    else
    {
        tip_n = 0;
    }
}

/* ===========================================================================
 *
 *  ULTRASONIC (HC-SR04)
 *  Non-blocking: TRIG pulse from the scheduler, ECHO edges timestamped in
 *  the EXTI1 ISR with TIM2 (1 us), distance computed in HCSR04_Process()
 *
 * ======================================================================== */

static void HCSR04_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /* PB0 TRIG: output, push-pull, low */
    GPIOB->BSRR    =  (1U << (0 + 16));
    GPIOB->MODER  &= ~(3U << (0*2));
    GPIOB->MODER  |=  (1U << (0*2));
    GPIOB->OTYPER &= ~(1U << 0);

    /* PB1 ECHO: input, no pull (5 V ECHO through the divider) */
    GPIOB->MODER  &= ~(3U << (1*2));
    GPIOB->PUPDR  &= ~(3U << (1*2));

    /* EXTI1 <- PB1, both edges */
    SYSCFG->EXTICR[0] &= ~(0xFU << 4);      // EXTI1 field = bits [7:4]
    SYSCFG->EXTICR[0] |=  (0x1U << 4);      // 1 = port B
    EXTI->RTSR |= (1U << 1);
    EXTI->FTSR |= (1U << 1);
    EXTI->PR    = (1U << 1);                // clear a stale pending bit
    EXTI->IMR  |= (1U << 1);

    NVIC_EnableIRQ(EXTI1_IRQn);
}

void EXTI1_IRQHandler(void)
{
    if (EXTI->PR & (1U << 1))
    {
        EXTI->PR = (1U << 1);               // rc_w1: write 1 to clear
        uint32_t t = TIM2->CNT;

        if (GPIOB->IDR & (1U << 1))         // rising edge: echo starts
        {
            echo_t_rise = t;
            echo_high   = 1;
        }
        else if (echo_high)                 // falling edge: echo ends
        {
            echo_width_us = t - echo_t_rise;
            echo_high     = 0;
            echo_new      = 1;
        }
    }
}

static void HCSR04_Trigger(void)
{
    /* Out of range, genuine modules end ECHO after ~38 ms, many clones only
     * after ~200 ms. Only a missed falling edge lasts longer: drop it then,
     * or it would block all further pings. */
    if (echo_high && (TIM2->CNT - echo_t_rise) > 300000U)
        echo_high = 0;

    if (echo_high)
        return;                             // previous echo still running

    GPIOB->BSRR = (1U << 0);                // TRIG high
    delay_us(12);                           // >= 10 us
    GPIOB->BSRR = (1U << (0 + 16));         // TRIG low
}

static void HCSR04_Process(uint32_t now)
{
    if (!echo_new)
        return;

    __disable_irq();
    uint32_t w = echo_width_us;
    echo_new = 0;
    __enable_irq();

    dist_cm = (w > US_MAX_ECHO_US) ? US_FAR_CM : (w / 58U);
    dist_ms = now;
}

/* ===========================================================================
 *
 *  MOTORS (L298N)
 *  TIM3 CH1 = ENA (PA6), CH2 = ENB (PA7); direction on PA8/PA9 and PB4/PB5
 *  Motor_Set(): signed duty, -PWM_MAX..PWM_MAX, + = forward, 0 = coast
 *
 * ======================================================================== */

static void Motor_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    /* Direction pins low first: both motors off */
    GPIOA->BSRR = (1U << (8 + 16)) | (1U << (9 + 16));
    GPIOB->BSRR = (1U << (4 + 16)) | (1U << (5 + 16));

    /* PA8, PA9 (IN1, IN2): output, push-pull */
    GPIOA->MODER  &= ~((3U << (8*2)) | (3U << (9*2)));
    GPIOA->MODER  |=  ((1U << (8*2)) | (1U << (9*2)));
    GPIOA->OTYPER &= ~((1U << 8) | (1U << 9));

    /* PB4, PB5 (IN3, IN4): output, push-pull, no pull
     * (PB4 is NJTRST at reset with a pull-up; SWD does not need it) */
    GPIOB->MODER  &= ~((3U << (4*2)) | (3U << (5*2)));
    GPIOB->MODER  |=  ((1U << (4*2)) | (1U << (5*2)));
    GPIOB->OTYPER &= ~((1U << 4) | (1U << 5));
    GPIOB->PUPDR  &= ~((3U << (4*2)) | (3U << (5*2)));

    /* PA6, PA7 (ENA, ENB): AF2 = TIM3_CH1 / TIM3_CH2, push-pull */
    GPIOA->MODER  &= ~((3U << (6*2)) | (3U << (7*2)));
    GPIOA->MODER  |=  ((2U << (6*2)) | (2U << (7*2)));
    GPIOA->OTYPER &= ~((1U << 6) | (1U << 7));
    GPIOA->AFR[0] &= ~((0xFU << (6*4)) | (0xFU << (7*4)));
    GPIOA->AFR[0] |=  ((2U   << (6*4)) | (2U   << (7*4)));

    /* TIM3: 16 MHz / 90 / 1000 = ~178 Hz PWM */
    TIM3->CR1   = 0;
    TIM3->PSC   = 89;
    TIM3->ARR   = PWM_MAX;
    TIM3->CCR1  = 0;
    TIM3->CCR2  = 0;
    TIM3->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE    // CH1 PWM mode 1
                | (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;   // CH2 PWM mode 1
    TIM3->CCER  = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM3->CR1   = TIM_CR1_ARPE;
    TIM3->EGR   = TIM_EGR_UG;           // load PSC/ARR/CCR preload now
    TIM3->CR1  |= TIM_CR1_CEN;
}

static int16_t clamp_duty(int16_t d)
{
    if (d >  PWM_MAX) return  PWM_MAX;
    if (d < -PWM_MAX) return -PWM_MAX;
    return d;
}

static void Motor_Set(int16_t left, int16_t right)
{
    left  = clamp_duty(left);
    right = clamp_duty(right);

    if (left != mot_left || right != mot_right)
        motor_change_ms = ms_ticks;     // start collision blanking
    mot_left  = left;
    mot_right = right;

    /* Motor A (left): IN1 = PA8, IN2 = PA9 */
    if (left > 0)       GPIOA->BSRR = (1U << 8)         | (1U << (9 + 16));
    else if (left < 0)  GPIOA->BSRR = (1U << (8 + 16))  | (1U << 9);
    else                GPIOA->BSRR = (1U << (8 + 16))  | (1U << (9 + 16));

    /* Motor B (right): IN3 = PB4, IN4 = PB5 */
    if (right > 0)      GPIOB->BSRR = (1U << 4)         | (1U << (5 + 16));
    else if (right < 0) GPIOB->BSRR = (1U << (4 + 16))  | (1U << 5);
    else                GPIOB->BSRR = (1U << (4 + 16))  | (1U << (5 + 16));

    TIM3->CCR1 = (uint32_t)(left  < 0 ? -left  : left);
    TIM3->CCR2 = (uint32_t)(right < 0 ? -right : right);
}

/* ===========================================================================
 *
 *  CONTROL (state machine, 50 Hz)
 *
 *  IDLE --B1--> SETTLE --clear--> FORWARD --obstacle--> SETTLE
 *                  |                                       |
 *                  +--blocked--> TURN --90 deg--> SETTLE <-+
 *  any running state --collision / tipped / i2c / no echo / timeout--> FAULT
 *  B1 in a running state -> IDLE, B1 in FAULT -> IDLE
 *
 * ======================================================================== */

static uint8_t is_running(void)
{
    return (state == ST_SETTLE) || (state == ST_FORWARD) || (state == ST_TURN);
}

static void enter_state(state_t s, uint32_t now)
{
    state    = s;
    state_ms = now;

    if (s == ST_TURN)
        turn_start = heading;
    else
        Motor_Set(0, 0);                // every other state starts stopped
}

static void enter_fault(fault_t f, uint32_t now)
{
    fault = f;
    enter_state(ST_FAULT, now);
}

static void Control_Task(uint32_t now)
{
    uint8_t btn   = Button_Pressed();
    uint8_t fresh = (now - dist_ms) < US_FRESH_MS;

    /* --- Button: start / stop / clear ------------------------------------- */
    if (btn)
    {
        if (state == ST_IDLE)
        {
            collision_flag = 0;         // ignore handling before the start
            tipped_flag    = 0;
            fault          = F_NONE;
            enter_state(ST_SETTLE, now);
        }
        else
        {
            enter_state(ST_IDLE, now);  // stop, or clear a fault
        }
        return;
    }

    /* --- Safety checks (running states only) ------------------------------ */
    if (is_running())
    {
        if (collision_flag)                     { enter_fault(F_COLLISION, now);  return; }
        if (tipped_flag)                        { enter_fault(F_TIPPED, now);     return; }
        if (imu_fail_streak >= IMU_FAIL_LIMIT)     { enter_fault(F_IMU, now);        return; }
        if ((now - dist_ms) > US_STALE_MS)      { enter_fault(F_ULTRASONIC, now); return; }
    }

    /* --- States ------------------------------------------------------------ */
    switch (state)
    {
        case ST_IDLE:
        case ST_FAULT:
            Motor_Set(0, 0);
            break;

        case ST_SETTLE:
            /* Wait SETTLE_MS, then decide on a distance measured while stopped */
            Motor_Set(0, 0);
            if ((now - state_ms) >= SETTLE_MS && fresh &&
                (int32_t)(dist_ms - state_ms) > (int32_t)(SETTLE_MS / 2))
            {
                if (dist_cm >= CLEAR_CM)
                    enter_state(ST_FORWARD, now);
                else
                    enter_state(ST_TURN, now);
            }
            break;

        case ST_FORWARD:
            Motor_Set(DUTY_FORWARD, DUTY_FORWARD);
            if (!fresh || dist_cm < OBSTACLE_CM)
                enter_state(ST_SETTLE, now);
            break;

        case ST_TURN:
            /* Left in place: left back, right forward (heading increases).
             * fabsf() makes the stop condition independent of wiring sign. */
            Motor_Set(-DUTY_TURN, DUTY_TURN);
            if (fabsf(heading - turn_start) >= TURN_DEG)
                enter_state(ST_SETTLE, now);
            else if ((now - state_ms) > TURN_TIMEOUT_MS)
                enter_fault(F_TURN_TIMEOUT, now);
            break;
    }
}

/* ===========================================================================
 *
 *  USER BUTTON (B1 = PC13, active low, external pull-up on the Nucleo)
 *  Polled every CTRL_PERIOD_MS: 20 ms sampling debounces the contact
 *
 * ======================================================================== */

static void Button_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC->MODER &= ~(3U << (13*2));    // input
    GPIOC->PUPDR &= ~(3U << (13*2));    // no pull (board has one)
}

/* Returns 1 once per press (high -> low transition) */
static uint8_t Button_Pressed(void)
{
    static uint8_t prev = 1;
    uint8_t cur = (GPIOC->IDR >> 13) & 1U;
    uint8_t pressed = (prev == 1U) && (cur == 0U);
    prev = cur;
    return pressed;
}

/* ===========================================================================
 *
 *  DEBUG PRINT (10 Hz)
 *  st:FWD  d:123 cm  hdg:-12.34  axy:0.05 g  axy_max:0.40 g  imu_err:0  [fault]
 *
 * ======================================================================== */

static void Print_Task(void)
{
    UART2_SendString("st:");      UART2_SendString(state_name[state]);
    UART2_SendString("  d:");     UART2_SendInt((int32_t)dist_cm);
    UART2_SendString(" cm");
    if (imu_fail_streak > 0)
    {
        /* No fresh sample: hdg/axy would be stale, show the bus lines instead
         * (IDR reads the pin level even in AF mode) */
        UART2_SendString("  IMU DEAD  SCL=");
        UART2_SendInt((GPIOB->IDR >> 6) & 1U);
        UART2_SendString(" SDA=");
        UART2_SendInt((GPIOB->IDR >> 7) & 1U);
    }
    else
    {
        UART2_SendString("  hdg:");      UART2_SendFixed2(heading);
        UART2_SendString("  axy:");      UART2_SendFixed2(accel_xy_g);
        UART2_SendString(" g  axy_max:"); UART2_SendFixed2(accel_xy_max_g);
        UART2_SendString(" g");
    }
    UART2_SendString("  imu_err:"); UART2_SendInt((int32_t)imu_read_errors);
    if (state == ST_FAULT)
    {
        UART2_SendString("  [");
        UART2_SendString(fault_name[fault]);
        UART2_SendString("]");
    }
    UART2_SendString("\r\n");

    accel_xy_max_g = 0.0f;              // max is per print interval
}

/* ===========================================================================
 *
 *  I2C1 LOW-LEVEL
 *  Bus clear, init, START/STOP, address and data phases.
 *  All transfer functions return 0 = OK, -1 = NACK or timeout.
 *
 * ======================================================================== */

/* Short busy wait for bit-banging, roughly 10 us at 16 MHz (not critical) */
static void i2c_bb_delay(void)
{
    for (volatile int d = 0; d < 15; d++);
}

/* Bus clear (NXP UM10204 "Bus clear"): if the STM32 reset mid-transfer, the
 * MPU6050 is still somewhere inside a byte. Always send 9 SCL pulses with SDA
 * released: this finishes any byte (8 data bits + ACK slot), the released SDA
 * in the ACK slot is a NACK, so the MPU stops sending. Then send a STOP to
 * reset its state machine. Idle slaves ignore clocks without a START.
 * Pins are left as GPIO; I2C1_Init() switches them back to AF4. */
static void I2C1_BusClear(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    GPIOB->OTYPER |=  (1U << 6) | (1U << 7);            // open-drain
    GPIOB->PUPDR  &= ~((3U << (6*2)) | (3U << (7*2)));
    GPIOB->PUPDR  |=  ((1U << (6*2)) | (1U << (7*2)));  // pull-up
    GPIOB->BSRR    =  (1U << 6) | (1U << 7);            // release both high
    GPIOB->MODER  &= ~((3U << (6*2)) | (3U << (7*2)));
    GPIOB->MODER  |=  ((1U << (6*2)) | (1U << (7*2)));  // GPIO output
    i2c_bb_delay();

    /* Always 9 clocks: SDA high now may just be a '1' bit mid-byte */
    for (int i = 0; i < 9; i++)
    {
        GPIOB->BSRR = (1U << (6 + 16));   // SCL low
        i2c_bb_delay();
        GPIOB->BSRR = (1U << 6);          // SCL high
        i2c_bb_delay();
    }

    /* STOP condition: SDA low -> high while SCL high */
    GPIOB->BSRR = (1U << (6 + 16));       // SCL low
    i2c_bb_delay();
    GPIOB->BSRR = (1U << (7 + 16));       // SDA low
    i2c_bb_delay();
    GPIOB->BSRR = (1U << 6);              // SCL high
    i2c_bb_delay();
    GPIOB->BSRR = (1U << 7);              // SDA high
    i2c_bb_delay();
}

static void I2C1_Init(void)
{
    I2C1_BusClear();

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /* PB6 SCL, PB7 SDA: AF4, open-drain, high speed, pull-up */
    GPIOB->MODER   &= ~((3U << (6*2)) | (3U << (7*2)));
    GPIOB->MODER   |=  ((2U << (6*2)) | (2U << (7*2)));
    GPIOB->OTYPER  |=  (1U << 6) | (1U << 7);
    GPIOB->OSPEEDR |=  (3U << (6*2)) | (3U << (7*2));
    GPIOB->PUPDR   &= ~((3U << (6*2)) | (3U << (7*2)));
    GPIOB->PUPDR   |=  ((1U << (6*2)) | (1U << (7*2)));
    GPIOB->AFR[0]  &= ~((0xFU << (6*4)) | (0xFU << (7*4)));
    GPIOB->AFR[0]  |=  ((4U   << (6*4)) | (4U   << (7*4)));

    /* Software reset clears a stale BUSY flag (also covers F4 errata
     * where the analog filter can latch BUSY) */
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;                      // out of reset, PE = 0

    I2C1->CR2   = 16;                   // APB1 = 16 MHz
    I2C1->CCR   = 80;                   // 100 kHz: 16 MHz / (2 * 100 kHz)
    I2C1->TRISE = 17;                   // 1000 ns / 62.5 ns + 1

    I2C1->CR1 |= I2C_CR1_PE;
}

/* After a timeout or NACK: bus clear + peripheral reset + MPU wake-up */
static void I2C1_Recover(void)
{
    I2C1_Init();                        // includes bus clear and SWRST
    MPU6050_Init();
}

/* Wait for an SR1 flag; fail on NACK, arbitration lost, bus error or timeout */
static int I2C1_WaitSR1(uint32_t flag)
{
    const uint32_t err = I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR;
    uint32_t t0 = ms_ticks;

    while (!(I2C1->SR1 & flag))
    {
        if (I2C1->SR1 & err)                    // NACK, arbitration lost or bus error
        {
            I2C1->SR1 = ~err;                   // rc_w0: clears only these flags
            I2C1_Stop();                        // release the bus
            return -1;
        }
        if ((ms_ticks - t0) > I2C_TIMEOUT_MS)   // stuck bus or dead slave
        {
            return -1;
        }
    }
    return 0;
}

static int I2C1_Start(void)
{
    /* Wait until a previous STOP has been generated */
    uint32_t t0 = ms_ticks;
    while (I2C1->CR1 & I2C_CR1_STOP)
    {
        if ((ms_ticks - t0) > I2C_TIMEOUT_MS)
            return -1;
    }

    I2C1->CR1 |= I2C_CR1_START;
    return I2C1_WaitSR1(I2C_SR1_SB);
}

static void I2C1_Stop(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
}

static int I2C1_WriteAddr(uint8_t addr)
{
    I2C1->DR = addr;
    if (I2C1_WaitSR1(I2C_SR1_ADDR) != 0)
        return -1;
    (void)I2C1->SR1;
    (void)I2C1->SR2;                    // clear ADDR: read SR1 then SR2
    return 0;
}

static int I2C1_WriteData(uint8_t data)
{
    if (I2C1_WaitSR1(I2C_SR1_TXE) != 0)
        return -1;
    I2C1->DR = data;
    return I2C1_WaitSR1(I2C_SR1_BTF);
}

/* ===========================================================================
 *
 *  MPU6050 DRIVER
 *  Register write, single read, burst read (RM0390 sequences)
 *
 * ======================================================================== */

/* Full configuration, written explicitly (also after every bus clear,
 * because an interrupted write can leave a register corrupted) */
static int MPU6050_Init(void)
{
    if (MPU6050_WriteReg(PWR_MGMT_1,   0x01) != 0) return -1;   // wake, PLL with X gyro
    if (MPU6050_WriteReg(SMPLRT_DIV,   4)    != 0) return -1;   // 1 kHz / (1 + 4) = 200 Hz
    if (MPU6050_WriteReg(CONFIG,       0x03) != 0) return -1;   // DLPF ~44 Hz (motor vibration)
    if (MPU6050_WriteReg(GYRO_CONFIG,  0x00) != 0) return -1;   // +-250 dps, 131 LSB/dps
    if (MPU6050_WriteReg(ACCEL_CONFIG, 0x00) != 0) return -1;   // +-2 g, 16384 LSB/g
    return 0;
}

static int MPU6050_WriteReg(uint8_t reg, uint8_t value)
{
    if (I2C1_Start() != 0)                       goto err;
    if (I2C1_WriteAddr(MPU6050_ADDR_WRITE) != 0) goto err;
    if (I2C1_WriteData(reg) != 0)                goto err;
    if (I2C1_WriteData(value) != 0)              goto err;
    I2C1_Stop();
    return 0;
err:
    I2C1_Stop();
    return -1;
}

/* Single-byte read (RM0390 sequence for N = 1):
 * ACK must be cleared BEFORE ADDR is cleared, STOP set right after. */
static int MPU6050_ReadReg(uint8_t reg, uint8_t *value)
{
    if (I2C1_Start() != 0)                       goto err;
    if (I2C1_WriteAddr(MPU6050_ADDR_WRITE) != 0) goto err;
    if (I2C1_WriteData(reg) != 0)                goto err;

    if (I2C1_Start() != 0)                       goto err;   // repeated start
    I2C1->CR1 &= ~I2C_CR1_ACK;                               // NACK the only byte
    if (I2C1_WriteAddr(MPU6050_ADDR_READ) != 0)  goto err;   // clears ADDR
    I2C1_Stop();

    if (I2C1_WaitSR1(I2C_SR1_RXNE) != 0)         goto err;
    *value = (uint8_t)I2C1->DR;
    return 0;
err:
    I2C1_Stop();
    return -1;
}

/* Burst read, n >= 3 (RM0390 sequence for N > 2):
 * read with ACK until 3 bytes remain, then use BTF to place NACK and
 * STOP exactly on the last byte. The register pointer auto-increments. */
static int MPU6050_ReadBurst(uint8_t reg, uint8_t *buf, uint8_t n)
{
    uint8_t i = 0;

    if (n < 3) return -1;

    if (I2C1_Start() != 0)                       goto err;
    if (I2C1_WriteAddr(MPU6050_ADDR_WRITE) != 0) goto err;
    if (I2C1_WriteData(reg) != 0)                goto err;

    I2C1->CR1 |= I2C_CR1_ACK;                                // ACK data bytes
    if (I2C1_Start() != 0)                       goto err;   // repeated start
    if (I2C1_WriteAddr(MPU6050_ADDR_READ) != 0)  goto err;   // reception starts

    while ((n - i) > 3)
    {
        if (I2C1_WaitSR1(I2C_SR1_RXNE) != 0)     goto err;
        buf[i++] = (uint8_t)I2C1->DR;
    }

    /* 3 bytes left: N-2 in DR, N-1 in shift register when BTF sets */
    if (I2C1_WaitSR1(I2C_SR1_BTF) != 0)          goto err;
    I2C1->CR1 &= ~I2C_CR1_ACK;                               // NACK the last byte
    buf[i++] = (uint8_t)I2C1->DR;                            // N-2

    /* N-1 in DR, N in shift register */
    if (I2C1_WaitSR1(I2C_SR1_BTF) != 0)          goto err;
    I2C1_Stop();
    buf[i++] = (uint8_t)I2C1->DR;                            // N-1

    if (I2C1_WaitSR1(I2C_SR1_RXNE) != 0)         goto err;
    buf[i++] = (uint8_t)I2C1->DR;                            // N
    return 0;
err:
    I2C1->CR1 &= ~I2C_CR1_ACK;
    I2C1_Stop();
    return -1;
}

/* One 14-byte burst from 0x3B: accel, temp and gyro from the SAME sample */
static int MPU6050_ReadAll(int16_t *ax, int16_t *ay, int16_t *az,
                           int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t b[14];

    if (MPU6050_ReadBurst(ACCEL_XOUT_H, b, 14) != 0)
        return -1;

    *ax = (int16_t)((b[0]  << 8) | b[1]);
    *ay = (int16_t)((b[2]  << 8) | b[3]);
    *az = (int16_t)((b[4]  << 8) | b[5]);
    /* b[6..7] = TEMP_OUT: degC = raw / 340 + 36.53 */
    *gx = (int16_t)((b[8]  << 8) | b[9]);
    *gy = (int16_t)((b[10] << 8) | b[11]);
    *gz = (int16_t)((b[12] << 8) | b[13]);
    return 0;
}

/* ===========================================================================
 *
 *  UART2 OUTPUT
 *  Blocking TX: char, string, int, hex, fixed-point
 *
 * ======================================================================== */

static void UART2_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA2 TX, PA3 RX: AF7 */
    GPIOA->MODER  &= ~((3U << (2*2)) | (3U << (3*2)));
    GPIOA->MODER  |=  ((2U << (2*2)) | (2U << (3*2)));
    GPIOA->AFR[0] &= ~((0xFU << (2*4)) | (0xFU << (3*4)));
    GPIOA->AFR[0] |=  ((7U   << (2*4)) | (7U   << (3*4)));

    USART2->CR1 = 0;                    // disable while configuring

    /* 115200 baud @ 16 MHz, oversampling 16:
     * USARTDIV = 16e6 / (16 * 115200) = 8.68 -> mantissa 8, fraction 11/16
     * BRR = (8 << 4) | 11 = 0x8B  (actual 115108 baud, -0.08 %) */
    USART2->BRR = 0x8B;
    USART2->CR2 = 0;                    // 1 stop bit
    USART2->CR3 = 0;                    // no flow control
    USART2->CR1 = USART_CR1_TE          // transmitter on, 8N1 (M = 0)
                | USART_CR1_UE;         // USART on
}

static void UART2_SendChar(char c)
{
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = (uint8_t)c;
}

static void UART2_SendString(const char *str)
{
    while (*str)
        UART2_SendChar(*str++);
}

static void UART2_SendInt(int32_t num)
{
    char     buf[12];
    int      i = 0;
    uint32_t u;

    if (num < 0)
    {
        UART2_SendChar('-');
        u = (uint32_t)(-(num + 1)) + 1; // safe for INT32_MIN
    }
    else
    {
        u = (uint32_t)num;
    }

    if (u == 0) buf[i++] = '0';
    while (u > 0)
    {
        buf[i++] = (char)('0' + (u % 10));
        u /= 10;
    }
    while (i > 0)
        UART2_SendChar(buf[--i]);
}

static void UART2_SendHex8(uint8_t v)
{
    const char hex[] = "0123456789ABCDEF";
    UART2_SendChar(hex[v >> 4]);
    UART2_SendChar(hex[v & 0x0F]);
}

/* Prints a float with 2 decimals, correct sign and leading zero
 * (e.g. -0.50, 1.07) */
static void UART2_SendFixed2(float v)
{
    int32_t x = (int32_t)(v * 100.0f + (v >= 0.0f ? 0.5f : -0.5f));
    if (x < 0)
    {
        UART2_SendChar('-');
        x = -x;
    }
    UART2_SendInt(x / 100);
    UART2_SendChar('.');
    int32_t f = x % 100;
    if (f < 10) UART2_SendChar('0');
    UART2_SendInt(f);
}

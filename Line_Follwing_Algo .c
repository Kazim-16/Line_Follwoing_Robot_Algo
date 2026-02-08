#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

/* =========================================================
======================== GYROSCOPE =========================
========================================================= */

/* ================= I2C ================= */
#define I2C_SDA             21
#define I2C_SCL             22
#define I2C_PORT            I2C_NUM_0
#define I2C_FREQ_HZ         100000

/* ================= MPU6050 ================= */
#define MPU6050_ADDR        0x68
#define MPU_PWR_MGMT_1      0x6B
#define MPU_GYRO_ZOUT_H     0x47

#define GYRO_SENS_250DPS    131.0f
#define CALIB_SAMPLES       1000
#define GYRO_DEADZONE_DPS   1.0f

static float gyro_z_bias = 0.0f;
static float yaw = 0.0f;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
typedef enum {
    DIR_FORWARD = 0,   // 0°
    DIR_RIGHT   = 90,
    DIR_BACK    = 180,
    DIR_LEFT    = 270
} heading_t;

volatile heading_t logical_heading = DIR_FORWARD;

/* ================= I2C LOW LEVEL ================= */
static esp_err_t i2c_write_byte(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_bytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void mpu6050_init(void)
{
    ESP_ERROR_CHECK(i2c_write_byte(MPU_PWR_MGMT_1, 0x00));
    vTaskDelay(pdMS_TO_TICKS(100));
}

static float read_gyro_z_dps(void)
{
    uint8_t raw[2];
    i2c_read_bytes(MPU_GYRO_ZOUT_H, raw, 2);
    int16_t gz = (raw[0] << 8) | raw[1];
    return (float)gz / GYRO_SENS_250DPS;
}

static void calibrate_gyro_z(void)
{
    float sum = 0.0f;
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        sum += read_gyro_z_dps();
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    gyro_z_bias = sum / CALIB_SAMPLES;
}

typedef enum {
    LAST_NONE = 0,
    LAST_LEFT,
    LAST_RIGHT
} last_turn_t;

volatile last_turn_t last_turn = LAST_NONE;


/* =========================================================
=========================== BLE ============================
========================================================= */

#define BT_UART_NUM UART_NUM_2
#define BT_TX 17
#define BT_RX 16
#define BT_BAUD 38400
#define RX_BUF_SIZE 256

void bt_send(const char *msg)
{
    uart_write_bytes(BT_UART_NUM, msg, strlen(msg));
}

/* ================= GYRO TASK (CORE 0) ================= */
void GyroTask(void *arg)
{
    mpu6050_init();
    calibrate_gyro_z();

    const float dt = 0.01f;
    char msg[64];

    while (1)
    {
        float gz = read_gyro_z_dps() - gyro_z_bias;
        gz = -gz;

        if (fabsf(gz) < GYRO_DEADZONE_DPS)
            gz = 0.0f;

        portENTER_CRITICAL(&mux);
        yaw += gz * dt;
        if (yaw >= 360.0f) yaw -= 360.0f;
        else if (yaw < 0.0f) yaw += 360.0f;
        portEXIT_CRITICAL(&mux);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* =========================================================
======================= LINE FOLLOWER ======================
========================================================= */

/* ---------------- MOTORS ---------------- */
#define R_F   23
#define R_B   5
#define L_F   27
#define L_B   4
#define R_PWM 19
#define L_PWM 18

/* ---------------- ENCODERS ---------------- */
#define ENC_L 13
#define ENC_R 14

volatile int left_ticks = 0;
volatile int right_ticks = 0;

/* ---------------- SENSORS ---------------- */
#define NUM_SENSORS 8
#define S0 ADC_CHANNEL_9
#define S1 ADC_CHANNEL_8
#define S2 ADC_CHANNEL_5
#define S3 ADC_CHANNEL_4
#define S4 ADC_CHANNEL_7
#define S5 ADC_CHANNEL_6
#define S6 ADC_CHANNEL_3
#define S7 ADC_CHANNEL_0

int raw[NUM_SENSORS];
int cal_min[NUM_SENSORS];
int cal_max[NUM_SENSORS];
int norm[NUM_SENSORS];

#define ALL_WHITE_PATTERN  0x00

#define TURN_LEFT_OFFSET   5.0f   // degrees

static uint16_t left_cnt = 0;
static uint16_t right_cnt = 0;
static uint16_t t_cnt = 0;
static uint16_t post_cnt = 0;
static uint16_t white_cnt = 0;
static uint8_t prev_pattern = 0;
static bool pid_hold = false;

volatile bool do_calibration = false;
volatile bool robot_run = false;
volatile bool junction_detected = false;

volatile float Kp = 400;
volatile int BASE_PWM = 700;

#define BLACK_TH 600

const float weights[NUM_SENSORS] = {
    -3.5, -2.5, -1.5, -0.5,
     0.5,  1.5,  2.5,  3.5
};

/* ================= TURN CONTROL ================= */
typedef enum {
    TURN_NONE = 0,
    TURN_LEFT,
    TURN_RIGHT
} turn_state_t;


volatile turn_state_t turn_state = TURN_NONE;
volatile float turn_target_yaw = 0.0f;

static inline float angle_error_360(float target, float current)
{
    float err = target - current;

    if (err > 180.0f)  err -= 360.0f;
    if (err < -180.0f) err += 360.0f;

    return err;   // range: -180 .. +180
}


/* ================= MOTOR ================= */
void motor_set(int left, int right)
{
    /* LEFT motor direction */
    if (left >= 0) {
        gpio_set_level(L_F, 1);
        gpio_set_level(L_B, 0);
    } else {
        gpio_set_level(L_F, 0);
        gpio_set_level(L_B, 1);
        left = -left;
    }

    /* RIGHT motor direction */
    if (right >= 0) {
        gpio_set_level(R_F, 1);
        gpio_set_level(R_B, 0);
    } else {
        gpio_set_level(R_F, 0);
        gpio_set_level(R_B, 1);
        right = -right;
    }

    if (left > 1023) left = 1023;
    if (right > 1023) right = 1023;

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, left);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, right);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
}

static inline float get_yaw_safe(void)
{
    float y;
    portENTER_CRITICAL(&mux);
    y = yaw;
    portEXIT_CRITICAL(&mux);
    return y;
}

void handle_left_turn(void)
{
    if (turn_state != TURN_LEFT)
        return;

    static uint8_t dbg_div = 0;   // debug rate divider

    float curr_yaw = get_yaw_safe();   // 0–360
    float err = angle_error_360(turn_target_yaw, curr_yaw);

    const int TURN_PWM = 800;

    /* -------- STOP CONDITION -------- */
    if (fabsf(err) < 3.0f)
    {
        motor_set(-BASE_PWM, -BASE_PWM);
        vTaskDelay(pdMS_TO_TICKS(530));
        motor_set(0, 0);
        bt_send("TURN_LEFT DONE\r\n");

        turn_state = TURN_NONE;
        junction_detected = false;
        pid_hold = false;
        robot_run = true;

        return;
    }

    /* -------- ROTATE LEFT -------- */
    motor_set(-TURN_PWM, TURN_PWM);
}

void handle_right_turn(void)
{
    if (turn_state != TURN_RIGHT)
        return;

    static uint8_t dbg_div = 0;

    float curr_yaw = get_yaw_safe();   // 0–360
    float err = angle_error_360(turn_target_yaw, curr_yaw);

    const int TURN_PWM = 800;

    /* -------- STOP CONDITION -------- */
    if (fabsf(err) < 3.0f)
    {
        motor_set(-BASE_PWM, -BASE_PWM);
        vTaskDelay(pdMS_TO_TICKS(530));
        motor_set(0, 0);
        bt_send("TURN_RIGHT DONE\r\n");

        turn_state = TURN_NONE;
        junction_detected = false;
        pid_hold = false;
        robot_run = true;
        return;
    }

    /* -------- ROTATE RIGHT -------- */
    motor_set(TURN_PWM, -TURN_PWM);
}



/* ================= LINE POSITION ================= */
float line_position(void)
{
    float sum = 0, weighted = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        sum += norm[i];
        weighted += norm[i] * weights[i];
    }
    if (sum == 0) return 0;
    return weighted / sum;
}

/* ================= SENSOR PATTERN ================= */
uint8_t sensor_pattern(void)
{
    uint8_t p = 0;
    for (int i = 0; i < NUM_SENSORS; i++)
        if (norm[i] > BLACK_TH)
            p |= (1 << (7 - i));
    return p;
}

/* ================= STRING PATTERN ================= */
static void pattern_to_str(uint8_t p, char *out)
{
    for (int i = 0; i < 8; i++)
        out[i] = (p & (1 << (7 - i))) ? '1' : '0';
    out[8] = '\0';
}

#define DIR_TOL 20
static inline bool yaw_in_valid_direction(float y)
{
    const char *dir = NULL;

    if (y >= 360 - DIR_TOL || y <= DIR_TOL)
        return true;
    else if (y >= 90 - DIR_TOL && y <= 90 + DIR_TOL)
        return true;
    else if (y >= 180 - DIR_TOL && y <= 180 + DIR_TOL)
        return true;
    else if (y >= 270 - DIR_TOL && y <= 270 + DIR_TOL)
        return true;
    else
        return false;   //  DO NOT SEND INVALID
}

static inline bool is_false_t_pattern(uint8_t p)
{
    switch (p)
    {
        case 0b11110111:
        case 0b11101111:
        case 0b11100111:
        case 0b11000111:
        case 0b01110011:
        case 0b11000011:
        case 0b10000011:
        case 0b11000001:
        case 0b10000001:
            return true;

        default:
            return false;
    }
}


/* ================= JUNCTION DETECTION ================= */
void detect_lr_t_junction(void)
{
    float curr_yaw = get_yaw_safe();
    /* ---------- HEADING CHECK ---------- */
    if (!yaw_in_valid_direction(curr_yaw))
    {
        /* Allow PID to continue */
        portENTER_CRITICAL(&mux);
        pid_hold = false;
        portEXIT_CRITICAL(&mux);

        /* Reset all junction counters */
        t_cnt = 0;
        left_cnt = 0;
        right_cnt = 0;
        white_cnt = 0;
        post_cnt = 0;

        prev_pattern = sensor_pattern();
        return;
    }

    uint8_t curr_pattern = sensor_pattern();
    char b[9], msg[64];
        pattern_to_str(curr_pattern, b);
        sprintf(msg, " pattern:%s \r\n", b);
        bt_send(msg);

    /* ================= T POSSIBLE ================= */
    if (curr_pattern == 0b11111111)
    {
        post_cnt = 0;
        left_cnt = 0;
        right_cnt = 0;
        portENTER_CRITICAL(&mux);
        pid_hold = true;
        t_cnt++;
        portEXIT_CRITICAL(&mux);

    }

    /* ================= LEFT POSSIBLE ================= */
    else if (curr_pattern == 0b11111000 || curr_pattern == 0b11111100)
    {
        post_cnt = 0;

        portENTER_CRITICAL(&mux);
        pid_hold = true;
        t_cnt = 0;
        right_cnt = 0;
        if ((prev_pattern == 0b11111000 || prev_pattern == 0b11111100) &&
            (curr_pattern == 0b11111000 || curr_pattern == 0b11111100))
            left_cnt++;
        else
            left_cnt = 1;
        portEXIT_CRITICAL(&mux);
    }

    /* ================= RIGHT POSSIBLE ================= */
    else if (curr_pattern == 0b00011111 || curr_pattern == 0b00111111)
    {
        post_cnt = 0;

        portENTER_CRITICAL(&mux);
        pid_hold = true;
        t_cnt = 0;
        left_cnt = 0;
        if ((prev_pattern == 0b00011111 || prev_pattern == 0b00111111) &&
            (curr_pattern == 0b00011111 || curr_pattern == 0b00111111))
            right_cnt++;
        else
            right_cnt = 1;
        portEXIT_CRITICAL(&mux);

    }

    /* ================= DECISION REGION ================= */
    else
    {
        portENTER_CRITICAL(&mux);
        pid_hold = false;
        portEXIT_CRITICAL(&mux);

        post_cnt++;

        if (post_cnt <= 20)
        {
            if (curr_pattern == ALL_WHITE_PATTERN)
                white_cnt++;
            else
                white_cnt = 0;

            /* ---------- T CONFIRM OR RESET USING WHITE COUNT ---------- */
            if (t_cnt > 10)
            {
                /* ===== FALSE T (CIRCLE / DIAMOND PATTERNS) ===== */
                if (is_false_t_pattern(curr_pattern))
                {
                    bt_send("FALSE T → PATTERN REJECT\r\n");

                    portENTER_CRITICAL(&mux);
                    pid_hold = false;
                    portEXIT_CRITICAL(&mux);

                    t_cnt = 0;
                    white_cnt = 0;
                    post_cnt = 0;
                    return;
                }

                /* ===== POSSIBLE TRUE T → CHECK ALL WHITE ===== */
                if (curr_pattern == ALL_WHITE_PATTERN)
                    white_cnt++;
                else
                    white_cnt = 0;

                if (white_cnt >= 10)
                {
                    char msg[96];
                    sprintf(msg,
                            "T CONFIRMED | YAW:%.2f\r\n",
                            curr_yaw);
                    bt_send(msg);

                    /* move into junction */
                    motor_set(BASE_PWM, BASE_PWM);
                    vTaskDelay(pdMS_TO_TICKS(370));

                    float curr_yaw = get_yaw_safe();   // 0–360

                    if (last_turn == LAST_LEFT)
                    {
                        // last was LEFT → now go RIGHT
                        turn_target_yaw = fmodf(curr_yaw + 85.0f, 360.0f);
                        turn_state = TURN_RIGHT;
                        last_turn = LAST_RIGHT;
                    }
                    else
                    {
                        // last was RIGHT or NONE → now go LEFT
                        turn_target_yaw = fmodf(curr_yaw + 285.0f, 360.0f);
                        turn_state = TURN_LEFT;
                        last_turn = LAST_LEFT;
                    }

                    junction_detected = true;
                    robot_run = false;
                    pid_hold = true;

                    t_cnt = 0;
                    white_cnt = 0;
                    post_cnt = 0;
                    return;
                }
            }

            /* ---------- LEFT CONFIRM ---------- */
            if (left_cnt > 10 && white_cnt >= 10)
            {
                bt_send("PURE LEFT JUNCTION\r\n");

                /* ---- STEP 1: MOVE INTO JUNCTION ---- */
                motor_set(BASE_PWM, BASE_PWM);
                vTaskDelay(pdMS_TO_TICKS(370));   // tune this
                float curr_yaw = get_yaw_safe(); 
                /* ---- STEP 2: SET TURN ---- */
                turn_target_yaw = fmodf(curr_yaw + 285.0f, 360.0f);
                turn_state = TURN_LEFT;
                /*  update memory */
                last_turn = LAST_LEFT;
                junction_detected = true;
                robot_run = false;
                pid_hold = true;
                left_cnt = 0;
                white_cnt = 0;
                post_cnt = 0;
                return;
            }

            /* ---------- RIGHT CONFIRM ---------- */
            if (right_cnt > 10 && white_cnt >= 10)
            {
                bt_send("PURE RIGHT JUNCTION\r\n");

                /* ---- STEP 1: MOVE INTO JUNCTION ---- */
                motor_set(BASE_PWM, BASE_PWM);
                vTaskDelay(pdMS_TO_TICKS(370));   // same as left

                float curr_yaw = get_yaw_safe(); 

                /* ---- STEP 2: SET TURN ---- */
                turn_target_yaw = fmodf(curr_yaw + 85.0f, 360.0f);
                turn_state = TURN_RIGHT;
                /*  update memory */
                last_turn = LAST_RIGHT;
                junction_detected = true;
                robot_run = false;
                pid_hold = true;

                right_cnt = 0;
                white_cnt = 0;
                post_cnt = 0;
                return;
            }

        }
        else
        {
            /* ---------- TIMEOUT ---------- */
            t_cnt = 0;
            left_cnt = 0;
            right_cnt = 0;
            white_cnt = 0;
            post_cnt = 0;
        }
    }

    prev_pattern = curr_pattern;
}



/* ================= COMMAND PARSER ================= */
void parse_line(char *line)
{
    for (int i = 0; line[i]; i++)
        if (line[i] == '\r' || line[i] == '\n') line[i] = 0;

    if (strcasecmp(line, "RUN") == 0) {

        junction_detected = false;
        left_cnt = 0;
        pid_hold = false;
        prev_pattern = sensor_pattern();  //  sync

        robot_run = true;
        bt_send("RUN | PID RESUMED\r\n");
        return;
    }

    if (strcasecmp(line, "STOP") == 0) {
        robot_run = false;
        motor_set(0, 0);
        bt_send("STOP\r\n");
        return;
    }

    if (strcasecmp(line, "CAL") == 0) {
        do_calibration = true;
        motor_set(0, 0);
        bt_send("CAL | SEND BLACK\r\n");
        return;
    }

    if (do_calibration && strcasecmp(line, "BLACK") == 0) {
        for (int i = 0; i < NUM_SENSORS; i++) cal_max[i] = raw[i];
        bt_send("BLACK OK | SEND WHITE\r\n");
        return;
    }

    if (do_calibration && strcasecmp(line, "WHITE") == 0) {
        for (int i = 0; i < NUM_SENSORS; i++) cal_min[i] = raw[i];
        do_calibration = false;
        bt_send("CAL DONE\r\n");
        return;
    }

    bt_send("ERR\r\n");
}

static void send_status_ble(void)
{
    static uint32_t last_tick = 0;
    uint32_t now = xTaskGetTickCount();

    /* send every 200 ms (5 Hz) */
    if ((now - last_tick) < pdMS_TO_TICKS(200))
        return;

    last_tick = now;

    char msg[96];
    sprintf(msg,
            "STAT | RUN:%d | JUNC:%d | PID:%d\r\n",
            robot_run ? 1 : 0,
            junction_detected ? 1 : 0,
            pid_hold ? 1 : 0);

    bt_send(msg);
}


/* ================= SENSOR TASK ================= */
void SensorTask(void *arg)
{
    adc_oneshot_unit_handle_t adc1, adc2;

    adc_oneshot_new_unit(&(adc_oneshot_unit_init_cfg_t){.unit_id = ADC_UNIT_1}, &adc1);
    adc_oneshot_new_unit(&(adc_oneshot_unit_init_cfg_t){.unit_id = ADC_UNIT_2}, &adc2);

    adc_oneshot_chan_cfg_t cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12
    };

    adc_oneshot_config_channel(adc2, S0, &cfg);
    adc_oneshot_config_channel(adc2, S1, &cfg);
    adc_oneshot_config_channel(adc1, S2, &cfg);
    adc_oneshot_config_channel(adc1, S3, &cfg);
    adc_oneshot_config_channel(adc1, S4, &cfg);
    adc_oneshot_config_channel(adc1, S5, &cfg);
    adc_oneshot_config_channel(adc1, S6, &cfg);
    adc_oneshot_config_channel(adc1, S7, &cfg);

    while (1) {

        if (turn_state == TURN_LEFT)
        {
            handle_left_turn();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (turn_state == TURN_RIGHT)
        {
            handle_right_turn();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        adc_oneshot_read(adc2, S0, &raw[0]);
        adc_oneshot_read(adc2, S1, &raw[1]);
        adc_oneshot_read(adc1, S2, &raw[2]);
        adc_oneshot_read(adc1, S3, &raw[3]);
        adc_oneshot_read(adc1, S4, &raw[4]);
        adc_oneshot_read(adc1, S5, &raw[5]);
        adc_oneshot_read(adc1, S6, &raw[6]);
        adc_oneshot_read(adc1, S7, &raw[7]);

        for (int i = 0; i < NUM_SENSORS; i++) {
            if (cal_max[i] > cal_min[i]) {
                int v = (raw[i] - cal_min[i]) * 1000 /
                        (cal_max[i] - cal_min[i]);
                if (v < 0) v = 0;
                if (v > 1000) v = 1000;
                norm[i] = v;
            }
        }

        if (robot_run && !do_calibration && !junction_detected) {
            
            detect_lr_t_junction();
            if (!junction_detected) {
                if (pid_hold)
                    motor_set(BASE_PWM, BASE_PWM);
                else {
                    float err = line_position();
                    motor_set(BASE_PWM + Kp * err,
                              BASE_PWM - Kp * err);
                }
            }
        }
        send_status_ble();  
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


/* ================= UART TASK ================= */
void UART_Task(void *arg)
{
    uint8_t buf[RX_BUF_SIZE];
    int idx = 0;

    while (1) {
        int len = uart_read_bytes(BT_UART_NUM,
                                  buf + idx,
                                  RX_BUF_SIZE - idx - 1,
                                  pdMS_TO_TICKS(50));
        if (len > 0) {
            idx += len;
            buf[idx] = 0;

            char *nl;
            while ((nl = strchr((char *)buf, '\n'))) {
                *nl = 0;
                parse_line((char *)buf);
                memmove(buf, nl + 1, strlen(nl + 1) + 1);
                idx = strlen((char *)buf);
            }
        }
    }
}

/* ================= ENCODER ISR ================= */
void IRAM_ATTR enc_l_isr(void *arg) { left_ticks++; }
void IRAM_ATTR enc_r_isr(void *arg) { right_ticks++; }

void encoder_init(void)
{
    gpio_config_t io = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
        .pin_bit_mask = (1ULL << ENC_L) | (1ULL << ENC_R)
    };

    gpio_config(&io);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENC_L, enc_l_isr, NULL);
    gpio_isr_handler_add(ENC_R, enc_r_isr, NULL);
}


/* ================= MAIN ================= */
void app_main(void)
{
    uart_driver_install(BT_UART_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(BT_UART_NUM,
        &(uart_config_t){
            .baud_rate = BT_BAUD,
            .data_bits = UART_DATA_8_BITS,
            .stop_bits = UART_STOP_BITS_1
        });
    uart_set_pin(BT_UART_NUM, BT_TX, BT_RX,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);

    gpio_set_direction(L_F, GPIO_MODE_OUTPUT);
    gpio_set_direction(L_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(R_F, GPIO_MODE_OUTPUT);
    gpio_set_direction(R_B, GPIO_MODE_OUTPUT);

    ledc_timer_config(&(ledc_timer_config_t){
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000,
        .duty_resolution = LEDC_TIMER_10_BIT
    });

    ledc_channel_config(&(ledc_channel_config_t){
        .channel = LEDC_CHANNEL_0,
        .gpio_num = L_PWM,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    });

    ledc_channel_config(&(ledc_channel_config_t){
        .channel = LEDC_CHANNEL_1,
        .gpio_num = R_PWM,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    });
    encoder_init();
    xTaskCreatePinnedToCore(UART_Task,  "UART",  4096, NULL, 7,  NULL, 0);
    xTaskCreatePinnedToCore(GyroTask,  "GYRO",  4096, NULL, 6,  NULL, 0);
    xTaskCreatePinnedToCore(SensorTask,"SENSOR",4096, NULL, 5, NULL, 1);

    bt_send("BOOT\r\n");
}

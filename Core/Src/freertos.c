/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : FreeRTOS Multi-Task IMU Sensor Fusion
  *                      Tasks: IMU Read, Kalman Filter, UART Print, LED Blink
  *
  * Features:
  *   - 4 concurrent FreeRTOS tasks with assigned priorities
  *   - MPU6050 IMU read over I2C at 100Hz
  *   - Kalman filter for roll/pitch estimation
  *   - Thread-safe UART output using mutex
  *   - Inter-task communication via queues
  *   - Stack high-water mark monitoring
  *   - Per-task CPU usage via runtime stats
  *   - IWDG watchdog for production-grade reliability
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include "queue.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} IMU_RawData_t;

typedef struct {
    float roll;
    float pitch;
} IMU_Angles_t;

typedef struct {
    float angle;
    float bias;
    float P[2][2];
} KalmanFilter_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR        (0x68 << 1)
#define PWR_MGMT_1          0x6B
#define ACCEL_XOUT_H        0x3B
#define ACCEL_CONFIG        0x1C
#define GYRO_CONFIG         0x1B
#define DT                  0.01f       // 10ms task period
#define RAD_TO_DEG          57.295779513f
#define MAX_TASKS           10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static QueueHandle_t xRawDataQueue;
static QueueHandle_t xAnglesQueue;
static SemaphoreHandle_t xUARTMutex;
static TaskHandle_t xIMUTaskHandle;
static TaskHandle_t xKalmanTaskHandle;
static TaskHandle_t xUARTTaskHandle;
static TaskHandle_t xLEDTaskHandle;

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern IWDG_HandleTypeDef hiwdg;
extern TIM_HandleTypeDef htim2;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void MPU6050_Init(void);
static void MPU6050_Read(IMU_RawData_t *data);
static float Kalman_Update(KalmanFilter_t *kf, float newAngle, float newRate);
static void Kalman_Init(KalmanFilter_t *kf);

void Task_IMU_Read(void *argument);
void Task_Kalman_Filter(void *argument);
void Task_UART_Print(void *argument);
void Task_LED_Blink(void *argument);
/* USER CODE END FunctionPrototypes */

/* USER CODE BEGIN Application */

/* ============================================================
 * MPU6050 Functions
 * ============================================================ */
static void MPU6050_Init(void) {
    uint8_t data = 0x00;
    // Wake up MPU6050
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, 1, &data, 1, 100);
    HAL_Delay(100);
    // Accel: +-2g, Gyro: +-250 deg/s
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG, 1, &data, 1, 100);
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG,  1, &data, 1, 100);
}

static void MPU6050_Read(IMU_RawData_t *data) {
    uint8_t buf[14];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H, 1, buf, 14, 100);
    data->ax = (int16_t)(buf[0]  << 8 | buf[1]);
    data->ay = (int16_t)(buf[2]  << 8 | buf[3]);
    data->az = (int16_t)(buf[4]  << 8 | buf[5]);
    data->gx = (int16_t)(buf[8]  << 8 | buf[9]);
    data->gy = (int16_t)(buf[10] << 8 | buf[11]);
    data->gz = (int16_t)(buf[12] << 8 | buf[13]);
}

/* ============================================================
 * Kalman Filter
 * ============================================================ */
static void Kalman_Init(KalmanFilter_t *kf) {
    kf->angle  = 0.0f;
    kf->bias   = 0.0f;
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
}

static float Kalman_Update(KalmanFilter_t *kf, float newAngle, float newRate) {
    const float Q_angle   = 0.001f;
    const float Q_bias    = 0.003f;
    const float R_measure = 0.03f;

    // Predict
    float rate = newRate - kf->bias;
    kf->angle += DT * rate;
    kf->P[0][0] += DT * (DT * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + Q_angle);
    kf->P[0][1] -= DT * kf->P[1][1];
    kf->P[1][0] -= DT * kf->P[1][1];
    kf->P[1][1] += Q_bias * DT;

    // Update
    float S  = kf->P[0][0] + R_measure;
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;
    float y  = newAngle - kf->angle;
    kf->angle += K0 * y;
    kf->bias  += K1 * y;
    kf->P[0][0] -= K0 * kf->P[0][0];
    kf->P[0][1] -= K0 * kf->P[0][1];
    kf->P[1][0] -= K1 * kf->P[0][0];
    kf->P[1][1] -= K1 * kf->P[0][1];

    return kf->angle;
}

/* ============================================================
 * TASK 1: IMU Read (Priority 4 - Highest)
 * Reads MPU6050 over I2C every 10ms (100Hz)
 * Sends raw data to xRawDataQueue
 * ============================================================ */
void Task_IMU_Read(void *argument) {
    IMU_RawData_t rawData;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(10);

    for (;;) {
        MPU6050_Read(&rawData);
        xQueueOverwrite(xRawDataQueue, &rawData);
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

/* ============================================================
 * TASK 2: Kalman Filter (Priority 3)
 * Receives raw IMU data, applies Kalman filter
 * Sends filtered roll/pitch to xAnglesQueue
 * ============================================================ */
void Task_Kalman_Filter(void *argument) {
    IMU_RawData_t rawData;
    IMU_Angles_t angles;
    KalmanFilter_t kf_roll, kf_pitch;

    Kalman_Init(&kf_roll);
    Kalman_Init(&kf_pitch);

    for (;;) {
        if (xQueueReceive(xRawDataQueue, &rawData, pdMS_TO_TICKS(20)) == pdTRUE) {
            float ax = rawData.ax / 16384.0f;
            float ay = rawData.ay / 16384.0f;
            float az = rawData.az / 16384.0f;
            float gx = rawData.gx / 131.0f;
            float gy = rawData.gy / 131.0f;

            float accel_roll  = atan2f(ay, az) * RAD_TO_DEG;
            float accel_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;

            angles.roll  = Kalman_Update(&kf_roll,  accel_roll,  gx);
            angles.pitch = Kalman_Update(&kf_pitch, accel_pitch, gy);

            xQueueOverwrite(xAnglesQueue, &angles);
        }
    }
}

/* ============================================================
 * TASK 3: UART Print (Priority 2)
 * Prints Roll/Pitch every 100ms
 * Prints Stack HWM + CPU usage every 1 second
 * Uses mutex for thread-safe UART access
 * ============================================================ */
void Task_UART_Print(void *argument) {
    IMU_Angles_t angles;
    char buf[256];
    uint32_t printCount = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(100);

    static char statsBuffer[40 * MAX_TASKS];

    for (;;) {
        if (xQueuePeek(xAnglesQueue, &angles, pdMS_TO_TICKS(50)) == pdTRUE) {

            if (xSemaphoreTake(xUARTMutex, pdMS_TO_TICKS(10)) == pdTRUE) {

                // Roll/Pitch every 100ms
                snprintf(buf, sizeof(buf), "Roll:%6.2f Pitch:%6.2f\r\n",
                         angles.roll, angles.pitch);
                HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);

                // Every 1 second: stack HWM + CPU usage
                if (++printCount % 10 == 0) {

                    // Stack high water marks
                    snprintf(buf, sizeof(buf),
                        "--- Stack HWM (words) IMU:%lu KF:%lu UART:%lu LED:%lu ---\r\n",
                        uxTaskGetStackHighWaterMark(xIMUTaskHandle),
                        uxTaskGetStackHighWaterMark(xKalmanTaskHandle),
                        uxTaskGetStackHighWaterMark(xUARTTaskHandle),
                        uxTaskGetStackHighWaterMark(xLEDTaskHandle));
                    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);

                    // CPU usage per task
                    const char *cpuHeader = "--- CPU Usage ---\r\n";
                    HAL_UART_Transmit(&huart2, (uint8_t*)cpuHeader, strlen(cpuHeader), 100);

                    vTaskGetRunTimeStats(statsBuffer);
                    if(strlen(statsBuffer) == 0)
                        snprintf(statsBuffer, 40*MAX_TASKS, "STATS EMPTY\r\n");
                    HAL_UART_Transmit(&huart2, (uint8_t*)statsBuffer, strlen(statsBuffer), 500);

                    const char *cpuFooter = "-----------------\r\n";
                    HAL_UART_Transmit(&huart2, (uint8_t*)cpuFooter, strlen(cpuFooter), 100);
                }

                xSemaphoreGive(xUARTMutex);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

/* ============================================================
 * TASK 4: LED Blink (Priority 1 - Lowest)
 * Heartbeat blink every 500ms
 * Kicks IWDG watchdog — if this task freezes, MCU resets
 * ============================================================ */
void Task_LED_Blink(void *argument) {
    for (;;) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_IWDG_Refresh(&hiwdg);   // Pet the watchdog every 500ms
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ============================================================
 * MX_FREERTOS_Init — called from main.c
 * Creates queues, mutex, and all 4 tasks
 * ============================================================ */
void MX_FREERTOS_Init(void) {

    // Initialize MPU6050
    MPU6050_Init();

    // Start TIM2 as free-running counter for CPU runtime stats
    HAL_TIM_Base_Start(&htim2);

    // Create queues (depth 1 — always holds latest value)
    xRawDataQueue = xQueueCreate(1, sizeof(IMU_RawData_t));
    xAnglesQueue  = xQueueCreate(1, sizeof(IMU_Angles_t));

    // Create UART mutex for thread-safe transmission
    xUARTMutex = xSemaphoreCreateMutex();

    // Startup message
    const char *startMsg = "FreeRTOS IMU System Started! (IWDG + CPU Stats Active)\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)startMsg, strlen(startMsg), 1000);

    // Create tasks with priorities and stack sizes
    xTaskCreate(Task_IMU_Read,      "IMU",    256, NULL, 4, &xIMUTaskHandle);
    xTaskCreate(Task_Kalman_Filter, "Kalman", 512, NULL, 3, &xKalmanTaskHandle);
    xTaskCreate(Task_UART_Print,    "UART",   512, NULL, 2, &xUARTTaskHandle);
    xTaskCreate(Task_LED_Blink,     "LED",    128, NULL, 1, &xLEDTaskHandle);
}

/* USER CODE END Application */

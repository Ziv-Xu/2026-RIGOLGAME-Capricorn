#include "track.h"
#include "soft_i2c_track.h"
#include "motor.h"
/*==================== 防抖参数配置 ====================*/
#define TRACK_DEBOUNCE_SAMPLES   5
#define TRACK_DEBOUNCE_DELAY_MS  1

/*==================== 循迹权重（从左到右 8 路） ====================*/
static const int IR_Weight[8] = {-4, -3, -2, -1, 1, 2, 3, 4};

/*==================== PID 实例 ====================*/
static PID_TypeDef PID = {12, 0, 6, 0, 0, 0, 0, 0, 0};   // Kp=12, Ki=0, Kd=6

/*==================== 原有 I2C 初始化函数 ====================*/
void Track_Init(void)
{
    HAL_Delay(10);  // 等待模块上电稳定
}

/*==================== 原有 I2C 读取函数（带防抖） ====================*/
uint8_t Track_Read_All(void)
{
    uint8_t last_value, current_value;
    uint8_t stable_count = 0;
    uint32_t timeout = 100;

    last_value = TRACK_I2C_Read_One_Byte(TRACK_I2C_ADDR, TRACK_REG_DATA);
    stable_count = 1;

    while (stable_count < TRACK_DEBOUNCE_SAMPLES && timeout > 0)
    {
        HAL_Delay(TRACK_DEBOUNCE_DELAY_MS);
        current_value = TRACK_I2C_Read_One_Byte(TRACK_I2C_ADDR, TRACK_REG_DATA);

        if (current_value == last_value)
        {
            stable_count++;
        }
        else
        {
            last_value = current_value;
            stable_count = 1;
        }
        timeout--;
    }
    return last_value;
}

uint8_t Track_Read_Channel(uint8_t channel)
{
    uint8_t data;
    if (channel > 7) return 0;
    data = Track_Read_All();
    return (data >> channel) & 0x01;
}

/*==================== 新增：获取加权偏差 ====================*/
int Get_Track_Error(void)
{
    uint8_t status = Track_Read_All();   // bit0~bit7 对应 1~8 路，1=黑线
    int sum = 0;
    int count = 0;

    for (int i = 0; i < 8; i++)
    {
        if (status & (1 << i))
        {
            sum += IR_Weight[i];
            count++;
        }
    }

    if (count == 0)
    {
        // 全丢线处理：返回 0（可在此添加自救逻辑）
        return 0;
    }
    return sum / count;   // 加权平均偏差
}

/*==================== 新增：PID 计算 ====================*/
int PID_Calc(int error)
{
    PID.error = error;
    PID.P = (int)(PID.Kp * PID.error);
    PID.I += (int)(PID.Ki * PID.error);          // 积分项累加
    PID.D = (int)(PID.Kd * (PID.error - PID.last_error));
    PID.output = PID.P + PID.I + PID.D;
    PID.last_error = PID.error;
    return PID.output;
}

/*==================== 新增：循迹主控函数 ====================*/
void Track_Run(void)
{
    #define BASE_SPEED   50
    #define SPEED_MAX    99
    #define SPEED_MIN    0

    int error = Get_Track_Error();
    int pwm   = PID_Calc(error);
    int left  = BASE_SPEED + pwm;
    int right = BASE_SPEED - pwm;

    // 限幅
    if (left  > SPEED_MAX) left  = SPEED_MAX;
    if (left  < SPEED_MIN) left  = SPEED_MIN;
    if (right > SPEED_MAX) right = SPEED_MAX;
    if (right < SPEED_MIN) right = SPEED_MIN;

    Motor(left, right);   // 需用户实现
}
#include <stdio.h>
#include "M051Series.h"
#include "string.h"
#include "PtsPta_TypeDef.h"
#include "PtsPta_Global.h"
#include "pid.h"
int labubu = 0;
int32_t SpdPIDCalc_HI(float NextPoint);
int32_t SpdPIDCalc_LO(float NextPoint);
void OverCurrent_Handler(void);
extern unsigned char T_1ms, T_1ms222;
void Motor_Control_cw_Task();
void Motor_Control_ccw_Task();
#define OVERCURRENT_LIMIT 1400   /// 1200	 // 單位：mA
#define OVERCURRENT_CLEAR 200    // 清除門檻
#define OVERCURRENT_HOLD_LOOP 20 // 每 10ms 呼叫一次，25 次約 250ms

#define SPD_P_DATA_LO 0.080f // 低轉速 p曲線值
#define SPD_I_DATA_LO 0.05f  // 低轉速 i曲線值
#define SPD_D_DATA_LO 0.0f   // 低轉速 d曲線值
#define TARGET_SPEED_LO 650  // 設定630==>實際650 // 目標低轉速
#define TARGET_SPEED_HI 1020 // 350 // 目標高轉速
#define SPD_P_DATA_HI 0.22f  // 0.57f  // 高轉速 p曲線值
#define SPD_I_DATA_HI 0.020f // 0.029f // 高轉速 i曲線值
#define SPD_D_DATA_HI 0.0f   // 高轉速 d曲線值
#define close_cW_Flag 0
typedef struct
{
    __IO int32_t SetPoint;
    __IO float SumError;
    __IO float Proportion;
    __IO float Integral;
    __IO float Derivative;
    __IO int LastError;
    __IO int PrevError;
} PID_TypeDef; // pid結構
float k = 0;
int16_t c = 0, r = 0;
extern unsigned int G_Current;
uint8_t enter_boot_speed_mode = 0;
uint8_t lock_pwm_flag = 0;
PID_TypeDef sPID_LO_CW, sPID_LO_CCW, sPID_HI_CW, sPID_HI_CCW;
uint8_t overcurrent_counter = 0;
uint16_t prev_rpm_cw = 0;
uint16_t prev_current_cw = 0;
uint16_t prev_pwm_cw = 0;
uint16_t prev_rpm_ccw = 0;
uint16_t prev_current_ccw = 0;
uint8_t rpm_count = 0;
uint8_t rpm_burst_flag = 0;
uint8_t rpm_burst_flag222 = 0;
uint8_t current_count = 0;
uint8_t current_burst_flag = 0;
uint16_t rpm_now = 0, rpm_prev = 0;
uint16_t current_now = 0, current_prev = 0;
uint8_t current_dis_flag = 0;
uint8_t current_dis_count = 0;
extern unsigned long int LO_count_300ms;
signed int jump_pwm = 0;
extern unsigned long int HI_count_300ms;
signed int jump_pwm222 = 0;
unsigned int bat = 0;
extern unsigned int L_TargetCount;
float iError_HI_CW = 0, iError_HI_CCW = 0, iError_LO_CW = 0, iError_LO_CCW = 0, iIncpid_HI_CW = 0, iIncpid_HI_CCW = 0, iIncpid_LO_CW = 0, iIncpid_LO_CCW = 0;
extern volatile int32_t g_bWait;
extern unsigned int G_StallCnt;
extern unsigned char errorflag;
uint8_t getextra20data_flag = 0;
uint8_t mcu_boot_key_Up = 1; // down
extern int Enter_HI_Loop;
extern int Enter_LO_Loop;
int temppwm = 0, temppwm222 = 0;
int LO_Loop = 0;
int count_3 = 0;
int count_4 = 0;
unsigned long int HI_count_300ms;
unsigned long int LO_count_300ms;
extern volatile int32_t g_bWait;
extern unsigned int G_StallCnt;
extern unsigned char errorflag;
extern uint8_t getextra20data_flag;
int Enter_HI_Loop = 0;
int Enter_LO_Loop = 0;
int HI_Loop = 0;
extern int temppwm, temppwm222;
extern int LO_Loop;
extern int HI_Loop;
unsigned long int HI_count_300ms;
unsigned long int LO_count_300ms;
extern volatile uint32_t t;
extern volatile uint32_t t2;
#define ROUND_TO_INT32(x) ((int32_t)(x) + 0.5f) >= (x) ? ((int32_t)(x)) : ((uint32_t)(x) + 1)
typedef enum
{
    OVERCURR_IDLE,
    OVERCURR_STEP1,
    OVERCURR_STEP2,
    OVERCURR_STEP3,
    OVERCURR_DONE
} OverCurrentState_t;

OverCurrentState_t oc_state = OVERCURR_IDLE;
uint32_t oc_timestamp = 0;
extern uint32_t t3;
void delayms(uint32_t ms)
{
    CLK_SysTickDelay(ms * 1000);
}

// ======================
// 參數設定區
// ======================
#define RPM_AVG_SIZE 1 // 移動平均樣本數
#define CUR_AVG_SIZE 1
#define LOAD_CONFIRM_CNT 1   // 連續成立次數
#define RPM_DROP_TH_HI 20.0f // rpm 下降閾值 //high7
#define CUR_RISE_TH 1.0f     // 電流上升閾值 (A)
#define RPM_DROP_TH_LO 20.0f // rpm 下降閾值 //high7

// ======================
// 全域變數
// ======================
float rpm_buf[RPM_AVG_SIZE] = {0};
float cur_buf[CUR_AVG_SIZE] = {0};
uint8_t rpm_idx = 0, cur_idx = 0;

float rpm_avg = 0, cur_avg = 0;
float rpm_prev_avg = 0, cur_prev_avg = 0;

uint8_t load_count = 0;
uint8_t load_flag = 0;

// ======================
// 移動平均函式
// ======================
float moving_average(float *buf, uint8_t *idx, uint8_t size, float new_val)
{
    uint8_t i;
    float sum;
    buf[*idx] = new_val;
    (*idx)++;
    if (*idx >= size)
        *idx = 0;

    sum = 0;
    for (i = 0; i < size; i++)
        sum += buf[i];

    return sum / size;
}
#include <stdbool.h>
#define AVG_WINDOW 3
float current_buf[AVG_WINDOW] = {0};
uint8_t idx = 0;
float avg_current = 0;

void UpdateCurrent(float current_sample)
{
    int i;
    float sum = 0;
    current_buf[idx] = current_sample;
    idx = (idx + 1) % AVG_WINDOW;
    sum = 0;
    for (i = 0; i < AVG_WINDOW; i++)
        sum += current_buf[i];
    avg_current = sum / AVG_WINDOW;
}

#define SPEED_AVG_WINDOW 3 // 移動平均窗長，可調整
float speed_buf[SPEED_AVG_WINDOW] = {0};
uint8_t speed_idx = 0;
float avg_speed = 0;

#define SPEED_AVG_WINDOW2 3 // 移動平均窗長，可調整
float speed_buf2[SPEED_AVG_WINDOW2] = {0};
uint8_t speed_idx2 = 0;
float avg_speed2 = 0;

void UpdateSpeed_CW(float speed_sample)
{
    float sum = 0;
    int i;
    speed_buf[speed_idx] = speed_sample;
    speed_idx = (speed_idx + 1) % SPEED_AVG_WINDOW;

    sum = 0;
    for (i = 0; i < SPEED_AVG_WINDOW; i++)
        sum += speed_buf[i];
    avg_speed = sum / SPEED_AVG_WINDOW;
}

void UpdateSpeed_CCW(float speed_sample)
{
    float sum = 0;
    int i;
    speed_buf2[speed_idx2] = speed_sample;
    speed_idx2 = (speed_idx2 + 1) % SPEED_AVG_WINDOW2;

    sum = 0;
    for (i = 0; i < SPEED_AVG_WINDOW2; i++)
        sum += speed_buf2[i];
    avg_speed2 = sum / SPEED_AVG_WINDOW2;
}
////////////////////////////////////////////////////低速部分

#include "M051Series.h"
#include <stdint.h>
#include <stdbool.h>

// 閾值設定
#define TH_PWM_START 100 // 啟動區門檻
#define TH_RPM_MIN 60    // 轉速過低門檻
#define TH_COUNT_MAX 3   // 連續次數門檻

// 狀態變數
uint8_t stuck_count = 0;
bool load_detected = false;

/////////////////////////////////////////////////////////////
uint8_t enter_speed_count = 0;
uint8_t G_ScrewRPM_zero_count = 0;
int cc1 = 0, cc2 = 0;
int r1 = 0, r2 = 0;
uint8_t save_special_flag = 0;
// unsigned int var[3][200] = {0};
uint32_t varcount = 0;
int16_t ccc = 0;

unsigned int Temp_var[3][200] = {1};

extern uint8_t stall_flag;
float now_delta_pwm = 0;
float cal_delta_pwm = 0;
float cal__pwm = 0;
float ii_average = 0;
float RPM_average = 0;
float CMP_average = 0;
uint8_t doonce_RPM_average = 0;
float RPM_sum_Flag = 0;
float On_the_spot_fastening_average = 0;
float On_the_spot_fastening_flag = 0;
float On_the_spot_fastening_count = 0;
uint8_t no_load_flag = 255;
uint8_t Motor_Drifting_HI_CW_flag = 0;
uint8_t Motor_Drifting_LO_CW_flag = 0;
int16_t delta_pwm = 0;
float delta_pwm_avg = 0;
uint16_t record_No_load_PWM_MAX_LO = 410;
uint16_t record_No_load_PWM_MAX_HI = 645;
uint8_t record_No_load_PWM_MAX_LO_count = 0;
uint8_t record_No_load_PWM_MAX_HI_count = 0;
uint8_t No_load_flag = 0;
uint8_t No_load_flag2 = 0;
void detect_enter_boot_speed_CW_HI(uint16_t rpm, uint16_t current) // 正轉高速強力鎖付函數
{

    bool rpm_drop;
    uint8_t i;
    bool cur_rise;

      /* if (varcount == 42) // 42的時候判斷RPM  (原理在該時間點 空載rpm>鎖付中負載rpm>原地鎖付rpm)
       {
           if (G_ScrewRPM <= 85)      // 目前這個rpm值是在原地鎖付的範圍內,但若閥值設定不好或三者的rpm太接近，就會讓條件成立，這時請將count時間點調到比較後面，但時間點還要調在原地鎖付能聽脫的範圍，你太晚跳，原地鎖付也跳不過
               rpm_burst_flag222 = 4; // 進入加速強力鎖付模式
       }
       else
       {
       }
       */
       if ((varcount > 20) && (((int)prev_rpm_cw - (int)G_ScrewRPM) > 30))//過高速正轉cmk專用項 // 3以上才能正確跳脫 2以下有時抖動會有雜訊變成條件意外進入加速狀態，建議判斷至少(int)prev_rpm_cw - (int)G_ScrewRPM)都要2以上，意思就是遇到有負載
       {
           rpm_burst_flag222 = 1; // 確定有遇到負載
       }

      /* if ((No_load_flag == 1) && (varcount >= 142)) // 如果遇到負載後，等到一段時間(varcount >= 142)到，鎖付到底，然後進入強力鎖付加速模式
       {
           rpm_burst_flag222 = 1; // 進入加速強力鎖付模式
       }
       else
       {
       }
			 */
   
    if (varcount <= 199)
    {

        Temp_var[0][varcount] = G_ScrewRPM;
        Temp_var[1][varcount] = G_Current;
        Temp_var[2][varcount] = G_PwmCMR;
    }

    varcount++;

    if (lock_pwm_flag == 0) // 電流不過高的情況下
    {

        if (rpm_burst_flag222 == 1) // 第一種強力鎖付模式，先進入第一階段全力鎖付力道 //過cmk測試，只過高紐
					{

            //G_PwmCMR = 2210;
					G_PwmCMR = 1860;//1810 ok 1860 good cmk //過高速正轉cmk專用項

           // rpm_burst_flag222 = 2;
        }
        else if (rpm_burst_flag222 == 2) ////第一種強力鎖付模式進入第二階段減速階段，這兩階段配合可使鎖付的力道感覺加速沒那麼大 比較平滑，若用最大力就感覺加速太明顯
        {
            if (G_PwmCMR >= 100) // pwm>100滿足，則每次進來漸少pwm20
                G_PwmCMR = G_PwmCMR - 20;
        }
        else if (rpm_burst_flag222 == 4) // 第二種強力鎖付模式-用在原地跳脫,直接開最大
        {
            G_PwmCMR = 2210;
        }

        else
        {
        }
    }

    prev_rpm_cw = rpm;
    prev_current_cw = G_Current;
    prev_pwm_cw = G_PwmCMR;
}

void detect_enter_boot_speed_CW_LO(uint16_t rpm, uint16_t current) // 正轉高速強力鎖付函數
{

    bool rpm_drop;
    uint8_t i;
    bool cur_rise;

   /*   if (varcount == 44) // 44的時候判斷RPM  (原理在該時間點 空載rpm>鎖付中負載rpm>原地鎖付rpm)
      {
          if (G_ScrewRPM <= 29)      // 目前這個rpm值是在原地鎖付的範圍內,但若閥值設定不好或三者的rpm太接近，就會讓條件成立，這時請將count時間點調到比較後面，但時間點還要調在原地鎖付能聽脫的範圍，你太晚跳，原地鎖付也跳不過
              rpm_burst_flag222 = 4; // 進入加速強力鎖付模式
      }
      else
      {
      }
*/
      if ((varcount > 30) && (((int)prev_rpm_cw - (int)G_ScrewRPM) > 30)) // 3以上才能正確跳脫 2以下有時抖動會有雜訊變成條件意外進入加速狀態，建議判斷至少(int)prev_rpm_cw - (int)G_ScrewRPM)都要2以上，意思就是遇到有負載
      {
          rpm_burst_flag222 = 1; // 確定有遇到負載
      }

     /* if ((No_load_flag == 1) && (varcount >= 210)) // 如果遇到負載後，等到一段時間(varcount >= 210)到，鎖付到底，然後進入強力鎖付加速模式
      {
          rpm_burst_flag222 = 1; // 進入加速強力鎖付模式
      }
      else
      {
      }
       */
    if (varcount <= 199)
    {

        Temp_var[0][varcount] = G_ScrewRPM;
        Temp_var[1][varcount] = G_Current;
        Temp_var[2][varcount] = G_PwmCMR;
    }

    varcount++;

    if (lock_pwm_flag == 0) // 電流不過高的情況下
    {

        if (rpm_burst_flag222 == 1) // 第一種強力鎖付模式，先進入第一階段全力鎖付力道
        {

            //G_PwmCMR = 2210;
						G_PwmCMR = 2210;
          //  rpm_burst_flag222 = 2;
        }
        else if (rpm_burst_flag222 == 2) ////第一種強力鎖付模式進入第二階段減速階段，這兩階段配合可使鎖付的力道感覺加速沒那麼大 比較平滑，若用最大力就感覺加速太明顯
        {
            if (G_PwmCMR >= 100)
                G_PwmCMR = G_PwmCMR - 20;
        }
        else if (rpm_burst_flag222 == 4) // pwm>100滿足，則每次進來漸少pwm20
        {
            G_PwmCMR = 2210; // 第二種強力鎖付模式-用在原地跳脫,直接開最大
        }

        else
        {
        }
    }

    prev_rpm_cw = rpm;
    prev_current_cw = G_Current;
    prev_pwm_cw = G_PwmCMR;
}

void detect_enter_boot_speed_CCW_HI(uint16_t rpm, uint16_t current) // 偵測進入加速模式在反轉階段
{

    UpdateCurrent((float)c);
    UpdateSpeed_CCW((float)G_PwmCMR);
    if (gu8StartSpeed == HI) // 高轉速情況下
    {
        if (varcount <= 199)
        {
            Temp_var[0][varcount] = G_ScrewRPM;
            Temp_var[1][varcount] = G_Current;
            Temp_var[2][varcount] = G_PwmCMR;
            varcount++;
        }

        //  if (avg_speed2 > 180)
        //      rpm_burst_flag = 1;
    }

    if (lock_pwm_flag != 1) // 電流不過高的情況下
    {
        if (rpm_burst_flag == 1) // 進入全力鎖付模式
        {

            enter_boot_speed_mode = 1;
            G_PwmCMR = 2210;
        }
    }

    prev_rpm_ccw = rpm;
    prev_current_ccw = current;
}
void detect_enter_boot_speed_CCW_LO(uint16_t rpm, uint16_t current) // 偵測進入加速模式在反轉階段
{

    UpdateCurrent((float)c);
    UpdateSpeed_CCW((float)G_PwmCMR);
    if (varcount <= 199)
    {
        Temp_var[0][varcount] = G_ScrewRPM;
        Temp_var[1][varcount] = G_Current;
        Temp_var[2][varcount] = G_PwmCMR;
        varcount++;
    }
    // if (avg_speed2 > 140) // 2040 115 2011 119
    //   rpm_burst_flag = 1;

    if (lock_pwm_flag != 1) // 電流不過高的情況下
    {
        if (rpm_burst_flag == 1) // 第一階段旗標和第二階段判斷旗標都成立時
        {

            enter_boot_speed_mode = 1;
        }
    }

    prev_rpm_ccw = rpm;
    prev_current_ccw = current;
}

void PID_ParamInit_LO_CW(void) // 正轉的pid的低轉速初始化
{
    sPID_LO_CW.LastError = 0;
    sPID_LO_CW.PrevError = 0;
    /*if (G_Voltage_Battery <= 1760)
    {
        sPID_LO_CW.Proportion = 0.25f;
        sPID_LO_CW.Integral = SPD_I_DATA_LO;
    }
    else if (G_Voltage_Battery <= 1800)
    {
        sPID_LO_CW.Proportion = 0.1f;
        sPID_LO_CW.Integral = SPD_I_DATA_LO;
    }
    else if (G_Voltage_Battery <= 1860)
    {
        sPID_LO_CW.Proportion = 0.05;
        sPID_LO_CW.Integral = SPD_I_DATA_LO;
    }
    else
    {
        sPID_LO_CW.Proportion = SPD_P_DATA_LO;
        sPID_LO_CW.Integral = SPD_I_DATA_LO;
    }
        */
    sPID_LO_CW.Proportion = SPD_P_DATA_LO;
    sPID_LO_CW.Integral = SPD_I_DATA_LO;
    sPID_LO_CW.Derivative = SPD_D_DATA_LO;
    sPID_LO_CW.SetPoint = TARGET_SPEED_LO; // 設定目標轉速
}

void PID_ParamInit_LO_CCW(void) // 逆轉的低轉速pid參數
{
    sPID_LO_CCW.LastError = 0;
    sPID_LO_CCW.PrevError = 0;
    /*if (G_Voltage_Battery <= 1760)
    {
        sPID_LO_CCW.Proportion = 0.25f;
        sPID_LO_CCW.Integral = SPD_I_DATA_LO;
    }
    else if (G_Voltage_Battery <= 1800)
    {
        sPID_LO_CCW.Proportion = 0.1f;
        sPID_LO_CCW.Integral = SPD_I_DATA_LO;
    }
    else if (G_Voltage_Battery <= 1860)
    {
        sPID_LO_CCW.Proportion = 0.05;
        sPID_LO_CCW.Integral = SPD_I_DATA_LO;
    }
    else
    {
        sPID_LO_CCW.Proportion = SPD_P_DATA_LO;
        sPID_LO_CCW.Integral = SPD_I_DATA_LO;
    }
        */
    sPID_LO_CCW.Proportion = SPD_P_DATA_LO;
    sPID_LO_CCW.Integral = SPD_I_DATA_LO;
    sPID_LO_CCW.Derivative = SPD_D_DATA_LO;
    sPID_LO_CCW.SetPoint = TARGET_SPEED_LO; // 設定目標轉速
}

void PID_ParamInit_HI_CW(void) // 設定正轉高轉速的pid參數
{
    sPID_HI_CW.LastError = 0;
    sPID_HI_CW.PrevError = 0;
    /*if (G_Voltage_Battery <= 1760)
    {
        sPID_HI_CW.Proportion = 4.5f;
        sPID_HI_CW.Integral = SPD_I_DATA_HI;
    }
    else if (G_Voltage_Battery <= 1800)
    {
        sPID_HI_CW.Proportion = 2.3f;
        sPID_HI_CW.Integral = SPD_I_DATA_HI;
    }
    else if (G_Voltage_Battery <= 1860)
    {
        sPID_HI_CW.Proportion = 1.0f;
        sPID_HI_CW.Integral = SPD_I_DATA_HI;
    }
    else if (G_Voltage_Battery <= 1920)
    {
        sPID_HI_CW.Proportion = 0.6f;
        sPID_HI_CW.Integral = SPD_I_DATA_HI;
    }
    else
    {
        sPID_HI_CW.Proportion = SPD_P_DATA_HI;
        sPID_HI_CW.Integral = SPD_I_DATA_HI;
    }
        */
    sPID_HI_CW.Proportion = SPD_P_DATA_HI;
    sPID_HI_CW.Integral = SPD_I_DATA_HI;
    sPID_HI_CW.Derivative = SPD_D_DATA_HI;
    sPID_HI_CW.SetPoint = TARGET_SPEED_HI; // 設定目標轉速
}

void PID_ParamInit_HI_CCW(void) // 設定高轉逆轉的pid參數初始化
{
    sPID_HI_CCW.LastError = 0;
    sPID_HI_CCW.PrevError = 0;
    /*if (G_Voltage_Battery <= 1760)
    {
        sPID_HI_CCW.Proportion = 4.5f;
        sPID_HI_CCW.Integral = SPD_I_DATA_HI;
    }
    else if (G_Voltage_Battery <= 1800)
    {
        sPID_HI_CCW.Proportion = 2.3f;
        sPID_HI_CCW.Integral = SPD_I_DATA_HI;
    }
    else if (G_Voltage_Battery <= 1860)
    {
        sPID_HI_CCW.Proportion = 1.0f;
        sPID_HI_CCW.Integral = SPD_I_DATA_HI;
    }
    else if (G_Voltage_Battery <= 1920)
    {
        sPID_HI_CCW.Proportion = 0.6f;
        sPID_HI_CCW.Integral = SPD_I_DATA_HI;
    }
    else
    {
        sPID_HI_CCW.Proportion = SPD_P_DATA_HI;
        sPID_HI_CCW.Integral = SPD_I_DATA_HI;
    }
        */
    sPID_HI_CCW.Proportion = SPD_P_DATA_HI;
    sPID_HI_CCW.Integral = SPD_I_DATA_HI;
    sPID_HI_CCW.Derivative = SPD_D_DATA_HI;
    sPID_HI_CCW.SetPoint = TARGET_SPEED_HI; // 設定目標轉速
}

int32_t SpdPIDCalc_LO_CW(float NextPoint) // pid低轉速正轉運行函數
{

    iError_LO_CW = (float)sPID_LO_CW.SetPoint - NextPoint;
    if ((iError_LO_CW < 30.0f) && (iError_LO_CW > -30.0f))
        iError_LO_CW = 0.0f;

    iIncpid_LO_CW = (sPID_LO_CW.Proportion * iError_LO_CW) - (sPID_LO_CW.Integral * sPID_LO_CW.LastError) + (sPID_LO_CW.Derivative * sPID_LO_CW.PrevError);

    sPID_LO_CW.PrevError = sPID_LO_CW.LastError;
    sPID_LO_CW.LastError = iError_LO_CW;
    // printf("ROUND_TO_INT32(iIncpid)=%d",ROUND_TO_INT32(iIncpid));
    // t++;
    return (ROUND_TO_INT32(iIncpid_LO_CW));
}

int32_t SpdPIDCalc_LO_CCW(float NextPoint) // pid低轉速逆轉運行函數
{

    iError_LO_CCW = (float)sPID_LO_CCW.SetPoint - NextPoint;
    if ((iError_LO_CCW < 30.0f) && (iError_LO_CCW > -30.0f))

        iError_LO_CCW = 0.0f;

    iIncpid_LO_CCW = (sPID_LO_CCW.Proportion * iError_LO_CCW) - (sPID_LO_CCW.Integral * sPID_LO_CCW.LastError) + (sPID_LO_CCW.Derivative * sPID_LO_CCW.PrevError);

    sPID_LO_CCW.PrevError = sPID_LO_CCW.LastError;
    sPID_LO_CCW.LastError = iError_LO_CCW;
    // printf("ROUND_TO_INT32(iIncpid)=%d",ROUND_TO_INT32(iIncpid));
    // t++;
    return (ROUND_TO_INT32(iIncpid_LO_CCW));
}

int32_t SpdPIDCalc_HI_CW(float NextPoint) // pid高轉速正轉運行函數
{
    iError_HI_CW = (float)sPID_HI_CW.SetPoint - NextPoint;

    if ((iError_HI_CW < 30.0f) && (iError_HI_CW > -30.0f))
        iError_HI_CW = 0.0f;

    iIncpid_HI_CW = (sPID_HI_CW.Proportion * iError_HI_CW) - (sPID_HI_CW.Integral * sPID_HI_CW.LastError) + (sPID_HI_CW.Derivative * sPID_HI_CW.PrevError);

    sPID_HI_CW.PrevError = sPID_HI_CW.LastError; //
    sPID_HI_CW.LastError = iError_HI_CW;

    //
    // printf("ROUND_TO_INT32(iIncpid)=%d",ROUND_TO_INT32(iIncpid));
    return (ROUND_TO_INT32(iIncpid_HI_CW)); //
}
int32_t SpdPIDCalc_HI_CCW(float NextPoint) // pid高轉速逆轉運行函數
{
    iError_HI_CCW = (float)sPID_HI_CCW.SetPoint - NextPoint;
    if ((iError_HI_CCW < 30.0f) && (iError_HI_CCW > -30.0f))
        iError_HI_CCW = 0.0f;

    iIncpid_HI_CCW = (sPID_HI_CCW.Proportion * iError_HI_CCW) - (sPID_HI_CCW.Integral * sPID_HI_CCW.LastError) + (sPID_HI_CCW.Derivative * sPID_HI_CCW.PrevError);

    sPID_HI_CCW.PrevError = sPID_HI_CCW.LastError;
    sPID_HI_CCW.LastError = iError_HI_CCW;

    //
    // printf("ROUND_TO_INT32(iIncpid)=%d",ROUND_TO_INT32(iIncpid));
    return (ROUND_TO_INT32(iIncpid_HI_CCW));
}
#define PWM_MAX 2210
uint8_t lock_pwm_flag2 = 0;
extern uint32_t t3, t4;
void Exponential_PWM_Ramp_CCW(void) // 增量型pid加速程式-逆轉
{

    WDT_RESET_COUNTER();
    // static uint8_t overcurrent_counter = 0;
    //  --- 過電流保護 ---
    if (G_Current > OVERCURRENT_LIMIT) // 電流超過高臨界值
    {
        overcurrent_counter++;
        if (overcurrent_counter >= OVERCURRENT_HOLD_LOOP) // 超過門檻次數
        {

            SetPWMCMR(0);       // 清除pwm
            lock_pwm_flag2 = 1; // 進入電流保護鎖定機制
            overcurrent_counter = 0;
            // printf("overcurrent\r\n");
        }
    }
    else if (G_Current < OVERCURRENT_CLEAR) // 電流低於最小安全值
    {
        if (overcurrent_counter > 0)
            overcurrent_counter--;
        if (overcurrent_counter == 0)
        {
            // t = 0;
            // t2 = 0;
            lock_pwm_flag2 = 0; // 解除電流保護鎖定
        }
    }
    else
    {
    }

    if (gu8StartSpeed == HI) // 高轉速
    {

        k = 10;
    }
    else // 低轉速
    {
        k = 0.1;
    }
    if (lock_pwm_flag2 == 0) // 馬達電流在安全區域值
    {

        // G_PwmCMR = PWM_MAX * (1 - expf(-k * t4)); // 設定pwm參數以指數型態及相關時間參數去控制
        G_PwmCMR = PWM_MAX;
    }
    WDT_RESET_COUNTER();
}

extern OverCurrentState_t oc_state;
extern uint32_t oc_timestamp;
extern void Switch_P13_To_ADC(void);
extern void Switch_P13_To_GPIO(void);
extern uint8_t stall_flag;
void default_init(void) // 每次解除按壓馬達按鍵還原參數值
{
    int i, i2;
    // LIGHT_OFF;
    // C_PtsPta_Buzzer_OFF;
    /*if (varcount > 0)
    {
        for (i = 0; i < varcount; i++)
        {

            printf("G_ScrewRPM = %d,\t\t,G_Current=%d,\t\tG_PwmCMR = %d\r\n", Temp_var[0][i], Temp_var[1][i], Temp_var[2][i]);
        }

        varcount = 0;
    }
        */

    // printf("ff=%f\r\n",cal_delta_pwm);
    labubu = 0;
    Enter_HI_Loop = 0;
    HI_Loop = 0;

    HI_count_300ms = 0;
    jump_pwm = 0;

    Enter_LO_Loop = 0;
    LO_Loop = 0;
    LO_count_300ms = 0;
    jump_pwm222 = 0;
    bat = G_Voltage_Battery;
    L_TargetCount = 42;

    rpm_now = 0, rpm_prev = 0;
    current_now = 0, current_prev = 0;

    sPID_LO_CW.SetPoint = TARGET_SPEED_LO;  //
    sPID_HI_CW.SetPoint = TARGET_SPEED_HI;  //
    sPID_LO_CCW.SetPoint = TARGET_SPEED_LO; //
    sPID_HI_CCW.SetPoint = TARGET_SPEED_HI; //

    prev_rpm_cw = 0;
    prev_current_cw = 0;
    prev_rpm_ccw = 0;
    prev_current_ccw = 0;

    rpm_count = 0;
    rpm_burst_flag = 0;
    current_count = 0;
    current_burst_flag = 0;

    enter_boot_speed_mode = 0;
    overcurrent_counter = 0;
    t = 0;
    t2 = 0;
    t3 = 0;
    t4 = 0;
    G_StallCnt = 0;
    current_dis_flag = 0;
    current_dis_count = 0;
    oc_state = OVERCURR_IDLE;
    oc_timestamp = 0;
    G_PwmCMR = 0;
    lock_pwm_flag = 0;
    lock_pwm_flag2 = 0;
    current_count = 0;

    c = 0, r = 0;
    load_count = 0;
    PID_ParamInit_HI_CCW();
    PID_ParamInit_HI_CW();
    PID_ParamInit_LO_CCW();
    PID_ParamInit_LO_CW();
    for (i = 0; i < AVG_WINDOW; i++)
    {
        current_buf[i] = 0;
    }
    for (i = 0; i < SPEED_AVG_WINDOW; i++)
        speed_buf[i] = 0;
    enter_speed_count = 0;
    overcurrent_counter = 0;
    rpm_burst_flag = 0;
    G_ScrewRPM_zero_count = 0;
    avg_current = 0;
    avg_speed = 0;
    cc1 = 0, cc2 = 0;
    G_ScrewRPM = 0;
    rpm_burst_flag222 = 0;
    ccc = 0;
    varcount = 0;
    stall_flag = 0;
    stuck_count = 0;
    load_detected = false;
    no_load_flag = 255;
    Motor_Drifting_HI_CW_flag = 0;
    Motor_Drifting_LO_CW_flag = 0;
    prev_pwm_cw = 0;
    ii_average = 0;
    RPM_average = 0;
    RPM_sum_Flag = 0;
    On_the_spot_fastening_average = 0;
    On_the_spot_fastening_flag = 0;
    On_the_spot_fastening_count = 0;
    doonce_RPM_average = 0;
    CMP_average = 0;
    delta_pwm = 0;
    delta_pwm_avg = 0;

    record_No_load_PWM_MAX_LO_count = 0;
    record_No_load_PWM_MAX_HI_count = 0;
    parm_init();
    No_load_flag = 0;
    No_load_flag2 = 0;
}

void uvw_check_function(void) // uvw異常的判斷程式
{

    if (gu8StartSpeed == HI)
    {
        if (G_StallCnt > 10 && G_MotorStatus == eMotorStatus_Normal && HI_count_300ms < 20)
        {
            errorflag = eBehaving_Uvw_Error; // 1.eBehaving_eBehaving_Uvw_Error
            G_BuzzerLED = errorflag;
            Buzzer_LED_Behaving(&G_BuzzerLED);
        }
    }
    else
    {
        if (G_StallCnt > 10 && G_MotorStatus == eMotorStatus_Normal && LO_count_300ms < 20)
        {
            errorflag = eBehaving_Uvw_Error; // 1.eBehaving_eBehaving_Uvw_Error
            G_BuzzerLED = errorflag;
            Buzzer_LED_Behaving(&G_BuzzerLED);
        }
    }
}

void parm_init(void) // 馬達壓板按壓後第一次會先還原預設值
{

    if (gu8StartSpeed == HI)
    {

        if (Enter_HI_Loop == 0) // real 2ms
        {

            //  Enter_HI_Loop = 1;
            enter_boot_speed_mode = 0;
            t = 0;
            t2 = 0;
            t3 = 0;
            t4 = 0;
            sPID_HI_CW.PrevError = 0;
            sPID_HI_CW.LastError = 0;
            sPID_HI_CW.Proportion = SPD_P_DATA_HI;
            sPID_HI_CW.Integral = SPD_I_DATA_HI;
            iError_HI_CW = 0;
            iIncpid_HI_CW = 0;
            sPID_HI_CCW.PrevError = 0;
            sPID_HI_CCW.LastError = 0;
            sPID_HI_CCW.Proportion = SPD_P_DATA_HI;
            sPID_HI_CCW.Integral = SPD_I_DATA_HI;
            iError_HI_CCW = 0;
            iIncpid_HI_CCW = 0;
            HI_count_300ms = 0;
            overcurrent_counter = 0;
            T_1ms = 0;
            // cal_delta_pwm=155.0f+(2040-G_Voltage_Battery)*0.1667;

        } // 20.4v 114.5pwm  20.1 119.5pwm
    }
    else
    {
        if (Enter_LO_Loop == 0) // real 2ms
        {

            //  Enter_LO_Loop = 1;
            enter_boot_speed_mode = 0;
            t = 0;
            t2 = 0;
            t3 = 0;
            t4 = 0;
            sPID_LO_CW.PrevError = 0;
            sPID_LO_CW.LastError = 0;
            sPID_LO_CW.Proportion = SPD_P_DATA_LO;
            sPID_LO_CW.Integral = SPD_I_DATA_LO;
            iError_LO_CW = 0;
            iIncpid_LO_CW = 0;
            sPID_LO_CCW.PrevError = 0;
            sPID_LO_CCW.LastError = 0;
            sPID_LO_CCW.Proportion = SPD_P_DATA_LO;
            sPID_LO_CCW.Integral = SPD_I_DATA_LO;
            iError_LO_CCW = 0;
            iIncpid_LO_CCW = 0;
            sPID_LO_CW.SetPoint = TARGET_SPEED_LO;
            LO_count_300ms = 0;
            overcurrent_counter = 0;
            T_1ms = 0;
            // cal_delta_pwm=116.5f+(2040-G_Voltage_Battery)*0.1650;
        }
    }
}

void motor_algorithm_HI(void) // 馬達演算法
{

    if (T_1ms222 >= 3)
    {
        T_1ms222 = 0;
        HI_Loop++;
        if (G_Direction == _CW) // 正轉部分
        {
            if (HI_Loop == 1)
            {
                temppwm222 = G_PwmCMR;
                temppwm = SpdPIDCalc_HI_CW(G_ScrewRPM); // pid控速最終達到290rpm
                if (((temppwm + temppwm222) < 0) || ((temppwm + temppwm222) > 2210))
                {
                }
                else
                    G_PwmCMR = G_PwmCMR + temppwm;
            }
            else if (HI_Loop == 3)
            {
              // printf("%d,%d,%d,%d,1,1\n", G_ScrewRPM, G_Current, G_PwmCMR, G_Voltage_Battery); // 這個部分要搭配程式開cmk才會過，不然扭力輸出打cmk會不穩
                // printf("iError_HI_CW=%f\n\n",iError_HI_CW);
               detect_enter_boot_speed_CW_HI(G_ScrewRPM, G_Current); // 偵測馬達是否遇到阻力,則進加速模式
                HI_Loop = 0;
                labubu++;

                HI_Loop = 0;
            }
            else if (HI_Loop >= 4) // 2m*5=10ms
            {
            }
            else
            {
            }
        }
        else // 反轉部分 if (G_Direction == _CCW)
        {
            if (HI_Loop == 1)
            {
                temppwm222 = G_PwmCMR;
                temppwm = SpdPIDCalc_HI_CCW(G_ScrewRPM);
                if (((temppwm + temppwm222) < 0) || ((temppwm + temppwm222) > 2210))
                {
                }
                else
                    G_PwmCMR = G_PwmCMR + temppwm;
            }
            else if (HI_Loop == 3)
            {
                if (enter_boot_speed_mode == 0)
                {
                    detect_enter_boot_speed_CCW_HI(G_ScrewRPM, G_Current); // 偵測馬達是否遇到阻力,則進加速模式
                    // if (enter_boot_speed_mode == 0)
                    HI_Loop = 0;
                }
                else
                {
                    Exponential_PWM_Ramp_CCW();
                }
                // printf("%d,%d,%d,%d,1,1\n", G_ScrewRPM, G_Current, G_PwmCMR, G_Voltage_Battery); // 這個部分要搭配程式開cmk才會過，不然扭力輸出打cmk會不穩
                //  printf("iError_HI_CCW=%f\n\n",iError_HI_CCW);
                labubu++;
            }
            else if (HI_Loop >= 4) // 2m*5=10ms
            {
            }
            else
            {
            }
        }
    }
}

void motor_algorithm_LO(void) // 馬達演算法
{

    if (T_1ms222 >= 3) // 1ms以內
    {
        T_1ms222 = 0;
        LO_Loop++;
        if (G_Direction == _CW) // 正轉部分
        {
            if (LO_Loop == 1)
            {
                temppwm222 = G_PwmCMR;
                temppwm = SpdPIDCalc_LO_CW(G_ScrewRPM); // pid控速最終達到290rpm
                if (((temppwm + temppwm222) < 0) || ((temppwm + temppwm222) > 2210))
                {
                }
                else
                    G_PwmCMR = G_PwmCMR + temppwm;
            }
            else if (LO_Loop == 3)
            {
                  detect_enter_boot_speed_CW_LO(G_ScrewRPM, G_Current); // 偵測馬達是否遇到阻力,則進加速模式
                //	printf("%d,%d,%d,%d,1,1\n", G_ScrewRPM, G_Current, G_PwmCMR, G_Voltage_Battery); // 這個部分要搭配程式開cmk才會過，不然扭力輸出打cmk會不穩

                //  printf("%d,%d,%d,%d,1,1\n", G_ScrewRPM, G_Current, G_PwmCMR, G_Voltage_Battery); // 這個部分要搭配程式開cmk才會過，不然扭力輸出打cmk會不穩
                LO_Loop = 0;
                // labubu++;
            }
            else if (LO_Loop >= 4) // 2m*5=10ms
            {

                // LO_Loop = 0;
            }
            else
            {
            }
        }
        else // 反轉部分 if (G_Direction == _CCW) //
        {
            if (LO_Loop == 1) // 2m*5=10ms
            {
                temppwm222 = G_PwmCMR;
                temppwm = SpdPIDCalc_LO_CCW(G_ScrewRPM); // pid控速最終達到290rpm
                if (((temppwm + temppwm222) < 0) || ((temppwm + temppwm222) > 2210))
                {
                }
                else
                    G_PwmCMR = G_PwmCMR + temppwm;
            }
            else if (LO_Loop == 3)
            {
                if (enter_boot_speed_mode == 0)
                {
                    detect_enter_boot_speed_CCW_LO(G_ScrewRPM, G_Current); // 偵測馬達是否遇到阻力,則進加速模式
                                                                           // if (enter_boot_speed_mode == 0)
                    LO_Loop = 0;
                }
                else
                {
                    Exponential_PWM_Ramp_CCW();
                }

                // printf("%d,%d,%d,%d,1,1\n", G_ScrewRPM, G_Current, G_PwmCMR, G_Voltage_Battery); // 這個部分要搭配程式開cmk才會過，不然扭力輸出打cmk會不穩
                // printf("%d,%d,%d,%d,1,1\n", G_ScrewRPM, G_Current, G_PwmCMR, G_Voltage_Battery); // 這個部分要搭配程式開cmk才會過，不然扭力輸出打cmk會不穩
                labubu++;
                LO_Loop = 0;
            }
            else if (LO_Loop >= 4) // 2m*5=10ms
            {
            }
            else
            {
            }
        }
    }
}

void OverCurrent_Handler(void) // 這邊是過流時的燈源警示狀態
{
    switch (oc_state)
    {
    case OVERCURR_STEP1:
        C_NG_RED_ON;
        // printf("# 88888888\r\n");
        C_Buzzer_ON;
        if (T_1ms - oc_timestamp >= 500)
        { // 500ms
            oc_state = OVERCURR_STEP2;
            oc_timestamp = T_1ms;
            C_NG_RED_OFF;
            C_Buzzer_OFF;
        }
        break;
    case OVERCURR_STEP2:
        if (T_1ms - oc_timestamp >= 100)
        { // 100ms
            oc_state = OVERCURR_STEP3;
            oc_timestamp = T_1ms;
            C_Buzzer_ON;
        }
        break;
    case OVERCURR_STEP3:
        if (T_1ms - oc_timestamp >= 200)
        { // 200ms
            oc_state = OVERCURR_DONE;
            C_Buzzer_OFF;
        }
        break;
    case OVERCURR_DONE:
        // 保護動作完成，可保持在此或重置為 IDLE
        break;
    default:
        break;
    }
}
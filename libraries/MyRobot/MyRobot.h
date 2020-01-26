#ifndef MYROBOT_H
#define MYROBOT_H

#include <XM540-W270-R.h>
#include <math.h>
#include <SD.h>
#include <SPI.h>

//Default Setting
#define FL_SWITCH_ID    0
#define FL_YALL_ID      1
#define FL_HIP_ID       2
#define FL_KNEE_ID      3
#define FL_WHEEL_ID     4
#define FR_SWITCH_ID    5
#define FR_YALL_ID      6
#define FR_HIP_ID       7
#define FR_KNEE_ID      8
#define FR_WHEEL_ID     9
#define BL_SWITCH_ID    10
#define BL_YALL_ID      11
#define BL_HIP_ID       12
#define BL_KNEE_ID      13
#define BL_WHEEL_ID     14
#define BR_SWITCH_ID    15
#define BR_YALL_ID      16
#define BR_HIP_ID       17
#define BR_KNEE_ID      18
#define BR_WHEEL_ID     19



//Value Settings
#define TORQUE_ENABLE           1
#define TORQUE_DISABLE          0
#define LED_ON                  1
#define LED_OFF                 0
#define VELOCITY_LIMIT          1023
#define MIN_POSITION_VALUE      -1048575 //-1048575~1048575
#define MAX_POSITION_VALUE      1048575  //-256~256 revolution


//Gain
#define SWITCH_POSITION_P_GAIN  4000 //0~16383
#define SWITCH_POSITION_I_GAIN  0
#define SWITCH_POSITION_D_GAIN  300
#define YALL_POSITION_P_GAIN    4000 //0~16383
#define YALL_POSITION_I_GAIN    0
#define YALL_POSITION_D_GAIN    300
#define HIP_POSITION_P_GAIN     4000 //0~16383
#define HIP_POSITION_I_GAIN     0
#define HIP_POSITION_D_GAIN     300
#define KNEE_POSITION_P_GAIN    1000 //0~16383
#define KNEE_POSITION_I_GAIN    0
#define KNEE_POSITION_D_GAIN    300
#define WHEEL_VELOCITY_P_GAIN   1000
#define WHEEL_VELOCITY_I_GAIN   3000


// OpenCR buttons
#define PUSHSW1 34
#define PUSHSW2 35
#define LED1 22
#define LED2 23
#define LED3 24

//SPI Settings
const int mosi = 11;
const int miso = 12;
const int sck = 13;
const int cs = 10;

#define VELOCITY_P_GAIN 1000
#define VELOCITY_I_GAIN 1920

#define WHEEL_GEAR_RATIO 3.0



#define NUMJOINTS 20 //モータの総数
#define LEGNUM  4 //脚の本数

#define OFFSET_FILE "OFFSET.txt" //オフセットの設定ファイル(変更するな！)


//初期姿勢の角度
//行は各脚の部位(switch, yall hip knee wheel)，列は脚の位置を表す(FL,FR,BL,BR)
double q_init[5][LEGNUM] = {
    //       FL          FR         BL         BR
    {        0,          0,         0,         0},
    {        0,          0,         0,         0},
    { M_PI / 6,   M_PI / 6, -M_PI / 6, -M_PI / 6},
    {-M_PI / 3,  -M_PI / 3,  M_PI / 3,  M_PI / 3},
    {        0,          0,         0,         0}
};

double L1 = 0.252;
double L2 = 0.252;

int32_t offset[NUMJOINTS];

/* それぞれの減速比 */
double gearRatio[5] = {
    4.0, 2.0, 3.24, 2.5, 0.298
};

//Angle[rad] -> Value
int convertAngle2Value(double rad){
    return 4095 * rad / (2 * M_PI);
}
// Velocity->Value
int32_t convertRpm2Value(double rpm){
    return ((rpm / (VELOCITY_COEFFICIENT * gearRatio[4])) + 0.5);
}

//Value -> Velocity
double convertValue2Rpm(int32_t val){
    return (val * VELOCITY_COEFFICIENT * gearRatio[4]);
}

#endif

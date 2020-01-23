/* 車輪走行のデータ取得用プログラム
　 3秒待ってからMAX速度(Valu=1023)で5秒車輪走行
   減速開始から5秒後までデータ取得
 */
#include <MyRobot.h>
#include <SD.h>
#include <SPI.h>

#define BAUDRATE                        2000000
#define DEVICENAME                      "/dev/ttyACM0"

#define FILENAME "OFFSET.txt"
#define LOG_FILE  "104.txt"


void ReadData(int32_t *q_, int16_t *current_, uint16_t *voltage_);
void WriteData(int32_t *q_, int16_t *current_, uint16_t *voltage_);


void initializeFL(void);
void initializeFR(void);
void initializeBL(void);
void initializeBR(void);
void enableFLTorque(void);
void enableFRTorque(void);
void enableBLTorque(void);
void enableBRTorque(void);
void disableAllTorque(void);


//Instance
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
HardwareTimer Timer(TIMER_CH1);

//Data file settings
File myFile;
bool flag = false;


//SDカードから読み取るオフセットの受け皿
int32_t offset_buffer[5];

double initialPose[5] = {
    0, 0, M_PI / 6, -M_PI/3, 0
};


//Time settings
double current_time = 0; //[ms]
double waiting_time = 3000; //[ms]
double endtime = 8000; //[ms]
double sampling_end = 13000; //[ms]

//データ取得用配列
int32_t q[5];
int16_t current[5];
uint16_t voltage[5];


void ReadData(int32_t *q_, int16_t *current_, uint16_t *voltage_){
    //Read Data
    for (int i = 0; i < 5; i++){//position & velocity
        if(i % 5 != 4){ //leg
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, i, ADDR_PRESENT_POSITION, (uint32_t*)&q_[i], &dxl_error);
        }
        else{ //Wheel
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, i, ADDR_PRESENT_VELOCITY, (uint32_t*)&q_[i], &dxl_error);
        }

        //current
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, i, ADDR_PRESENT_CURRENT, (uint16_t*)&current_[i], &dxl_error);  

        //voltage
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, i, ADDR_PRESENT_INPUT_VOLTAGE, &voltage_[i], &dxl_error);
    }
}

void WriteData(int32_t *q_, int16_t *current_, uint16_t *voltage_){
    //Write Data (SD)
    myFile.print(current_time);
    myFile.print(", ");
    
    for (int i = 0; i < 5; i++){
        if(i % 5 != 4){ //leg
            myFile.print(q_[i]);
            myFile.print(", ");
        }
        else{ //Wheel
            myFile.print(q_[i]);
            myFile.print(", ");
        }
    }
    myFile.print(" | ");

    for (int i = 0; i < 5; i++){
        myFile.print(current_[i]);
        myFile.print(", ");
    }
    myFile.print(" | ");
    
    for (int i = 0; i < 5; i++){
        myFile.print(voltage_[i]);
        myFile.print(", ");
    }
    myFile.print(" | ");

    myFile.println();

}

void WheelMove(void){
    ReadData(q, current, voltage);
    WriteData(q, current, voltage);
    current_time += SCAN_RATE /1000;

    Serial.println(current_time);

    //3sec -> start 
    if(current_time > waiting_time && flag == false){
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 4, ADDR_GOAL_VELOCITY, 1023, &dxl_error);
      //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 9, ADDR_GOAL_VELOCITY, 1023, &dxl_error);
      //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 14, ADDR_GOAL_VELOCITY, 1023, &dxl_error);
      //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 19, ADDR_GOAL_VELOCITY, 1023, &dxl_error);
      flag = true;
    }

    //8sec -> stop 
    if(current_time > endtime){
        // Setting Target Velocity
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 4, ADDR_GOAL_VELOCITY, 0, &dxl_error);
        //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 9, ADDR_GOAL_VELOCITY, 0, &dxl_error);
        //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 14, ADDR_GOAL_VELOCITY, 0, &dxl_error);
        //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 19, ADDR_GOAL_VELOCITY, 0, &dxl_error);

    }

    if(current_time > sampling_end){
        //recording finish!
        myFile.close();
        Serial.println("Sampling has been finished !");
        Serial.end();
        Timer.stop();

        Timer.detachInterrupt();
    }
}


//車輪を設置させる前の状態を記録
void ReadInitialState(int32_t *q_, int16_t *current_, uint16_t *voltage_){
    for (int i = 0; i < 5; i++){//position & velocity
        if(i % 5 != 4){ //leg
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, i, ADDR_PRESENT_POSITION, (uint32_t*)&q_[i], &dxl_error);
        }
        else{ //Wheel
            dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, i, ADDR_PRESENT_VELOCITY, (uint32_t*)&q_[i], &dxl_error);
        }
    
        //current
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, i, ADDR_PRESENT_CURRENT, (uint16_t*)&current_[i], &dxl_error);  

        //voltage
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, i, ADDR_PRESENT_INPUT_VOLTAGE, &voltage_[i], &dxl_error);
    }

    myFile.print("Initial_State");
    myFile.print(", ");
    
    for (int i = 0; i < 5; i++){
        if(i % 5 != 4){ //leg
            myFile.print(q_[i]);
            myFile.print(", ");
        }
        else{ //Wheel
            myFile.print(q_[i]);
            myFile.print(", ");
        }
    }

    for (int i = 0; i < 5; i++){
        myFile.print(current_[i]);
        myFile.print(", ");
    }
    
    for (int i = 0; i < 5; i++){
        myFile.print(voltage_[i]);
        myFile.print(", ");
    }
    myFile.println();
}


void initializeFL(void){
    //initialize FL_Switch -------------------------------------------------------------------
    //Setting Operating Mode to Position-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FL_SWITCH_ID, ADDR_OPERATING_MODE, EXTENDTED_POSITION_CONTROL_MODE, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FL_SWITCH_ID, ADDR_MAX_POSITION_LIMIT, MAX_POSITION_VALUE, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FL_SWITCH_ID, ADDR_MIN_POSITION_LIMIT, MIN_POSITION_VALUE, &dxl_error);
    // Setting Position P Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FL_SWITCH_ID, ADDR_POSITION_P_GAIN, SWITCH_POSITION_P_GAIN, &dxl_error);
    // Setting Position I Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FL_SWITCH_ID, ADDR_POSITION_I_GAIN, SWITCH_POSITION_I_GAIN, &dxl_error);
    // Setting Position D Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FL_SWITCH_ID, ADDR_POSITION_D_GAIN, SWITCH_POSITION_D_GAIN, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FL_SWITCH_ID, ADDR_LED, LED_ON, &dxl_error);
    //Profile Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FL_SWITCH_ID, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY, &dxl_error);
    //Profile Acceleration
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FL_SWITCH_ID, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION, &dxl_error);
    //initialize FL_Yall --uint8_t *id_, -----------------------------------------------------------------
    //Setting Operating Mode to Position-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FL_YALL_ID, ADDR_OPERATING_MODE, EXTENDTED_POSITION_CONTROL_MODE, &dxl_error);
    // Setting Position P Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FL_YALL_ID, ADDR_POSITION_P_GAIN, YALL_POSITION_P_GAIN, &dxl_error);
    // Setting Position I Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FL_YALL_ID, ADDR_POSITION_I_GAIN, YALL_POSITION_I_GAIN, &dxl_error);
    // Setting Position D Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FL_YALL_ID, ADDR_POSITION_D_GAIN, YALL_POSITION_D_GAIN, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FL_YALL_ID, ADDR_LED, LED_ON, &dxl_error);
    //Profile Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FL_YALL_ID, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY, &dxl_error);
    //Profile Acceleration
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FL_YALL_ID, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION, &dxl_error);
    //initialize FL_Hip -------------------------------------------------------------------
    //Setting Operating Mode to Position-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FL_HIP_ID, ADDR_OPERATING_MODE, EXTENDTED_POSITION_CONTROL_MODE, &dxl_error);
    // Setting Position P Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FL_HIP_ID, ADDR_POSITION_P_GAIN, HIP_POSITION_P_GAIN, &dxl_error);
    // Setting Position I Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FL_HIP_ID, ADDR_POSITION_I_GAIN, HIP_POSITION_I_GAIN, &dxl_error);
    // Setting Position D Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FL_HIP_ID, ADDR_POSITION_D_GAIN, HIP_POSITION_D_GAIN, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FL_HIP_ID, ADDR_LED, LED_ON, &dxl_error);
    //Profile Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FL_HIP_ID, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY, &dxl_error);
    //Profile Acceleration
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FL_HIP_ID, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION, &dxl_error);

    //initialize FL_Knee -------------------------------------------------------------------
    //Setting Operating Mode to Position-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FL_KNEE_ID, ADDR_OPERATING_MODE, EXTENDTED_POSITION_CONTROL_MODE, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FL_KNEE_ID, ADDR_MAX_POSITION_LIMIT, MAX_POSITION_VALUE, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FL_KNEE_ID, ADDR_MIN_POSITION_LIMIT, MIN_POSITION_VALUE, &dxl_error);
    // Setting Position P Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FL_KNEE_ID, ADDR_POSITION_P_GAIN, KNEE_POSITION_P_GAIN, &dxl_error);
    // Setting Position I Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FL_KNEE_ID, ADDR_POSITION_I_GAIN, KNEE_POSITION_I_GAIN, &dxl_error);
    // Setting Position D Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FL_KNEE_ID, ADDR_POSITION_D_GAIN, KNEE_POSITION_D_GAIN, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FL_KNEE_ID, ADDR_LED, LED_ON, &dxl_error);
    //Profile Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FL_KNEE_ID, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY, &dxl_error);
    //Profile Acceleration
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FL_KNEE_ID, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION, &dxl_error);
    //initialize FL_Wheel -------------------------------------------------------------------
    //Setting Operating Mode to Velocity-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FL_WHEEL_ID, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE, &dxl_error);
    // Setting Position P Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FL_WHEEL_ID, ADDR_VELOCITY_P_GAIN, WHEEL_VELOCITY_P_GAIN, &dxl_error);
    // Setting Position I Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FL_WHEEL_ID, ADDR_VELOCITY_I_GAIN, WHEEL_VELOCITY_I_GAIN, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FL_WHEEL_ID, ADDR_LED, LED_ON, &dxl_error);
    // Setting Velocity Limit
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FL_WHEEL_ID, ADDR_VELOCITY_LIMIT, VELOCITY_LIMIT, &dxl_error);
    // Setting Wheel Profile Acceleration
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FL_WHEEL_ID, ADDR_PROFILE_ACCELERATION, WHEEL_PROFILE_ACCELERATION, &dxl_error);
}

void initializeFR(void){
    //initialize FR_Switch -------------------------------------------------------------------
    //Setting Operating Mode to Position-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FR_SWITCH_ID, ADDR_OPERATING_MODE, EXTENDTED_POSITION_CONTROL_MODE, &dxl_error);
    // Setting Position P Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FR_SWITCH_ID, ADDR_POSITION_P_GAIN, SWITCH_POSITION_P_GAIN, &dxl_error);
    // Setting Position I Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FR_SWITCH_ID, ADDR_POSITION_I_GAIN, SWITCH_POSITION_I_GAIN, &dxl_error);
    // Setting Position D Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FR_SWITCH_ID, ADDR_POSITION_D_GAIN, SWITCH_POSITION_D_GAIN, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FR_SWITCH_ID, ADDR_LED, LED_ON, &dxl_error);
    //Profile Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FR_SWITCH_ID, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY, &dxl_error);
    //Profile Acceleration
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FR_SWITCH_ID, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION, &dxl_error);
    //initialize FR_Yall -------------------------------------------------------------------
    //Setting Operating Mode to Position-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FR_YALL_ID, ADDR_OPERATING_MODE, EXTENDTED_POSITION_CONTROL_MODE, &dxl_error);
    // Setting Position P Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FR_YALL_ID, ADDR_POSITION_P_GAIN, YALL_POSITION_P_GAIN, &dxl_error);
    // Setting Position I Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FR_YALL_ID, ADDR_POSITION_I_GAIN, YALL_POSITION_I_GAIN, &dxl_error);
    // Setting Position D Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FR_YALL_ID, ADDR_POSITION_D_GAIN, YALL_POSITION_D_GAIN, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FR_YALL_ID, ADDR_LED, LED_ON, &dxl_error);
    //Profile Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FR_YALL_ID, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY, &dxl_error);
    //Profile Acceleration
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FR_YALL_ID, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION, &dxl_error);
    //initialize FR_Hip -------------------------------------------------------------------
    //Setting Operating Mode to Position-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FR_HIP_ID, ADDR_OPERATING_MODE, EXTENDTED_POSITION_CONTROL_MODE, &dxl_error);
    // Setting Position P Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FR_HIP_ID, ADDR_POSITION_P_GAIN, HIP_POSITION_P_GAIN, &dxl_error);
    // Setting Position I Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FR_HIP_ID, ADDR_POSITION_I_GAIN, HIP_POSITION_I_GAIN, &dxl_error);
    // Setting Position D Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FR_HIP_ID, ADDR_POSITION_D_GAIN, HIP_POSITION_D_GAIN, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FR_HIP_ID, ADDR_LED, LED_ON, &dxl_error);
    //Profile Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FR_HIP_ID, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY, &dxl_error);
    //Profile Acceleration
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FR_HIP_ID, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION, &dxl_error);
    //initialize FR_Knee -------------------------------------------------------------------
    //Setting Operating Mode to Position-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FR_KNEE_ID, ADDR_OPERATING_MODE, EXTENDTED_POSITION_CONTROL_MODE, &dxl_error);
    // Setting Position P Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FR_KNEE_ID, ADDR_POSITION_P_GAIN, KNEE_POSITION_P_GAIN, &dxl_error);
    // Setting Position I Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FR_KNEE_ID, ADDR_POSITION_I_GAIN, KNEE_POSITION_I_GAIN, &dxl_error);
    // Setting Position D Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FR_KNEE_ID, ADDR_POSITION_D_GAIN, KNEE_POSITION_D_GAIN, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FR_KNEE_ID, ADDR_LED, LED_ON, &dxl_error);
    //Profile Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FR_KNEE_ID, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY, &dxl_error);
    //Profile Acceleration
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FR_KNEE_ID, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION, &dxl_error);
    //initialize FR_Wheel -------------------------------------------------------------------
    //Setting Operating Mode to Velocity-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FR_WHEEL_ID, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE, &dxl_error);
    // Setting Position P Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FR_WHEEL_ID, ADDR_VELOCITY_P_GAIN, WHEEL_VELOCITY_P_GAIN, &dxl_error);
    // Setting Position I Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, FR_WHEEL_ID, ADDR_VELOCITY_I_GAIN, WHEEL_VELOCITY_I_GAIN, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FR_WHEEL_ID, ADDR_LED, LED_ON, &dxl_error);
    // Setting Velocity Limit
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FR_WHEEL_ID, ADDR_VELOCITY_LIMIT, VELOCITY_LIMIT, &dxl_error);
    // Setting Wheel Profile Acceleration
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FR_WHEEL_ID, ADDR_PROFILE_ACCELERATION, WHEEL_PROFILE_ACCELERATION, &dxl_error);
}

void initializeBL(void){
    //initialize BL_Switch -------------------------------------------------------------------
    //Setting Operating Mode to Position-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BL_SWITCH_ID, ADDR_OPERATING_MODE, EXTENDTED_POSITION_CONTROL_MODE, &dxl_error);
    // Setting Position P Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BL_SWITCH_ID, ADDR_POSITION_P_GAIN, SWITCH_POSITION_P_GAIN, &dxl_error);
    // Setting Position I Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BL_SWITCH_ID, ADDR_POSITION_I_GAIN, SWITCH_POSITION_I_GAIN, &dxl_error);
    // Setting Position D Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BL_SWITCH_ID, ADDR_POSITION_D_GAIN, SWITCH_POSITION_D_GAIN, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BL_SWITCH_ID, ADDR_LED, LED_ON, &dxl_error);
    //Profile Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BL_SWITCH_ID, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY, &dxl_error);
    //Profile Acceleration
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BL_SWITCH_ID, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION, &dxl_error);

    //initialize BL_Yall -------------------------------------------------------------------
    //Setting Operating Mode to Position-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BL_YALL_ID, ADDR_OPERATING_MODE, EXTENDTED_POSITION_CONTROL_MODE, &dxl_error);
    // Setting Position P Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BL_YALL_ID, ADDR_POSITION_P_GAIN, YALL_POSITION_P_GAIN, &dxl_error);
    // Setting Position I Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BL_YALL_ID, ADDR_POSITION_I_GAIN, YALL_POSITION_I_GAIN, &dxl_error);
    // Setting Position D Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BL_YALL_ID, ADDR_POSITION_D_GAIN, YALL_POSITION_D_GAIN, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BL_YALL_ID, ADDR_LED, LED_ON, &dxl_error);
    //Profile Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BL_YALL_ID, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY, &dxl_error);
    //Profile Acceleration
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BL_YALL_ID, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION, &dxl_error);

    //initialize BL_Hip -------------------------------------------------------------------
    //Setting Operating Mode to Position-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BL_HIP_ID, ADDR_OPERATING_MODE, EXTENDTED_POSITION_CONTROL_MODE, &dxl_error);
    // Setting Position P Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BL_HIP_ID, ADDR_POSITION_P_GAIN, HIP_POSITION_P_GAIN, &dxl_error);
    // Setting Position I Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BL_HIP_ID, ADDR_POSITION_I_GAIN, HIP_POSITION_I_GAIN, &dxl_error);
    // Setting Position D Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BL_HIP_ID, ADDR_POSITION_D_GAIN, HIP_POSITION_D_GAIN, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BL_HIP_ID, ADDR_LED, LED_ON, &dxl_error);
    //Profile Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BL_HIP_ID, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY, &dxl_error);
    //Profile Acceleration
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BL_HIP_ID, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION, &dxl_error);

    //initialize BL_Knee -------------------------------------------------------------------
    //Setting Operating Mode to Position-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BL_KNEE_ID, ADDR_OPERATING_MODE, EXTENDTED_POSITION_CONTROL_MODE, &dxl_error);
    // Setting Position P Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BL_KNEE_ID, ADDR_POSITION_P_GAIN, KNEE_POSITION_P_GAIN, &dxl_error);
    // Setting Position I Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BL_KNEE_ID, ADDR_POSITION_I_GAIN, KNEE_POSITION_I_GAIN, &dxl_error);
    // Setting Position D Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BL_KNEE_ID, ADDR_POSITION_D_GAIN, KNEE_POSITION_D_GAIN, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BL_KNEE_ID, ADDR_LED, LED_ON, &dxl_error);
    //Profile Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BL_KNEE_ID, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY, &dxl_error);
    //Profile Acceleration
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BL_KNEE_ID, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION, &dxl_error);

    //initialize BL_Wheel -------------------------------------------------------------------
    //Setting Operating Mode to Velocity-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BL_WHEEL_ID, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE, &dxl_error);
    // Setting Position P Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BL_WHEEL_ID, ADDR_VELOCITY_P_GAIN, WHEEL_VELOCITY_P_GAIN, &dxl_error);
    // Setting Position I Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BL_WHEEL_ID, ADDR_VELOCITY_I_GAIN, WHEEL_VELOCITY_I_GAIN, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BL_WHEEL_ID, ADDR_LED, LED_ON, &dxl_error);
    // Setting Velocity Limit
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BL_WHEEL_ID, ADDR_VELOCITY_LIMIT, VELOCITY_LIMIT, &dxl_error);
    // Setting Wheel Profile Acceleration
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BL_WHEEL_ID, ADDR_PROFILE_ACCELERATION, WHEEL_PROFILE_ACCELERATION, &dxl_error);
}

void initializeBR(void){
    //initialize BR_Switch -------------------------------------------------------------------
    //Setting Operating Mode to Position-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BR_SWITCH_ID, ADDR_OPERATING_MODE, EXTENDTED_POSITION_CONTROL_MODE, &dxl_error);
    // Setting Position P Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BR_SWITCH_ID, ADDR_POSITION_P_GAIN, SWITCH_POSITION_P_GAIN, &dxl_error);
    // Setting Position I Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BR_SWITCH_ID, ADDR_POSITION_I_GAIN, SWITCH_POSITION_I_GAIN, &dxl_error);
    // Setting Position D Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BR_SWITCH_ID, ADDR_POSITION_D_GAIN, SWITCH_POSITION_D_GAIN, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BR_SWITCH_ID, ADDR_LED, LED_ON, &dxl_error);
    //Profile Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BR_SWITCH_ID, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY, &dxl_error);
    //Profile Acceleration
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BR_SWITCH_ID, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION, &dxl_error);
    //initialize BR_Yall -------------------------------------------------------------------
    //Setting Operating Mode to Position-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BR_YALL_ID, ADDR_OPERATING_MODE, EXTENDTED_POSITION_CONTROL_MODE, &dxl_error);
    // Setting Position P Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BR_YALL_ID, ADDR_POSITION_P_GAIN, YALL_POSITION_P_GAIN, &dxl_error);
    // Setting Position I Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BR_YALL_ID, ADDR_POSITION_I_GAIN, YALL_POSITION_I_GAIN, &dxl_error);
    // Setting Position D Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BR_YALL_ID, ADDR_POSITION_D_GAIN, YALL_POSITION_D_GAIN, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BR_YALL_ID, ADDR_LED, LED_ON, &dxl_error);
        //Profile Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BR_YALL_ID, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY, &dxl_error);
    //Profile Acceleration
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BR_YALL_ID, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION, &dxl_error);

    //initialize BR_Hip -------------------------------------------------------------------
    //Setting Operating Mode to Position-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BR_HIP_ID, ADDR_OPERATING_MODE, EXTENDTED_POSITION_CONTROL_MODE, &dxl_error);
    // Setting Position P Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BR_HIP_ID, ADDR_POSITION_P_GAIN, HIP_POSITION_P_GAIN, &dxl_error);
    // Setting Position I Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BR_HIP_ID, ADDR_POSITION_I_GAIN, HIP_POSITION_I_GAIN, &dxl_error);
    // Setting Position D Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BR_HIP_ID, ADDR_POSITION_D_GAIN, HIP_POSITION_D_GAIN, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BR_HIP_ID, ADDR_LED, LED_ON, &dxl_error);
    //Profile Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BR_HIP_ID, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY, &dxl_error);
    //Profile Acceleration
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BR_HIP_ID, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION, &dxl_error);

    //initialize BR_Knee -------------------------------------------------------------------
    //Setting Operating Mode to Position-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BR_KNEE_ID, ADDR_OPERATING_MODE, EXTENDTED_POSITION_CONTROL_MODE, &dxl_error);
    // Setting Position P Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BR_KNEE_ID, ADDR_POSITION_P_GAIN, KNEE_POSITION_P_GAIN, &dxl_error);
    // Setting Position I Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BR_KNEE_ID, ADDR_POSITION_I_GAIN, KNEE_POSITION_I_GAIN, &dxl_error);
    // Setting Position D Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BR_KNEE_ID, ADDR_POSITION_D_GAIN, KNEE_POSITION_D_GAIN, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BR_KNEE_ID, ADDR_LED, LED_ON, &dxl_error);
    //Profile Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BR_KNEE_ID, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY, &dxl_error);
    //Profile Acceleration
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BR_KNEE_ID, ADDR_PROFILE_ACCELERATION, PROFILE_ACCELERATION, &dxl_error);

    //initialize BR_Wheel -------------------------------------------------------------------
    //Setting Operating Mode to Velocity-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BR_WHEEL_ID, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE, &dxl_error);
    // Setting Position P Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BR_WHEEL_ID, ADDR_VELOCITY_P_GAIN, WHEEL_VELOCITY_P_GAIN, &dxl_error);
    // Setting Position I Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, BR_WHEEL_ID, ADDR_VELOCITY_I_GAIN, WHEEL_VELOCITY_I_GAIN, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BR_WHEEL_ID, ADDR_LED, LED_ON, &dxl_error);
    // Setting Velocity Limit
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BR_WHEEL_ID, ADDR_VELOCITY_LIMIT, VELOCITY_LIMIT, &dxl_error);
    // Setting Wheel Profile Acceleration
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BR_WHEEL_ID, ADDR_PROFILE_ACCELERATION, WHEEL_PROFILE_ACCELERATION, &dxl_error);
}

void enableFLTorque(void){
    // Enable FL Switch Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FL_SWITCH_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // Enable FL Yall Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FL_YALL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // Enable FL Hip Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FL_HIP_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // Enable FL Knee Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FL_KNEE_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // Enable FL Wheel Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FL_WHEEL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // Setting Target Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FL_WHEEL_ID, ADDR_GOAL_VELOCITY, 0, &dxl_error);
    //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FL_WHEEL_ID, ADDR_GOAL_PWM, 855, &dxl_error);
}

void enableFRTorque(void){
    // Enable FR Switch Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FR_SWITCH_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // Enable FR Yall Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FR_YALL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // Enable FR Hip Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FR_HIP_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // Enable FR Knee Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FR_KNEE_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // Enable FR Wheel Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, FR_WHEEL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // Setting Target Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FR_WHEEL_ID, ADDR_GOAL_VELOCITY, 0, &dxl_error);
    //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FR_WHEEL_ID, ADDR_GOAL_PWM, 855, &dxl_error);
}

void enableBLTorque(void){
    // Enable BL Switch Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BL_SWITCH_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // Enable BL Yall Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BL_YALL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // Enable BL Hip Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BL_HIP_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // Enable BL Knee Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BL_KNEE_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // Enable BL Wheel Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BL_WHEEL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // Setting Target Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BL_WHEEL_ID, ADDR_GOAL_VELOCITY, 0, &dxl_error);
    //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BL_WHEEL_ID, ADDR_GOAL_PWM, 855, &dxl_error);
}

void enableBRTorque(void){
    // Enable BR Switch Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BR_SWITCH_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // Enable BR Yall Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BR_YALL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // Enable BR Hip Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BR_HIP_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // Enable BR Knee Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BR_KNEE_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // Enable BR Wheel Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, BR_WHEEL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // Setting Target Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BR_WHEEL_ID, ADDR_GOAL_VELOCITY, 0, &dxl_error);
    //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BR_WHEEL_ID, ADDR_GOAL_PWM, 855, &dxl_error);
}

void disableAllTorque(void){
    for(int i=0;i<5; i++){
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    }
}

/*
int getch(){
    while(1){
        if(Serial.available()>0){
            break;
        }
    }
}*/

//Value->Angle
double convertValue2Angle(int val){
    return 2*PI*val/4095 - PI;
}

void setup() {
    Serial.begin(BAUDRATE);
    while(!Serial);
    Serial.println("start...");
    
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    //Open Port
    portHandler->openPort();

    // Set port baudrate
    portHandler->setBaudRate(BAUDRATE);

    //SD card
    pinMode(cs, OUTPUT);
    if(!SD.begin(cs)){
        Serial.println("No SD !");
        while(1);
    }
    Serial.println("SD Initialization Done!");
    
    //offset Read
    myFile = SD.open(FILENAME, FILE_READ);

    Serial.print("filesize is :");
    Serial.println(myFile.size());
    if (myFile){
        for (int i = 0; i < 5;i++){
            offset_buffer[i] = myFile.parseInt();
            Serial.println(offset_buffer[i]);
        }
    }
    myFile.close();

    //Check File
    if(SD.exists(LOG_FILE)){
        Serial.println("The same name file has already exist !!");
        while(1);
    }


    initializeFL();
    initializeFR();
    initializeBL();
    initializeBR();

    //初期姿勢へ
    Serial.println("Press s to change initial pose");
    while (1){
        char input = 'i';
        if (Serial.available() > 0){
            input = Serial.read();
            if (input == 's'){
                break;
            }
        }
    }


    delay(500);
    enableFLTorque();
    enableFRTorque();
    enableBLTorque();
    enableBRTorque();

    /* 初期姿勢の目標角を設定 */
   for (int i = 0; i < 5; i++){
        if(i % 5 != 4){ //leg
            if(i/10 == 0){ //Forward
                dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDR_GOAL_POSITION, offset_buffer[i] + convertAngle2Value(gearRatio[i%5] * initialPose[i % 5]), &dxl_error);
            }else{ //Backward
                dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDR_GOAL_POSITION, offset_buffer[i] + convertAngle2Value(-gearRatio[i % 5] * initialPose[i % 5]), &dxl_error);
            }
        }
        else{ //Wheel
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDR_GOAL_VELOCITY, 0, &dxl_error);
        }
    }  


    

    //motor data書き込み
    myFile = SD.open(LOG_FILE, FILE_WRITE); 
    
    Serial.println("File opened");
    myFile.println();
    
    delay(500);
    ReadInitialState(q, current, voltage);


    if(myFile){
        myFile.print("time[ms] ,");
        myFile.print("FL_Switch[rad] ,");
        myFile.print("FL_Yall[rad] ,");
        myFile.print("FL_Hip[rad] ,");
        myFile.print("FL_Knee[rad] ,");
        myFile.print("FL_Wheel[rpm] ,");
        myFile.print("FR_Switch[rad] ,");
        myFile.print("FR_Yall[rad] ,");
        myFile.print("FR_Hip[rad] ,");
        myFile.print("FR_Knee[rad] ,");
        myFile.print("FR_Wheel[rpm] ,");
        myFile.print("BL_Switch[rad] ,");
        myFile.print("BL_Yall[rad] ,");
        myFile.print("BL_Hip[rad] ,");
        myFile.print("BL_Knee[rad] ,");
        myFile.print("BL_Wheel[rpm] ,");
        myFile.print("BR_Switch[rad] ,");
        myFile.print("BR_Yall[rad] ,");
        myFile.print("BR_Hip[rad] ,");
        myFile.print("BR_Knee[rad] ,");
        myFile.print("BR_Wheel[rpm] ,");
        myFile.print("FL_Switch_Current[mA] ,");
        myFile.print("FL_Yall_Current[mA] ,");
        myFile.print("FL_Hip_Current[mA] ,");
        myFile.print("FL_Knee_Current[mA] ,");
        myFile.print("FL_Wheel_Current[mA] ,");
        myFile.print("FR_Switch_Current[mA] ,");
        myFile.print("FR_Yall_Current[mA] ,");
        myFile.print("FR_Hip_Current[mA] ,");
        myFile.print("FR_Knee_Current[mA] ,");
        myFile.print("FR_Wheel_Current[mA] ,");
        myFile.print("BL_Switch_Current[mA] ,");
        myFile.print("BL_Yall_Current[mA] ,");
        myFile.print("BL_Hip_Current[mA] ,");
        myFile.print("BL_Knee_Current[mA] ,");
        myFile.print("BL_Wheel_Current[mA] ,");
        myFile.print("BR_Switch_Current[mA] ,");
        myFile.print("BR_Yall_Current[mA] ,");
        myFile.print("BR_Hip_Current[mA] ,");
        myFile.print("BR_Knee_Current[mA] ,");
        myFile.print("BR_Wheel_Current[mA] ,");
        myFile.print("FL_Switch_Voltage[V] ,");
        myFile.print("FL_Yall_Voltage[V] ,");
        myFile.print("FL_Hip_Voltage[V] ,");
        myFile.print("FL_Knee_Voltage[V] ,");
        myFile.print("FL_Wheel_Voltage[V] ,");
        myFile.print("FR_Switch_Voltage[V] ,");
        myFile.print("FR_Yall_Voltage[V] ,");
        myFile.print("FR_Hip_Voltage[V] ,");
        myFile.print("FR_Knee_Voltage[V] ,");
        myFile.print("FR_Wheel_Voltage[V] ,");
        myFile.print("BL_Switch_Voltage[V] ,");
        myFile.print("BL_Yall_Voltage[V] ,");
        myFile.print("BL_Hip_Voltage[V] ,");
        myFile.print("BL_Knee_Voltage[V] ,");
        myFile.print("BL_Wheel_Voltage[V] ,");
        myFile.print("BR_Switch_Voltage[V] ,");
        myFile.print("BR_Yall_Voltage[V] ,");
        myFile.print("BR_Hip_Voltage[V] ,");
        myFile.print("BR_Knee_Voltage[V] ,");
        myFile.print("BR_Wheel_Voltage[V] ,");
        myFile.println();
        Serial.println("Log Ready");
    }
    
    Serial.println("Waiting for m key input...");
    while (1){
        char input = 'i';
        if (Serial.available() > 0){
            input = Serial.read();
            if (input == 'm'){
                break;
            }
        }
    }//Scan start
    Serial.println("Scan start !");
    
    Timer.stop(); 
    Timer.setPeriod(SCAN_RATE); //SCAN_RATEごとに
    Timer.attachInterrupt(WheelMove); //割り込みさせる関数を指定
    Timer.start(); //割込み開始


    /* トルクoff */
    /*Serial.println("Press e to torque_disable");
    while (1){
        char input = 'i';
        if (Serial.available() > 0){
            input = Serial.read();
            if (input == 'e'){
                break;
            }
        }
    
    }
    disableAllTorque();

        Serial.println("All Motor Disabled !");*/
}   

void loop() {
    
}
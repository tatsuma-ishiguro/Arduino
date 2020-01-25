/* 車輪走行のデータ取得用プログラム
　(Dynamixel 5個で行うテスト用)
　 _秒待ってから設定速度(Valu)で_秒車輪走行
   減速開始から_秒後までデータ取得
 */
#include <MyRobot.h>
#include <SD.h>
#include <SPI.h>

#define BAUDRATE                        2000000
#define DEVICENAME                      "/dev/ttyACM0"

//加速
#define WHEEL_PROFILE_ACCELERATION 30
//減速（緊急停止時）
#define WHEEL_PROFILE_DECELERATION 100

#define SCAN_RATE 50000 //[us]

#define LOG_FILE  "test.txt"

//回転の速さ
double target_value = 1023;
double target_velocity = convertValue2Rpm(target_value);

void ReadData(int32_t *q_, int16_t *current_, uint16_t *voltage_);
void WriteData(int32_t *q_, int16_t *current_, uint16_t *voltage_);
void WriteInitialInfo(double target_velocity_);

bool isInitializeDone(void);
void setVelocityBasedProfile(void);

void initializeFL(void);
void initializeFR(void);
void initializeBL(void);
void initializeBR(void);
void enableFLTorque(void);
void enableFRTorque(void);
void enableBLTorque(void);
void enableBRTorque(void);

bool initializeFlag[5];

int32_t position_init[5]; //各関節の初期限界角度

//非常停止スイッチはNC
const int buttonPush = HIGH;
const int buttonNotPush = LOW;

const int buttonPin = 51; //ボタン読み取り設定をArduinoPin51番に(OpenCRではGPIO 2番)
int ledPin = BDPIN_LED_USER_1; //OpenCRのUSER1を非常停止時にON
int buttonState = 0; //ボタンの状態

//Instance
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
HardwareTimer Timer(TIMER_CH1);

//Data file settings
File myFile;
bool flag = false;
bool button = false;

double initialPose[5] = {
    0, 0, M_PI / 6, -M_PI/3, 0
};

//Time settings
double current_time = 0; //現在時間[ms]
double waiting_time = 3000; //走行開始時間[ms]
double endtime = 8000; //停止時間[ms]
double sampling_end = 12000; //記録時間[ms]
double stop_time = 0; //緊急停止時間記録用[ms]

//データ取得用配列
int32_t q[5];
int16_t current[5];
uint16_t voltage[5];

//SDカード内のファイルに最初にいろいろ記述する
void WriteInitialInfo(double target_velocity_){
  //「このファイルは屈伸させた時のデータ」というのを記載
  myFile.println("This file is the record in moving by wheel");

  //割り込み周期について
  myFile.print("The interval of this data is : ");
  myFile.println(SCAN_RATE);
  
  //設定した振幅を記入
  myFile.print("velocity[rpm] = ");
  myFile.println(target_velocity_);

  //
  myFile.print("Current time");
  myFile.print(" | ");
  myFile.print("position(or velocity)");
  myFile.print(" | ");
  myFile.print("current");
  myFile.print(" | ");
  myFile.print("voltage");
  myFile.println();
}


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

    buttonState = digitalRead(buttonPin); //ボタンの状態読み取り


    //3sec -> start 
    if(current_time > waiting_time && flag == false){
      dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FL_WHEEL_ID, ADDR_GOAL_VELOCITY, target_value, &dxl_error);
      //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FR_WHEEL_ID, ADDR_GOAL_VELOCITY, target_value, &dxl_error);
      //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BL_WHEEL_ID, ADDR_GOAL_VELOCITY, target_value, &dxl_error);
      //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BR_WHEEL_ID, ADDR_GOAL_VELOCITY, target_value, &dxl_error);
      flag = true;
    }

    //緊急停止
    if (buttonState == buttonPush && button == false){ //ボタンON
        //停止までの時間を短く
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FL_WHEEL_ID, ADDR_PROFILE_ACCELERATION, WHEEL_PROFILE_DECELERATION, &dxl_error);
        //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FR_WHEEL_ID, ADDR_PROFILE_ACCELERATION, WHEEL_PROFILE_DECELERATION, &dxl_error);
        //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BL_WHEEL_ID, ADDR_PROFILE_ACCELERATION, WHEEL_PROFILE_DECELERATION, &dxl_error);
        //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BR_WHEEL_ID, ADDR_PROFILE_ACCELERATION, WHEEL_PROFILE_DECELERATION, &dxl_error);

        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FL_WHEEL_ID, ADDR_GOAL_VELOCITY, 0, &dxl_error);
        //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FR_WHEEL_ID, ADDR_GOAL_VELOCITY, 0, &dxl_error);
        //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BL_WHEEL_ID, ADDR_GOAL_VELOCITY, 0, &dxl_error);
        //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BR_WHEEL_ID, ADDR_GOAL_VELOCITY, 0, &dxl_error);

        Serial.println("Emergency stop !!!");
        myFile.println("Emergency stop time !!!"); //緊急停止した時刻をSDカードに
        digitalWrite(ledPin, HIGH);
        stop_time = current_time; //緊急停止時間記録
    
        button = true;
    }
    
    //緊急停止から2秒後に記録終了
    if(button == true && current_time > stop_time + 2000){
        myFile.println("--- This experiment was finished by an emergency stop ---");
        myFile.close();
        Serial.println("Sampling has been finished !");
        Serial.end();
        Timer.stop();

        Timer.detachInterrupt();
    }

    //8sec -> stop 
    if(current_time > endtime){
        // Setting Target Velocity
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FL_WHEEL_ID, ADDR_GOAL_VELOCITY, 0, &dxl_error);
        //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, FR_WHEEL_ID, ADDR_GOAL_VELOCITY, 0, &dxl_error);
        //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BL_WHEEL_ID, ADDR_GOAL_VELOCITY, 0, &dxl_error);
        //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, BR_WHEEL_ID, ADDR_GOAL_VELOCITY, 0, &dxl_error);
    }

    //sampling_end -> sampling stop
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

//VelocityBased-Profileモード
void setVelocityBasedProfile(void){
    for(int i = 0; i < 5; i++){
        //DriveModeを読み込んで
        const uint8_t VelocityBased_Data = 0b11111011;
        uint8_t DriveMode_buffer;
        //現在のDriveModeを読み込む
        dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, i, ADDR_DRIVE_MODE, &DriveMode_buffer, &dxl_error);
        //Bit2を消す
        DriveMode_buffer = DriveMode_buffer & VelocityBased_Data;
        //新しいDriveModeを書き込む
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_DRIVE_MODE, DriveMode_buffer, &dxl_error);
    }
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

bool isInitializeDone(void){
    int sum = 0;
    for (int i = 0; i < 5; i++){
        sum += initializeFlag[i];
    }
    if (sum == 5){
        return true;
    }else{
        return false;
    }
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
    //SDカードの確認
    if(!SD.begin(cs)){
        Serial.println("No SD !");
        while(1);
    }
    Serial.println("SD Initialization Done!");
    
    /* オフセットファイルを読み込んで配列に格納 */
    myFile = SD.open(OFFSET_FILE, FILE_READ);

    Serial.println(myFile.size());
    if (myFile){
        for (int i = 0; i < 5;i++){
            offset[i] = myFile.parseInt();
            Serial.println(offset[i]);
        }
    }
    myFile.close();

    //Check File
    /*if(SD.exists(LOG_FILE)){
        Serial.println("The same name file has already exist !!");
        while(1);
    }*/

    //非常停止スイッチ
    pinMode(ledPin, OUTPUT);    //LED set      
    pinMode(buttonPin, INPUT_PULLUP); 

    initializeFL();
    initializeFR();
    initializeBL();
    initializeBR();

    for (int i = 0; i < 5;i++){
        initializeFlag[i] = false;
    }

    /* 限界角度での値を取得 */
    while (!isInitializeDone())  {
        Serial.println("Press enter the number : ");
        byte inputNum;

        //バッファをクリア
        while (Serial.available())    {
            Serial.read();
        }

        //モータ番号を受け取る
        while (1){
            if (Serial.available() > 0){
                inputNum = Serial.parseInt();
                break;
            }
        }

        //初期角度(限界角度)を計測
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, inputNum, ADDR_PRESENT_POSITION, (uint32_t *)&position_init[inputNum], &dxl_error);
        //LEDを消す
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, inputNum, ADDR_LED, LED_ON, &dxl_error);
        //オフセット完了の情報をかく
        initializeFlag[inputNum] = true;

        //今まで何番のモータのオフセットを取ったか確認
        Serial.print("You got position_limit of No. ");
        for (uint8_t i = 0; i < 5; i++){
            if (initializeFlag[i] == true){
                Serial.print(i);
                Serial.print(",");
            }
        }
        Serial.println();
    }

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

    enableFLTorque();
    enableFRTorque();
    enableBLTorque();
    enableBRTorque();

    /* 初期姿勢の目標角を設定 */
   for (int i = 0; i < 5; i++){
        if(i % 5 != 4){ //leg
            if(i/10 == 0){ //Forward
                dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDR_GOAL_POSITION, position_init[i] + offset[i] + convertAngle2Value(gearRatio[i%5] * initialPose[i % 5]), &dxl_error);
            }else{ //Backward
                dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDR_GOAL_POSITION, position_init[i] + offset[i] + convertAngle2Value(-gearRatio[i % 5] * initialPose[i % 5]), &dxl_error);
            }
        }
        else{ //Wheel
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDR_GOAL_VELOCITY, 0, &dxl_error);
        }
    }

    //非常停止スイッチを押していないか
    buttonState = digitalRead(buttonPin); //ボタンの状態読み取り

    if(buttonState == HIGH){
        Serial.println();
        Serial.println("Don't press the emergency button");
        Serial.println();
    }
    
    
    //motor data書き込み
    myFile = SD.open(LOG_FILE, FILE_WRITE); 
    
    Serial.println("File opened");
    myFile.println();
    
    delay(500);
    ReadInitialState(q, current, voltage);

    Serial.println();
    Serial.println("Waiting for m key input -> wheel moving");

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

}   

void loop() {    
}
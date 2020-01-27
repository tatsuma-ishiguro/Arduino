/* モックアップによる車輪走行のデータ取得用プログラム
   Time-basedにしてみた
　(Dynamixel 3個)
　 _秒待ってから設定速度(Valu)で_秒車輪走行
   減速開始から_秒後までデータ取得
 */
#include <XM540-W270-R.h>
#include <SD.h>
#include <SPI.h>
#include "math.h"
#include <DynamixelSDK.h>

#define SCAN_RATE 50000 //[us]

#define LOG_FILE  "test.txt"

//Protocol Version
#define PROTOCOL_VERSION            2.0

//Default Setting
#define DXL_HIP_ID                  1
#define DXL_KNEE_ID                 2
#define DXL_WHEEL_ID                3
#define BAUDRATE                        2000000
#define DEVICENAME                      "/dev/ttyACM0"

//Value Settings
#define TORQUE_ENABLE               1
#define TORQUE_DISABLE              0
#define LED_ON                      1
#define LED_OFF                     0
#define VELOCITY_LIMIT              1023
#define PWM_LIMIT                   885
#define MIN_POSITION_VALUE          0    //default
#define MAX_POSITION_VALUE          4095 //default

//Gain
#define HIP_POSITION_P_GAIN             2000 //0~16383
#define HIP_POSITION_I_GAIN             0
#define HIP_POSITION_D_GAIN             300
#define KNEE_POSITION_P_GAIN            1000 //0~16383
#define KNEE_POSITION_I_GAIN            0
#define KNEE_POSITION_D_GAIN            300
#define WHEEL_VELOCITY_P_GAIN           1000
#define WHEEL_VELOCITY_I_GAIN           3000

#define HIP_POSITION_P_GAIN_MOVING      10000 //0~16383
#define HIP_POSITION_I_GAIN_MOVING      0
#define HIP_POSITION_D_GAIN_MOVING      500
#define KNEE_POSITION_P_GAIN_MOVING     10000 //0~16383
#define KNEE_POSITION_I_GAIN_MOVING     0
#define KNEE_POSITION_D_GAIN_MOVING     500
#define WHEEL_VELOCITY_P_GAIN_MOVING    2000
#define WHEEL_VELOCITY_I_GAIN_MOVING    3000

//Operating Mode Setting
#define VELOCITY_CONTROL_MODE       1
#define POSITION_CONTROL_MODE       3   //0 ~ 4095

#define WHEEL_GEAR_RATIO            3.0

//Offset_Settings
int32_t hip_offset = -39;
int32_t knee_offset = 78;

void ReadData(int32_t *q_, int16_t *current_, uint16_t *voltage_);
void WriteData(int32_t *q_, int16_t *current_, uint16_t *voltage_);
void WriteInitialInfo(double target_velocity_);
void setProfileValue(uint8_t id, uint32_t ProfileVel, uint32_t ProfileAcc);

int32_t convertRpm2Value(double rpm);
double convertValue2Rpm(int32_t val);
int convertAngle2Value(double rad);

bool isInitializeDone(void);
void setTimeBasedProfile(void);

//回転の速さ
int32_t target_value = 50;
double target_velocity = convertValue2Rpm(target_value);


void initialize(void);
void enableTorque(void);

bool initializeFlag[5];

int32_t position_init[5]; //各関節の初期限界角度

//非常停止スイッチはNC
const int buttonPush = HIGH;
const int buttonNotPush = LOW;
const int buttonPin = 51; //ボタン読み取り設定をArduinoPin51番に(OpenCRではGPIO 2番)
int ledPin = BDPIN_LED_USER_1; //OpenCRのUSER1を非常停止時にON
int buttonState = 0; //ボタンの状態
bool button = false;

//Instance
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
HardwareTimer Timer(TIMER_CH1);

//Data file settings
File myFile;
bool flag = false;

//Target values
double q1_target = PI/6;
double q2_target = -PI/3;

//Time settings
double current_time = 0; //現在時間[ms]
double waiting_time = 2000; //走行開始時間[ms]
double endtime = 8000; //停止時間[ms]
double sampling_end = 12000; //記録時間[ms]
double stop_time = 0; //緊急停止時間記録用[ms]

//SPI Settings
const int mosi = 11;
const int miso = 12;
const int sck = 13;
const int cs = 10;

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
    for (int i = 1; i < 4; i++){//velocity & position
        if(i == 1 || i == 2){ //leg
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
    
    for (int i = 1; i < 4; i++){
        if(i == 1 || i == 2){ //leg
            myFile.print(q_[i]);
            myFile.print(", ");
        }
        else{ //Wheel
            myFile.print(q_[i]);
            myFile.print(", ");
        }
    }
    myFile.print(" | ");

    for (int i = 1; i < 4; i++){
        myFile.print(current_[i]);
        myFile.print(", ");
    }
    myFile.print(" | ");
    
    for (int i = 1; i < 4; i++){
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
        //ゲインをあげる
        setPIDgain(DXL_HIP_ID, HIP_POSITION_P_GAIN_MOVING, HIP_POSITION_I_GAIN_MOVING, HIP_POSITION_D_GAIN_MOVING);
        setPIDgain(DXL_KNEE_ID, KNEE_POSITION_P_GAIN_MOVING, KNEE_POSITION_I_GAIN_MOVING, KNEE_POSITION_D_GAIN_MOVING);
        setWheelPIgain(WHEEL_VELOCITY_P_GAIN_MOVING, WHEEL_VELOCITY_I_GAIN_MOVING);
        setProfileValue(4, 0, 1500); // Velocity Control Mode only uses Profile Acceleration
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 3, ADDR_GOAL_VELOCITY, target_value, &dxl_error);
        flag = true;
    }

    //緊急停止
    if (buttonState == buttonPush && button == false){ //ボタンON
        //停止までの時間を短く
        setProfileValue(4, 0, 500); //Velocity Control Mode only uses Profile Acceleration
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 3, ADDR_GOAL_VELOCITY, 0, &dxl_error);

        Serial.println("Emergency stop !!!");
        myFile.println("Emergency stop time !!!"); //緊急停止した時刻をSDカードに
        digitalWrite(ledPin, HIGH);
        stop_time = current_time; //緊急停止時間記録
    
        button = true;
    }
    
    //緊急停止から2秒後に記録終了
    if(button == true && current_time > stop_time + 2000){
        setPIDgain(DXL_HIP_ID, HIP_POSITION_P_GAIN, HIP_POSITION_I_GAIN, HIP_POSITION_D_GAIN);
        setPIDgain(DXL_KNEE_ID, KNEE_POSITION_P_GAIN, KNEE_POSITION_I_GAIN, KNEE_POSITION_D_GAIN);
        setWheelPIgain(WHEEL_VELOCITY_P_GAIN, WHEEL_VELOCITY_I_GAIN);
        myFile.println("--- This experiment was finished by an emergency stop ---");
        myFile.close();
        Serial.println("Sampling has been finished !");
        Serial.end();
        Timer.stop();

        Timer.detachInterrupt();
    }

    //8sec -> stop 
    if(current_time > endtime){
        setPIDgain(DXL_HIP_ID, HIP_POSITION_P_GAIN, HIP_POSITION_I_GAIN, HIP_POSITION_D_GAIN);
        setPIDgain(DXL_KNEE_ID, KNEE_POSITION_P_GAIN, KNEE_POSITION_I_GAIN, KNEE_POSITION_D_GAIN);
        setWheelPIgain(WHEEL_VELOCITY_P_GAIN, WHEEL_VELOCITY_I_GAIN);
        setProfileValue(4, 0, 1000); //Velocity Control Mode only uses Profile Acceleration
        // Setting Target Velocity
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 3, ADDR_GOAL_VELOCITY, 0, &dxl_error);
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
    for (int i = 1; i < 4; i++){//position & velocity
        if(i == 1 || i == 2){ //leg
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
    
    for (int i = 1; i < 4; i++){
        if(i == 1 || i == 2){ //leg
            myFile.print(q_[i]);
            myFile.print(", ");
        }
        else{ //Wheel
            myFile.print(q_[i]);
            myFile.print(", ");
        }
    }

    for (int i = 1; i < 4; i++){
        myFile.print(current_[i]);
        myFile.print(", ");
    }
    
    for (int i = 1; i < 4; i++){
        myFile.print(voltage_[i]);
        myFile.print(", ");
    }
    myFile.println();
}

/**   指定したモータをTimeBased-Controlモードに設定する   **/
void setTimeBasedProfile(void){
    for(int i = 1; i < 4; i++){
        //DriveModeを読み込んで
        const uint8_t TimeBased_Data = 0b00000100;
        uint8_t DriveMode_buffer;
        //現在のDriveModeを読み込む
        dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, i, ADDR_DRIVE_MODE, &DriveMode_buffer, &dxl_error);
        //Bit2を立てる
        DriveMode_buffer = DriveMode_buffer | TimeBased_Data;
        //新しいDriveModeを書き込む
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_DRIVE_MODE, DriveMode_buffer, &dxl_error);
    }
}

//Profile AccelerationとProfile Veloctyを設定する関数
void setProfileValue(uint8_t id, uint32_t ProfileVel, uint32_t ProfileAcc){
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDR_PROFILE_VELOCITY, ProfileVel, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDR_PROFILE_ACCELERATION, ProfileAcc, &dxl_error);
}

void setPIDgain(uint8_t id, uint16_t pgain, uint16_t igain, uint16_t dgain){
    // Setting Position P Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_POSITION_P_GAIN, pgain, &dxl_error);
    // Setting Position I Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_POSITION_I_GAIN, igain, &dxl_error);
    // Setting Position D Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_POSITION_D_GAIN, dgain, &dxl_error);
}

void setWheelPIgain(uint16_t pgain, uint16_t igain){
    // Setting Position P Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_WHEEL_ID, ADDR_VELOCITY_P_GAIN, pgain, &dxl_error);
    // Setting Position I Gain
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_WHEEL_ID, ADDR_VELOCITY_I_GAIN, igain, &dxl_error);
}

void initialize(void){
    //initialize Hip motor
    //Setting Operating Mode to Position-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_HIP_ID, ADDR_OPERATING_MODE, POSITION_CONTROL_MODE, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_HIP_ID, ADDR_LED, LED_ON, &dxl_error);
    //GAIN
    setPIDgain(DXL_HIP_ID, HIP_POSITION_P_GAIN, HIP_POSITION_I_GAIN, HIP_POSITION_D_GAIN);
    
    //initialize Knee motor
    //Setting Operating Mode to Position-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_KNEE_ID, ADDR_OPERATING_MODE, POSITION_CONTROL_MODE, &dxl_error);
    //change the direction of Knee Motor
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_KNEE_ID, ADDR_DRIVE_MODE, 1, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_KNEE_ID, ADDR_LED, LED_ON, &dxl_error);
    //GAIN
    setPIDgain(DXL_KNEE_ID, KNEE_POSITION_P_GAIN, KNEE_POSITION_I_GAIN, KNEE_POSITION_D_GAIN);

    //initialize Wheel motor
    //Setting Operating Mode to Velocity-Control-Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_WHEEL_ID, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE, &dxl_error);
    // LED
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_WHEEL_ID, ADDR_LED, LED_ON, &dxl_error);
    // Setting Velocity Limit
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_WHEEL_ID, ADDR_VELOCITY_LIMIT, VELOCITY_LIMIT, &dxl_error);
    //GAIN
    setWheelPIgain(WHEEL_VELOCITY_P_GAIN, WHEEL_VELOCITY_I_GAIN);

    //Offset
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_HIP_ID, ADDR_HOMING_OFFSET, hip_offset, &dxl_error);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_KNEE_ID, ADDR_HOMING_OFFSET, knee_offset, &dxl_error);
}


void enableTorque(void){
    // Enable Hip Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_HIP_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // Enable Knee Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_KNEE_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    // Enable Wheel Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_WHEEL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

    setProfileValue(DXL_HIP_ID, 5000, 2000); //Profile_Velocity:5000 Profile_Acceleration:2000
    setProfileValue(DXL_KNEE_ID, 5000, 2000); //Profile_Velocity:5000 Profile_Acceleration:2000
    setProfileValue(DXL_WHEEL_ID, 0, 1000); //Profile_Velocity:5000 Profile_Acceleration:2000
    //Set Hip initial Position
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_HIP_ID, ADDR_GOAL_POSITION, convertAngle2Value(q1_target), &dxl_error);
    //Set Knee initial Position
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_KNEE_ID, ADDR_GOAL_POSITION, convertAngle2Value(q2_target), &dxl_error);
    // Set Wheel initial Target Velocity
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_WHEEL_ID, ADDR_GOAL_VELOCITY, 0, &dxl_error);
}

//Velocity -> Value
int32_t convertRpm2Value(double rpm){
    return (uint32_t)((rpm / (VELOCITY_COEFFICIENT*WHEEL_GEAR_RATIO))+0.5);
}
//Value -> Velocity
double convertValue2Rpm(int32_t val){
    return val * VELOCITY_COEFFICIENT * WHEEL_GEAR_RATIO;
}

//Angle->Value
int convertAngle2Value(double rad){
    return 4095 * (rad + PI) / (2 * PI);
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

    //Check File
    if(SD.exists(LOG_FILE)){
        Serial.println("The same name file has already exist !!");
        while(1);
    }

    //非常停止スイッチ
    pinMode(ledPin, OUTPUT);    //LED set      
    pinMode(buttonPin, INPUT_PULLUP); 

    setTimeBasedProfile();
    initialize();


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

    //enableTorque & changeInitialPosition
    enableTorque();

    //非常停止スイッチを押していないか
    buttonState = digitalRead(buttonPin); //ボタンの状態読み取り

    if(buttonState == buttonPush){
        Serial.println();
        Serial.println("Don't press the emergency button");
        Serial.println();
    }
    
    
    //motor data書き込み
    myFile = SD.open(LOG_FILE, FILE_WRITE); 
    
    Serial.println("File opened");
    myFile.println();
    
    delay(500);
    WriteInitialInfo(target_velocity);
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
/* 横移動のプログラム（簡易）
   機体の向きを変えずに左右移動
 */
#include <tatsuMyRobot.h>
#include <SD.h>
#include <SPI.h>
#include <math.h>

#define BAUDRATE                        2000000
#define DEVICENAME                      "/dev/ttyACM0"

#define SCAN_RATE 50000 //[us]

#define LOG_FILE  "_001.txt"

#define DIRECTION_OF_MOVEMENT   1 //右移動=1,左移動=-1

//Wheel
int32_t target_value = 33; //車輪の回転速度
double target_velocity = convertValue2mpers(target_value);


//Yaw
void AngleofYaw(int32_t radius);
//Velocity of each motor
void VelocityWheel(int32_t radius,int32_t velocity);

void ReadData(int32_t *q_, int16_t *current_, uint16_t *voltage_);
void WriteData(int32_t *q_, int16_t *current_, uint16_t *voltage_);
void WriteInitialInfo(double target_velocity_);
void setProfileValue(uint8_t id, uint32_t ProfileVel, uint32_t ProfileAcc);

bool isInitializeDone(void);
void setTimeBasedProfile(void);

void initialize(void);
void enableTorque(void);

bool initializeFlag[NUMJOINTS];
　
int32_t position_init[NUMJOINTS]; //各関節の初期限界角度

//非常停止スイッチはNC
const int buttonPush = HIGH;
const int buttonNotPush = LOW;
const int buttonPin = 51; //ボタン読み取り設定をOpenCRのGPIO 2番pinに(ArduinoPin51番)
int buttonState = 0; //ボタンの状態
bool button = false;

//Instance
dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;
HardwareTimer Timer(TIMER_CH1);

//Data file settings
File myFile;
bool flag = false;
bool flag_stop = false;



//Time settings
double current_time = 0; //現在時間[ms]
double waiting_time = 3000; //走行開始時間[ms]
double endtime = 9000; //停止時間[ms]
double sampling_end = 12000; //記録時間[ms]
double stop_time = 0; //緊急停止時間記録用[ms]

//データ取得用配列
int32_t q[NUMJOINTS];
int16_t current[NUMJOINTS];
uint16_t voltage[NUMJOINTS];

//SDカード内のファイルに最初にいろいろ記述する
void WriteInitialInfo(double target_velocity_, double turning_radius_){
  //「このファイルは屈伸させた時のデータ」というのを記載
  myFile.println("This file is the record in turning");

  //割り込み周期について
  myFile.print("The interval of this data is : ");
  myFile.println(SCAN_RATE);
  
  //設定した振幅を記入
  myFile.print("velocity[m/s] = ");
  myFile.println(target_velocity_);
  myFile.print("turning_radius[m] = ");
  myFile.println(turning_radius_);  

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


//横移動モード
void AngleofYaw(void){
    for(int i = 0; i < NUMJOINTS; i++){
        if(i == 1 || i == 16){//CW(負回転)
            setProfileValue(i, 3000, 1000); //(ID,Profile_Velocity,Profile_Acceleration)
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDR_GOAL_POSITION, position_init[i] + offset[i] + convertAngle2Value(gearRatio[i % 5] * initialPose[i % 5]) + convertAngle2Value(gearRatio[i % 5] * (-M_PI / 2)), &dxl_error);
        }
        if(i == 6 || i == 11){//CCW(正回転)
            setProfileValue(i, 3000, 1000); //(ID,Profile_Velocity,Profile_Acceleration)
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDR_GOAL_POSITION, position_init[i] + offset[i] + convertAngle2Value(gearRatio[i % 5] * initialPose[i % 5]) + convertAngle2Value(gearRatio[i % 5] * (M_PI / 2)), &dxl_error);
        }
    }
}

//移動速度
void VelocityWheel(int32_t velocity){
    for(int i = 0; i < NUMJOINTS; i++){
        if(i == 4 || i == 19){//逆回転
            setProfileValue(i, 0, 3000); // Velocity Control Mode only uses Profile Acceleration
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDR_GOAL_VELOCITY, DIRECTION_OF_MOVEMENT * (-1) * velocity, &dxl_error);
        }
        if(i == 9 || i == 14){//正回転
            setProfileValue(i, 0, 3000); // Velocity Control Mode only uses Profile Acceleration
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDR_GOAL_VELOCITY, DIRECTION_OF_MOVEMENT * velocity, &dxl_error);
        }
    }
}



void ReadData(int32_t *q_, int16_t *current_, uint16_t *voltage_){
    //Read Data
    for (int i = 0; i < NUMJOINTS; i++){//position & velocity
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
    
    for (int i = 0; i < NUMJOINTS; i++){
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

    for (int i = 0; i < NUMJOINTS; i++){
        myFile.print(current_[i]);
        myFile.print(", ");
    }
    myFile.print(" | ");
    
    for (int i = 0; i < NUMJOINTS; i++){
        myFile.print(voltage_[i]);
        myFile.print(", ");
    }
    myFile.print(" | ");

    myFile.println();
}

void WheelMoveturning(void){
    ReadData(q, current, voltage);
    WriteData(q, current, voltage);
    current_time += SCAN_RATE /1000;

    Serial.println(current_time);

    buttonState = digitalRead(buttonPin); //ボタンの状態読み取り

    //waiting_time -> start 
    if(current_time > waiting_time && flag == false){
        VelocityWheel(target_value);
        flag = true;
    }

    //緊急停止
    if (buttonState == buttonPush && button == false){ //ボタンON
        for(int i = 0; i < NUMJOINTS; i++){
            if(i % 5 == 4){
                //停止までの時間を短く
                setProfileValue(i, 0, 1500); //Velocity Control Mode only uses Profile Acceleration
                dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDR_GOAL_VELOCITY, 0, &dxl_error);
            }
        }

        Serial.println("Emergency stop !!!");
        myFile.println("Emergency stop time !!!"); //緊急停止した時刻をSDカードに
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

    //endtime -> stop 
    if(current_time > endtime && flag_stop == false){
        for(int i = 0; i < NUMJOINTS; i++){
            if(i % 5 == 4){
                //停止までの時間を短く
                setProfileValue(i, 0, 3000); //Velocity Control Mode only uses Profile Acceleration
                dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDR_GOAL_VELOCITY, 0, &dxl_error);
            }
        }
        flag_stop = true;
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
    for (int i = 0; i < NUMJOINTS; i++){//position & velocity
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
    
    for (int i = 0; i < NUMJOINTS; i++){
        if(i % 5 != 4){ //leg
            myFile.print(q_[i]);
            myFile.print(", ");
        }
        else{ //Wheel
            myFile.print(q_[i]);
            myFile.print(", ");
        }
    }

    for (int i = 0; i < NUMJOINTS; i++){
        myFile.print(current_[i]);
        myFile.print(", ");
    }
    
    for (int i = 0; i < NUMJOINTS; i++){
        myFile.print(voltage_[i]);
        myFile.print(", ");
    }
    myFile.println();
}

/**   指定したモータをTimeBased-Controlモードに設定する   **/
void setTimeBasedProfile(void){
    for(int i = 0; i < NUMJOINTS; i++){
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

void initialize(void){
    for (int i = 0; i < NUMJOINTS; i++){
        if(i % 5 != 4){ //leg
            //Setting Operating Mode to Position-Control-Mode
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_OPERATING_MODE, EXTENDTED_POSITION_CONTROL_MODE, &dxl_error);
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDR_MAX_POSITION_LIMIT, MAX_POSITION_VALUE, &dxl_error);
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDR_MIN_POSITION_LIMIT, MIN_POSITION_VALUE, &dxl_error);
            // Setting Position P Gain
            dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, i, ADDR_POSITION_P_GAIN, SWITCH_POSITION_P_GAIN, &dxl_error);
            // Setting Position I Gain
            dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, i, ADDR_POSITION_I_GAIN, SWITCH_POSITION_I_GAIN, &dxl_error);
            // Setting Position D Gain
            dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, i, ADDR_POSITION_D_GAIN, SWITCH_POSITION_D_GAIN, &dxl_error);
            // LED
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_LED, LED_ON, &dxl_error);
        }
        else{ //Wheel
            //Setting Operating Mode to Velocity-Control-Mode
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE, &dxl_error);
            // Setting Position P Gain
            dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, i, ADDR_VELOCITY_P_GAIN, WHEEL_VELOCITY_P_GAIN, &dxl_error);
            // Setting Position I Gain
            dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, i, ADDR_VELOCITY_I_GAIN, WHEEL_VELOCITY_I_GAIN, &dxl_error);
            // LED
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_LED, LED_ON, &dxl_error);
            // Setting Velocity Limit
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDR_VELOCITY_LIMIT, VELOCITY_LIMIT, &dxl_error);
        }
    }
}

void enableTorque(void){
    for (int i = 0; i < NUMJOINTS; i++){
        if(i % 5 != 4){ //leg
            // Enable FL Switch Dynamixel Torque
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        }
        else{ //Wheel
            // Enable FL Wheel Dynamixel Torque
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
            // Setting Target Velocity
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDR_GOAL_VELOCITY, 0, &dxl_error);
        }
    }
}

bool isInitializeDone(void){
    int sum = 0;
    for (int i = 0; i < NUMJOINTS; i++){
        sum += initializeFlag[i];
    }
    if (sum == NUMJOINTS){
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

    //Open Port & Set port baudrate
    portHandler->openPort();
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

    if (myFile){
        for (int i = 0; i < NUMJOINTS;i++){
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
    pinMode(buttonPin, INPUT_PULLUP);     

    setTimeBasedProfile();
    initialize();

    for (int i = 0; i < NUMJOINTS;i++){
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
        for (uint8_t i = 0; i < NUMJOINTS; i++){
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

    enableTorque();

    /* 初期姿勢へ */
    for (int i = 0; i < NUMJOINTS; i++){
        setProfileValue(i, 3000, 1000); //(ID,Profile_Velocity,Profile_Acceleration)
        if(i % 5 != 4){ //leg
            if(i/10 == 0){ //Forward
                dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, i, ADDR_GOAL_POSITION, position_init[i] + offset[i] + convertAngle2Value(gearRatio[i % 5] * initialPose[i % 5]), &dxl_error);
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
    if(buttonState == buttonPush){ //押していたら警告
        Serial.println();
        Serial.println("Don't press the emergency button");
        Serial.println();
    }


    //motor data書き込み
    myFile = SD.open(LOG_FILE, FILE_WRITE); 
    
    Serial.println("LogFile opened");
    myFile.println();
    
    delay(500);
    WriteInitialInfo(target_velocity);
    ReadInitialState(q, current, voltage);

    Serial.println();
    Serial.println("Waiting for t key input -> Yaw axis rotation for Turning");
 
    while (1){
        char input = 'i';
        if (Serial.available() > 0){
            input = Serial.read();
            if (input == 't'){
                break;
            }
        }
    }//Yaw axis rotation
    Serial.println("Yaw axis rotation !");

    /* Yaw軸を回転 */
    AngleofYaw();




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
    Timer.attachInterrupt(WheelMoveturning); //割り込みさせる関数を指定
    Timer.start(); //割込み開始

}   

void loop() {
    
}





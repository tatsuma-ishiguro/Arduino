/*非常停止スイッチを使ってOpenCRのLEDを消す
　ボタンOFFではLED点灯 -> ボタンを押すと消灯
 */

/*
#define BDPIN_LED_USER_1        22
*/

const int buttonPin = 2;
int ledPin = BDPIN_LED_USER_1;
int buttonState = 0;


const int buttonON = LOW;    // ボタンが押されているとピンの値はLOW
const int buttonOFF = HIGH;

void setup() {
    Serial.begin(2000000);
    while(!Serial);
    Serial.println("start...");

    Serial.println("Press s -> USER1 LED ON");
    while(1){
        char input = 'i';
        if (Serial.available() > 0){
            input = Serial.read();
            if (input == 's'){
                break;
            }
        }       
    }

    pinMode(buttonPin, INPUT_PULLUP); // Inputモードでプルアップ抵抗を有効に
    pinMode(BDPIN_LED_USER_1, OUTPUT); //USER1 LED
    
    while(1){
        buttonState = digitalRead(buttonPin);
        if(buttonState == buttonOFF){
            digitalWrite(ledPin, HIGH);
        }
        else if(buttonState == buttonON){     // ボタンが押されていたらLEDをOFF
            digitalWrite(ledPin, LOW);
            Serial.println("LED OFF");
            break;
        }
    }
}

void loop(){

}

const int buttonON = LOW;    // ボタンが押されているとピンの値はLOW
const int buttonOFF = HIGH;  // ボタンが押されていないとピンの値はHIGH
 
const int buttonPin = 2;
int ledPin = BDPIN_LED_USER_1;
int buttonState = 0;
 
void setup() {
  pinMode(ledPin, OUTPUT);      
  pinMode(buttonPin, INPUT_PULLUP);     
}
 
void loop(){
  buttonState = digitalRead(buttonPin);
  if (buttonState == buttonOFF) {     // ボタンが押されていないなら
    digitalWrite(ledPin, HIGH);  
  } 
  else {
    digitalWrite(ledPin, LOW); 
  }
}
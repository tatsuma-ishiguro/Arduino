
const int buttonON = HIGH;    // ボタンが押されているとピンの値はLOW
const int buttonOFF = LOW;  // ボタンが押されていないとピンの値はHIGH
 
const int buttonPin = 2;
int ledPin = 13;
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
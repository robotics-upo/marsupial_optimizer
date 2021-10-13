//Initializing LED Pin
int led_pin = 6;
void setup() {
  //Declaring LED pin as output
  Serial.begin(9600);
  pinMode(led_pin, OUTPUT);
}
void loop() {
  //Fading the LED
  Serial.println("First Loop");

     analogWrite(led_pin, 127);
      delay(30000);

      
  for(int i=128; i<255; i++){
    analogWrite(led_pin, i);
    delay(100);
    Serial.print("value i= ");
    Serial.print(i);
    Serial.println();
    if (i == 254){
      analogWrite(led_pin, i);
      delay(5000);
    }
  }
  

  Serial.println("Forth Loop");
  for(int i=0; i<128; i++){
    analogWrite(led_pin, i);
    delay(100);
    Serial.print("value i= ");
    Serial.print(i);
    Serial.println();
    if (i == 127){
      analogWrite(led_pin, i);
      delay(5000);
    }
  }
}

#define JoyX A0
#define JoyY A1
#define JoyXzero 453
#define JoyYzero 463

#define captureFrequecy 500
#define Xsen 0.001
#define Ysen 0.001

void JoySetup(){
    //pinMode(LEDOnBoard, OUTPUT);
    pinMode(JoyX, INPUT);
    pinMode(JoyY, INPUT);
    }

int ReadJoyX(){

    return analogRead(JoyX);
}
int ReadJoyY(){

    return analogRead(JoyY);
}

float Xpos = 0;
float Ypos = 0;

void setup() {
  // put your setup code here, to run once:
  JoySetup();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  char Output[150];
  //sprintf(Output, "JOyX = %d\tJoyY = %d\n", ReadJoyX(), ReadJoyY());
  //Serial.print(Output);
    //delay(100);
  Xpos = Xpos + (ReadJoyX() - JoyXzero) * Xsen;
  Ypos = Ypos + (ReadJoyY() - JoyYzero) * Ysen;
  //sprintf(Output, "JOyX = %f\tJoyY = %f\n", Xpos,Ypos);
  //Serial.print(Output);
  Serial.print(Xpos, 3);
  Serial.print("\t");
  Serial.print(Ypos,3);
  Serial.print("\n");
  delay(1/captureFrequecy*1000);
   
}

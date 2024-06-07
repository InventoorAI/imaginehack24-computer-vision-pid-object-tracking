/*    
    PORTA Register
    ---------------------------
    (MSB) 0 0 0 0 0 0 0 0 (LSB)
    ---------------------------
    Byte 0: M1 (Pin 22)
    Byte 1: M2 (Pin 23)
    Byte 2: M3 (Pin 24)
    Byte 3: M4 (Pin 25)
    Byte 4: Unused
    Byte 5: Unused
    Byte 6: Unused
    Byte 7: Unused

    -------- ------------
   | Port A |  Register  |
    -------- ------------
   | Pin 22 |     PA0    |
   | Pin 23 |     PA1    |
   | Pin 24 |     PA2    |
   | Pin 25 |     PA3    |
   | Pin 26 |     PA4    |
   | Pin 27 |     PA5    |
   | Pin 28 |     PA6    |
   | Pin 29 |     PA7    |
    -------- ------------

    Copyright© Tung Tze Yang 2024
    For : IMAGINEHACK@Taylor's University 2024
    Edited : 8/6/2024 Tung Tze Yang 2024
*/
#define M1_PWM 6
#define M2_PWM 7
#define M3_PWM 8
#define M4_PWM 9

// defines ultrasnic pins numbers
const int trigPin = 46;
const int echoPin = 47;

// defines variables
long duration;
int distance;

double dt, timeOld = 0;
double integral = 0.00, previous = 0.00;
int error, xPos, basespeed;
double control;


int serial_retrieve(){
    if (Serial2.available() > 0){
    String input = Serial2.readStringUntil('\n');  // Read the string until a newline character is encountered
    Serial.println("Received: " + input);

    // Parse the string to extract X and Y values
    int spaceIndex = input.indexOf(' ');
    if (spaceIndex != -1) {
      // Extract X and Y substrings
      String xString = input.substring(0, spaceIndex);
      String yString = input.substring(spaceIndex + 1);

      // Convert X and Y substrings to integers
      int xPos = xString.substring(xString.indexOf('X') + 1).toInt();
      int yPos = yString.substring(yString.indexOf('Y') + 1).toInt();

      Serial.print("xPos: ");
      Serial.println(xPos);
      Serial.print("yPos: ");
      Serial.println(yPos);
      return xPos;
    }
  }
}

double pid(double error, double kp, double ki, double kd)
{
  double propotional = error;
  integral += error * dt;

  integral = constrain(integral, -200, 200);

  double derivative = (error - previous) / dt;
  previous = error;
  double output = (kp*propotional) + (ki*integral) + (kd*derivative);
  
  return output;  
}

void motorControl(double control){
  if (control > 0){
    PORTA = B00000000;
    //turn right, clockwise

  }else if (control < 0){
    PORTA = B00001111;
    //turn left, anticlockwise
  }else {
    PORTA = B00000110;

  }
  control = fabs(control);
  int control_int = (int) control;
  analogWrite(M1_PWM, (control_int + basespeed));
  analogWrite(M2_PWM, (control_int + basespeed));
  analogWrite(M3_PWM, (control_int + basespeed));
  analogWrite(M4_PWM, (control_int + basespeed));

}

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  
  Serial.begin(115200); //for monitoring/debugging
  Serial2.begin(115200); //UART communication with raspberrypi 
  Serial.println("Serial Initialized");

    //Timer 2: Pins 9, 10
  TCCR2B &= ~ _BV (CS22); 
  TCCR2B |= _BV (CS20);
  
  //Timer 4: Pins 6, 7 and 8
  TCCR4B &= ~(_BV(CS42) | _BV(CS41) | _BV(CS40));
  TCCR4B |= _BV(CS40);
  
  DDRA = B00001111;
  PORTA = B00000000;
}

void loop() {

  double now = millis();
  dt = (now -timeOld)/1000.00;
  timeOld =  now;

  xPos = serial_retrieve();
  error = -xPos;
  
  if(abs(error) <= 10){
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;

    basespeed = 50;

    if (distance <= 20){
       basespeed = map(distance, 0, 20, 0,50);
       Serial.println(basespeed);
    }

    control = 0;
    motorControl(control);
//    Serial.print("Control: ");
//    Serial.println(control);

  }
  else {
    basespeed = 0;
    control = pid(error, 0.2,0.1,0.1);
    motorControl(control);
    Serial.print("Control: ");
    Serial.println(control);
  }

  Serial.print("Distance: ");
  Serial.println(distance);

}

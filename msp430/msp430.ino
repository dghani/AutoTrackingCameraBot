/*
    B EE 526 A
    3/13/2019 
    
    Group 12
    Daniel Mamaghani
    Jean Milien
    
 */

int sensorA = A4; // Sharp IR GP2Y0A41SK0F pins  Pin P1.3 
int sensorB = A3; // PIN 1.4
// Motor Pins 
//our L298N control pins
int LeftMotorForward   = 13;  // Pin 2.5 to IN1 
int LeftMotorBackward  = 12;  // Pin 2.4 to IN2 
int RightMotorForward  = 7;   // Pin 1.5 to IN3 
int RightMotorBackward = 8;   // Pin 2.0 to IN4 

int LeftMotorPWMPin  = P1_6;
int RightMotorPWMPin = P2_6; // Previously P1_2

// Default speeds for going forward/backwards and for turning.
int DEFAULT_MOTOR_SPEED = 200;
int DEFAULT_TURN_SPEED = 150;

unsigned long lastUartReceivedPacketMillis = 0;
const unsigned long UART_CONTROL_TIMEOUT_MILLIS = 800;

// Left motor: 
//          OUT 3 - White ,Out 4 - RED -
// Right motor: 
//          OUT 2 - White, OUT 1 - RED


void setSpeedToDefault()
{
    analogWrite(LeftMotorPWMPin, DEFAULT_MOTOR_SPEED);
    analogWrite(RightMotorPWMPin, DEFAULT_MOTOR_SPEED);
}

void moveStop(){
  
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
  
  analogWrite(LeftMotorPWMPin, 0);
  analogWrite(RightMotorPWMPin, 0);
}

void moveForward(){

    setSpeedToDefault();
    
    digitalWrite(LeftMotorForward, LOW);
    digitalWrite(RightMotorForward, LOW);
  
    digitalWrite(LeftMotorBackward, HIGH);
    digitalWrite(RightMotorBackward, HIGH); 
  
}

void moveForward(int duration_millis)
{
    moveForward();
    delay(duration_millis);
    moveStop();
}

void moveBackward(){

  setSpeedToDefault();

  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  
}

void moveBackward(int duration_millis)
{
    moveBackward();
    delay(duration_millis);
    moveStop();
}

void turnRight(int duration_millis, int turn_pwm)
{
  analogWrite(LeftMotorPWMPin, turn_pwm);
  analogWrite(RightMotorPWMPin, turn_pwm);
  
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);
  
  delay(duration_millis);
  
  moveStop();
}


void turnRight(int duration_millis)
{
    turnRight(duration_millis, DEFAULT_TURN_SPEED);
}

void turnRight()
{
    turnRight(500, DEFAULT_TURN_SPEED);
}


void turnLeft(int duration_millis, int turn_pwm)
{
  analogWrite(LeftMotorPWMPin, turn_pwm);
  analogWrite(RightMotorPWMPin, turn_pwm);
  
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  
  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  
  delay(duration_millis);
  
  moveStop();
}

void turnLeft(int duration_millis)
{
    turnLeft(duration_millis, DEFAULT_TURN_SPEED);
}

void turnLeft()
{
    turnLeft(500, DEFAULT_TURN_SPEED);
}

void forwardBackwardLeftRight()
{
    moveForward();
    delay(500);
    moveStop();

    delay(2500);
    
    moveBackward();
    delay(500);
    moveStop();

    delay(2500);

    turnLeft(500);

    delay(2500);

    turnRight(500);

    delay(2500);
}

//
//
void setMotorSpeeds(const uint8_t left_motor_forward,
          const uint8_t left_motor_backward,
          const uint8_t right_motor_forward,
          const uint8_t right_motor_backward,
          const uint8_t left_motor_speed,
          const uint8_t right_motor_speed)
{
  // Sample code to show that this function gets called.
  
  // DEBUG code
//  Serial.println("setMotorSpeeds");
//  if ( left_motor_speed  >= 128 ) { digitalWrite(RED_LED,   HIGH); }
//  if ( right_motor_speed >= 128 ) { digitalWrite(GREEN_LED, HIGH); }
//  delay(10);
//  digitalWrite(RED_LED, LOW); 
//  digitalWrite(GREEN_LED, LOW);

  digitalWrite(LeftMotorForward,   left_motor_forward);
  digitalWrite(LeftMotorBackward,  left_motor_backward);
  digitalWrite(RightMotorForward,  right_motor_forward);
  digitalWrite(RightMotorBackward, right_motor_backward); 
  analogWrite(LeftMotorPWMPin, left_motor_speed); 
  analogWrite(RightMotorPWMPin,right_motor_speed);
}

void dropUARTPacket()
{
    const int PACKET_SIZE_BYTES = 6;
    for (int i = 0; i < PACKET_SIZE_BYTES; i++)
    {
      Serial.read();
    }
}

// Returns true if we read an update from the UART.
bool updateMotorsFromUART(uint8_t& left_motor_forward,
                          uint8_t& left_motor_backward,
                          uint8_t& right_motor_forward,
                          uint8_t& right_motor_backward,
                          uint8_t& left_motor_speed,
                          uint8_t& right_motor_speed)
{
  // DEBUG code
  //Serial.println("updateMotorsFromUART");
  int bytes_available = Serial.available();
  if (bytes_available >= 18) // There's too much data accumulating
  {
      dropUARTPacket();
  }
  if (abs(bytes_available) >= 6)
  {
    // DEBUG code
    //Serial.println("Serial.available() >= 6");
    left_motor_forward    = (uint8_t)Serial.read();
    left_motor_backward   = (uint8_t)Serial.read();
    right_motor_forward   = (uint8_t)Serial.read();
    right_motor_backward  = (uint8_t)Serial.read();
    left_motor_speed  = (uint8_t)Serial.read();
    right_motor_speed = (uint8_t)Serial.read();
    return true;
  }
  return false;
}

float volts1MovingAverage(float volts1)
{
    static const int MAX_SAMPLES = 10;
    static float samples[MAX_SAMPLES] = {1};
    static int cur_sample_index = 0;
    
    samples[cur_sample_index] = volts1;
    cur_sample_index = (cur_sample_index + 1) % MAX_SAMPLES;
    
    float sum = 0;
    for (int i = 0; i < MAX_SAMPLES; i++)
    {
      sum += samples[i];
    }
    
    return (sum / (float)MAX_SAMPLES);
}
  
float volts2MovingAverage(float volts2)
{
    static const int MAX_SAMPLES = 10;
    static float samples[MAX_SAMPLES] = {1};
    static int cur_sample_index = 0;
    
    samples[cur_sample_index] = volts2;
    cur_sample_index = (cur_sample_index + 1) % MAX_SAMPLES;

    float sum = 0;
    for (int i = 0; i < MAX_SAMPLES; i++)
    {
      sum += samples[i];
    }
    
    return (sum / (float)MAX_SAMPLES);
}

void avoidObstacles(const unsigned long UPDATE_INTERVAL_MILLISECONDS)
{
    const int IR_LOW_PASS_FILTER_THRESHOLD = 20;
    const float IR_BACKGROUND_NOISE_THRESHOLD = 1.0;
    const float THRESHOLD_RATIO = 1.30;
    
    float volts1 = analogRead(sensorA)*0.0048828125;  // value from sensor
    float volts2 = analogRead(sensorB)*0.0048828125; // value from sensor 

    if (volts1 > IR_LOW_PASS_FILTER_THRESHOLD)
    {
      volts1 = 0;
    }
    
    if (volts2 > IR_LOW_PASS_FILTER_THRESHOLD)
    {
      volts2 = 0;
    }

    float volts1trigger = max(THRESHOLD_RATIO * volts1MovingAverage(volts1), IR_BACKGROUND_NOISE_THRESHOLD);
    float volts2trigger = max(THRESHOLD_RATIO * volts2MovingAverage(volts2), IR_BACKGROUND_NOISE_THRESHOLD);
    
    if (volts1 > volts1trigger && volts2 > volts2trigger)
    {
        // Include a bias so that we do not repeatedly alternate between left and right turns due to noise.
        static const float BIAS_TOWARD_RIGHT = 0.5;
        if (volts1 + BIAS_TOWARD_RIGHT > volts2) // if the obstacles is closer on the left side
        {
            //moveBackward(UPDATE_INTERVAL_MILLISECONDS);
            turnRight(UPDATE_INTERVAL_MILLISECONDS);
        }
        else
        {
            //moveBackward(UPDATE_INTERVAL_MILLISECONDS);
            turnLeft(UPDATE_INTERVAL_MILLISECONDS);
        }
    }
    else if (volts1 > volts1trigger && volts1 > 1.25 * volts2)
    {
        // if we see an obstacle on the robot's left side, turn right
        turnRight(UPDATE_INTERVAL_MILLISECONDS);
    }
    else if (volts2 > volts2trigger && volts2 > 1.25 * volts1)
    {
        turnLeft(UPDATE_INTERVAL_MILLISECONDS);
    }
}



void periodicallyAvoidObstacles(const unsigned long UPDATE_INTERVAL_MILLISECONDS)
{
  // DEBUG code
  //Serial.println("periodicallyAvoidObstacles");
  static unsigned long lastUpdateTime = 0;
  static unsigned long currentTime = millis();
  

  // Read in the current time.
  currentTime = millis();
  
  // If enough time has passed, check for input from the raspberry pi.
  // If we are still 
  if ( (currentTime - lastUpdateTime) > UPDATE_INTERVAL_MILLISECONDS
      && (currentTime - lastUartReceivedPacketMillis) > UART_CONTROL_TIMEOUT_MILLIS )
  {
    lastUpdateTime = currentTime;

    avoidObstacles(UPDATE_INTERVAL_MILLISECONDS);
  }
}

void periodicallyUpdateMotorsFromUART(const unsigned long UPDATE_INTERVAL_MILLISECONDS)
{
  // DEBUG code
  //Serial.println("periodicallyUpdateMotorsFromUART");
  static unsigned long lastUpdateTime = 0;
  static unsigned long currentTime = millis();
  
  
  // this is where the UART motor settings are stored.
  
  static uint8_t left_motor_forward   = 0;
  static uint8_t left_motor_backward  = 0;
  static uint8_t right_motor_forward  = 0;
  static uint8_t right_motor_backward = 0;
  static uint8_t left_motor_speed  = 0;
  static uint8_t right_motor_speed = 0;

  // Read in the current time.
  currentTime = millis();
  
  // If enough time has passed, check for input from the raspberry pi.
  if ( (currentTime - lastUpdateTime) > UPDATE_INTERVAL_MILLISECONDS )
  {
    lastUpdateTime = currentTime;

    // Read the new motor speeds over serial
    bool receivedUpdate = updateMotorsFromUART(left_motor_forward,
                                               left_motor_backward,
                                               right_motor_forward,
                                               right_motor_backward,
                                               left_motor_speed,
                                               right_motor_speed);

    if (receivedUpdate)
    {
      lastUartReceivedPacketMillis = millis();
      // Set the motor speeds
      setMotorSpeeds(left_motor_forward,
                     left_motor_backward,
                     right_motor_forward,
                     right_motor_backward,
                     left_motor_speed,
                     right_motor_speed);
    }
  }
}

unsigned long start_time_millis = 0;
const unsigned long ONE_MINUTE_MILLIS = 60 * 1000;

void setup() {
  
  // -------- SETUP: IR AND MOTORS --------
  
  Serial.begin(9600); // start the serial port
  pinMode(sensorA, INPUT);  //  Data of the Sensor 
  pinMode(sensorB, INPUT); 
  
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  analogWrite(LeftMotorPWMPin, 0); 
  analogWrite(RightMotorPWMPin,0);
  
  
  // -------- SETUP: UART CONTROL --------
  
    // Open serial communications and wait for port to open:
    Serial.begin(9600);
    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);

    // Wait for the Serial.begin() call to finish setting up.
    // If we don't do this, some of our first calls to Serial.println() could fail.
    delay(100);

    // CODE FOR TRYING TO DRIVE FORWARDS AT START
    // Currently these aren't driving in the right directions.
    //forwardBackwardLeftRight();
    
    // DEBUG code
    //Serial.println("setup");

  // ---- Troubleshooting ----
  start_time_millis = millis();
}
void loop() {
  
  // -------- LOOP: IR AND MOTORS --------
  static const unsigned long IR_UPDATE_INTERVAL_MILLISECONDS = 200;
  
  periodicallyAvoidObstacles(IR_UPDATE_INTERVAL_MILLISECONDS);
  //delay(250);
  
  // -------- LOOP: UART CONTROL --------
  static const unsigned long UART_UPDATE_INTERVAL_MILLISECONDS = 200;

  periodicallyUpdateMotorsFromUART(UART_UPDATE_INTERVAL_MILLISECONDS);
  // DEBUG code
  //Serial.println("loop");


}  

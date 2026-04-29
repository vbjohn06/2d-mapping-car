#include <Encoder.h>

/*    Pin Constants   */

// Buttons
const byte BUTTON = 21;
const byte TRANS_BUTTON = 20;



// Ultrasonic sensor
const byte ECHO = 11;
const byte TRIG = 12;
// VCC RED, GND BLUE



// L298N motor driver
const byte IN1 = 46; // Right Motor Forward, BLACK
const byte IN2 = 47; // Right Motor Backward, WHITE
const byte IN3 = 48; // Left Motor Forward, GREY
const byte IN4 = 49; // Left Motor Backward, PURPLE
const byte ENA = 9;  // Right Speed, BROWN
const byte ENB = 10; // Left Speed, BLUE



// Left encoder motor
const byte L_C1 = 3; // GREEN
const byte L_C2 = 18; // YELLOW
Encoder knobLeft(L_C1, L_C2);
long posLeft = -999;
// VCC BLACK, GND BLUE



// Right encoder motor
const byte R_C1 = 2; // GREEN
const byte R_C2 = 19; // YELLOW
Encoder knobRight(R_C2, R_C1);
long posRight = -999;
// VCC BLACK, GND BLUE



/* Calulation Constants */
const int maxSpeed = 100;
float distance = 0;
volatile float closest = 999;
volatile long closestEncoding = 0;
long getPos = 0;

const int setDistance = 35;



/* Boolean Operators */
volatile bool stop = true;
bool hasStarted = false;
bool hasSpun = false;
bool spinStarted = false;
bool transRequested = false;


/* Timers */
unsigned long lastPrint = 0;
const long printInterval = 200;

unsigned long lastDistance = 0;
const long distanceInterval = 30;

unsigned long lastStateChange = 0;

volatile unsigned long lastInterrupt = 0;
const byte DEBOUNCE = 200;

unsigned long lastCheck = 0;



/* Mapping Variables */
float theta = 0;
const int maxPoints = 600;
int point = 1;
float x[maxPoints];
float y[maxPoints];
float xCurrent = 0.0;
float yCurrent = 0.0;

const float base = 13.5;
const float radius = 6.5/2;
const long targetTicks = 4200;

long lastLeft = 0;
long lastRight = 0;

float xLastSaved = 0.0;
float yLastSaved = 0.0;
const float saveThreshold = 2.5; // Save a point every 2.5 cm



/* Debugging Variables */
int printA, printB;



void setup() {
  // 1. Serial for USB and Bluetooth module (HC-05)
  Serial.begin(115200); 
  Serial1.begin(9600); // For Bluetooth Module (that chose to not work)

  // 2. Ultrasonic Sensor Pins
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // 3. L298N Motor Driver Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // 4. Encoder Pins
  pinMode(L_C1, INPUT_PULLUP);
  pinMode(L_C2, INPUT_PULLUP);
  pinMode(R_C1, INPUT_PULLUP);
  pinMode(R_C2, INPUT_PULLUP);

  // 5. Start/Toggle Button
  pinMode(BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON), ButtonISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(TRANS_BUTTON), TransButtonISR, CHANGE);

  // 6. Set origin
  x[0] = 0; y[0] = 0;
}


// Toggles the car's movement
void ButtonISR() {
  unsigned long currentTime = millis();
  if(currentTime - lastInterrupt >= DEBOUNCE) {
    stop = !stop;
    lastInterrupt = currentTime;
  }
}


// Flags for data to be transmitted
void TransButtonISR() {
  unsigned long currentTime = millis();
  if(currentTime - lastInterrupt >= DEBOUNCE) {
    transRequested = true;
    lastInterrupt = currentTime;
  }
}



enum State { 
    IDLE
  , STARTING
  , ANGLING
  , FINDWALL
  , DRIVING
  , STUCK
  , TRANSMITTING
};
State currentState = IDLE;
void (*actions[])() = {
    idle
  , starting
  , angling
  , findwall
  , driving
  , stuck
  , transmitting
};


// Used to track the car's position (cm) and direction (rad)
void updatePosition() {
  long currentLeft = knobLeft.read(); 
  long currentRight = knobRight.read();

  // 1. Calculate change in distance for each wheel (cm)
  float cm_per_tick = (2.0 * PI * radius) / targetTicks;
  float dL = (currentLeft - lastLeft) * cm_per_tick;
  float dR = (currentRight - lastRight) * cm_per_tick;

  // 2. Save current counts for next iteration
  lastLeft = currentLeft;
  lastRight = currentRight;

  // 3. Calculate linear and angular displacement
  float dS = (dR + dL) / 2.0;
  float dTheta = (dR - dL) / base;

  // 4. Update Pose (X, Y, Theta)
  // Using (theta + dTheta/2) provides better accuracy during turns
  xCurrent += dS * cos(theta + (dTheta / 2.0));
  yCurrent += dS * sin(theta + (dTheta / 2.0));
  theta += dTheta;

  // 5. Keep theta between -PI and PI
  if (theta > PI) theta -= 2 * PI;
  if (theta < -PI) theta += 2 * PI;
}


// Main loop
void loop() {
  updatePosition();

  // Constantly looping through the current state.
  actions[currentState]();

  // This block of code is what saves the car's location over time.
  if(point < maxPoints) {
    
    // Calculate how far we've moved from the last "outline" point.
    float distSinceSave = sqrt(pow(xCurrent - xLastSaved, 2) + pow(yCurrent - yLastSaved, 2));

    if(distSinceSave >= saveThreshold) {
      x[point] = xCurrent;
      y[point] = yCurrent;
      
      xLastSaved = xCurrent;
      yLastSaved = yCurrent;
      point++;
    }
  }

  // Debugging
  getDistance();
  if(!stop && millis() - lastPrint >= printInterval) {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print("\t\tState: ");
    Serial.print(currentState);
    Serial.print("\t\tLeft Speed: ");
    Serial.print(printB);
    Serial.print("\t\tRight Speed: ");
    Serial.println(printA);
    lastPrint = millis();
  }
}


// Car is not moving
void idle() {
  stopMoving();

  if(!stop) {
    currentState = STARTING;
    lastStateChange = millis();
  } else if (transRequested) {
    transRequested = false; // "Catch" and reset the flag
    currentState = TRANSMITTING;
    Serial.println("Entering Transmission Mode...");
  }
}


// The car's starting operation
void starting() {
  if(stop) {
    currentState = IDLE;
    return;
  }

  getDistance();
  // Wait a second before either Driving or Aligning
  if(millis() - lastStateChange >= 1000) {
    if(distance < setDistance + 10) {
      currentState = DRIVING;
      return;
    } else {
      lastStateChange = millis();
      currentState = ANGLING;
    }
  }
}


// Aligns the car to the nearest wall
void angling() {
  if(stop) { currentState = IDLE; spinStarted = false; return; }

  // Sets variables once
  if(!spinStarted) {
    getPos = knobRight.read();
    closest = 999; // Reset closest distance
    spinStarted = true;
    hasSpun = false;
  }

  getDistance();
  // PHASE 1: SCANNING
  if(!hasSpun) {
    if(abs(getPos - knobRight.read()) > (targetTicks * 2)) {
      hasSpun = true;
      stopMoving(); // This includes a 200ms delay and resets pins
      return; // Exit and wait for the next loop to start Phase 2
    }
    
    // Power CW until finished
    analogWrite(ENB, 70); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENA, 70); digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    printA = -50; printB = 50;

    if(distance < closest && distance > 0) {
      closest = distance;
      closestEncoding = knobRight.read();
    }
  }
  // PHASE 2: ALIGNING
  else {
    long currentPos = knobRight.read();
    // Increase the buffer to 300 to account for momentum at speed 50
    if(currentPos > closestEncoding + 1500) { // CCW
      analogWrite(ENB, 70); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
      analogWrite(ENA, 70); digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    } else if(currentPos < closestEncoding) { // CW
      analogWrite(ENB, 70); digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
      analogWrite(ENA, 70); digitalWrite(IN1, HIGH);  digitalWrite(IN2, LOW);
    } else {
      hardBrake();
      currentState = FINDWALL;
      spinStarted = false; // Reset for future use
      hasSpun = false;
      delay(500);
    }
  }
}


// Travels straight in aligned direction
void findwall() {
  if(stop) {
    currentState = IDLE;
    return;
  }
  
  getDistance();
  if(distance > setDistance + 10) {
    // Right Motor
    analogWrite(ENA, maxSpeed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    // Left Motor
    analogWrite(ENB, maxSpeed);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    
    // Print Settings
    printA = maxSpeed; printB = maxSpeed;
  } else {
    currentState = DRIVING;
  }
}


// Handles car's main driving functions
void driving() {
  if(stop) {
    currentState = IDLE;
    return;
  }

  getDistance();
  if(distance < 15 && distance > 0) { // back up right if stuck
    stopMoving();
    currentState = STUCK;
  } else if(distance < setDistance-1.5) { // turn right
    int speedA = constrain(maxSpeed - (setDistance-1.5 - distance)*10, 50, maxSpeed);
    analogWrite(ENA, speedA); 
    analogWrite(ENB, maxSpeed);
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); // Ensure direction
    printA = speedA; printB = maxSpeed; 
  } else if(distance > setDistance+1.5) { // turn left
    int speedB = constrain(maxSpeed - (distance - setDistance+1.5)*10, 50, maxSpeed);
    analogWrite(ENB, speedB);
    analogWrite(ENA, maxSpeed);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); // Ensure direction
    printB = speedB; printA = maxSpeed;
  } else {
    // Straight
    analogWrite(ENA, maxSpeed);
    analogWrite(ENB, maxSpeed);
    printA = maxSpeed; printB = maxSpeed;
  }
}


// If the car is in close proximity to walls
void stuck() {
  if(stop) { currentState = IDLE; return; }

  // 1. Initialize the timer only ONCE when entering this state
  static unsigned long stuckStartTime = 0;
  if (stuckStartTime == 0) stuckStartTime = millis();

  getDistance();

  // 2. Movement: Back up and Pivot
  // Right back (100), Left back (60)
  analogWrite(ENA, 100); digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  analogWrite(ENB, 60);  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  printA = -100; printB = -60;

  // 3. Exit Condition (Wait 1.2 seconds to ensure clearance)
  if (millis() - stuckStartTime > 1200) {
    // Reset timer for next time
    stuckStartTime = 0;
    
    // 4. Move forward straight for a moment to "reset" the robot's heading
    // This prevents it from immediately hooking back into the wall
    analogWrite(ENA, 80); digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    analogWrite(ENB, 80); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    delay(300); 
    
    currentState = DRIVING;
  }
}


// Transmitting of the data
void transmitting() {
  // If the user un-toggles the stop button, go back to IDLE
  if (!stop) {
    currentState = IDLE;
    return;
  }

  // Loop through all points that have been recorded (up to 'point')
  for(int i = 0; i < point; i++) {
    Serial.print(x[i]);
    Serial.print(",");
    Serial.println(y[i]);
    
    // Give the Serial buffer time to send data (approx 10-15ms)
    delay(10); 
  }

  Serial.println("END_DUMP");

  // Reset the trigger and go back to IDLE
  transRequested = false;
  currentState = IDLE;
}


// Uses the HC-SR04 to find distance from wall
float getDistance() {
  if(stop) return;
  if(millis() - lastDistance >= distanceInterval) {

    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    long duration = pulseIn(ECHO, HIGH, 30000); // 30ms timeout prevents freezing
    distance = (duration * 0.0343) / 2;

    lastDistance = millis();

  }
}


// Stops car with a coast
void stopMoving() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(200);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}


// Stops car quickly
void hardBrake() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255); // Full "holding" power for a split second
  analogWrite(ENB, 255);
}

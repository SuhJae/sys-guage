const int pwmPinCPU = 9;    // CPU usage pin
const int pwmPinRAM = 10;   // RAM usage pin
const int pwmPinNET = 11;   // Network usage pin

int cpuSetpoint = 0;     // CPU setpoint (0-255)
int ramSetpoint = 0;     // RAM setpoint (0-255)
unsigned long netUsage = 0;  // Raw network usage (in Mbps)

int cpuPWM = 0;          // Current CPU PWM value
int ramPWM = 0;          // Current RAM PWM value
float netPWM = 0.0;      // Current Network PWM value (using float for more precision)

int prevCPUPWM = 0;      // Previous CPU PWM value

float tCPU = 0.0;        // Time variable for CPU interpolation
float dt = 0.01;         // Time step for updating t
unsigned long lastUpdateTime = 0;  // Time of the last update

// Network movement variables
float netSpeed = 0.0;    // Speed at which the network needle moves (proportional to network usage, in float)
bool netDirection = true;  // Network needle direction (true = forward, false = backward)

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Set pins as output
  pinMode(pwmPinCPU, OUTPUT);
  pinMode(pwmPinRAM, OUTPUT);
  pinMode(pwmPinNET, OUTPUT);

  // Setup sequence for initialization
  analogWrite(pwmPinNET, 255);
  delay(250);
  analogWrite(pwmPinCPU, 255);
  delay(250);
  analogWrite(pwmPinRAM, 255);
  delay(500);
  analogWrite(pwmPinCPU, 0);
  analogWrite(pwmPinRAM, 0);
  analogWrite(pwmPinNET, 0);
  delay(500);
  analogWrite(pwmPinCPU, 255);
  analogWrite(pwmPinRAM, 255);
  analogWrite(pwmPinNET, 255);
  delay(500);
}

void loop() {
  unsigned long currentTime = millis();

  // Check if data is available on the serial port (we expect 2 bytes for CPU/RAM, 4 for network usage)
  if (Serial.available() >= 6) {
    // Read CPU and RAM values (1 byte each)
    cpuSetpoint = Serial.read();
    ramSetpoint = Serial.read();

    // Read 4 bytes of raw network usage (uint32_t)
    netUsage = 0;
    for (int i = 0; i < 4; i++) {
      netUsage = (netUsage << 8) | Serial.read();
    }

    // Determine the network speed based on the network usage (100 Mbps = full range movement)
    netSpeed = (float)netUsage / 100.0 * 255.0;  // Scale network usage to control needle movement speed (in float)

    // Reset CPU interpolation times and update previous PWM value for CPU
    tCPU = 0.0;
    prevCPUPWM = cpuPWM;
  }

  // Interpolate and update CPU PWM values using linear interpolation
  if (currentTime - lastUpdateTime >= 10) {  // Update every 10ms
    if (tCPU < 1.0) {
      cpuPWM = linearInterpolation(tCPU, prevCPUPWM, cpuSetpoint);
      tCPU += dt;
    } else {
      cpuPWM = cpuSetpoint;
    }

    // For RAM, directly set the PWM value based on the setpoint (no interpolation)
    ramPWM = ramSetpoint;

    // Update the network needle based on the current network speed
    updateNetworkNeedle();

    // Write CPU and RAM PWM values to their pins
    analogWrite(pwmPinCPU, constrain(cpuPWM, 0, 255));
    analogWrite(pwmPinRAM, constrain(ramPWM, 0, 255));

    // Write the network PWM value, rounding it for output
    analogWrite(pwmPinNET, constrain(round(netPWM), 0, 255));

    lastUpdateTime = currentTime;
  }
}

// Linear interpolation function
int linearInterpolation(float t, int start, int end) {
  return int((1 - t) * start + t * end);  // Simple linear interpolation formula
}

// Update the network needle position based on network speed
void updateNetworkNeedle() {
  if (netSpeed > 0) {
    // Move the needle according to the direction and speed
    if (netDirection) {
      netPWM += netSpeed * dt;  // Move the needle forward
      if (netPWM >= 255) {  // If the needle reaches the maximum, reverse direction
        netPWM = 255;
        netDirection = false;
      }
    } else {
      netPWM -= netSpeed * dt;  // Move the needle backward
      if (netPWM <= 0) {  // If the needle reaches the minimum, reverse direction
        netPWM = 0;
        netDirection = true;
      }
    }
  }
}

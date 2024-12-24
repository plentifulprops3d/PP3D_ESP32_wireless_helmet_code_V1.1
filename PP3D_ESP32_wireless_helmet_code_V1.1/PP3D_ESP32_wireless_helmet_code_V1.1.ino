#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

#define SERVO_1_PIN 1
#define SERVO_2_PIN 2
#define LED1_PIN 4
#define LED2_PIN 5
#define BUTTON_PIN 6

Servo servo1;
Servo servo2;

uint8_t soundBoardMAC[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Defining the servo open and close positions
int servo1Close = 0;
int servo2Close = 180;

int servo1Open = 180;
int servo2Open = 0;

enum Position {
  OPEN_POSITION,
  CLOSE_POSITION
};

Position currentPosition1 = CLOSE_POSITION;
Position currentPosition2 = CLOSE_POSITION;

// Defining the opening and closing speeds of the faceplate (1 being slowest, 10 being fastest)
int openSpeed = 10;
int closeSpeed = 10;

// Setting servo1 and/or servo2 to true will have the servo turn off once it reaches the closed/open position, setting a servo to false will keep the servo on at all times when in the open/closed position.
bool detachServo1AtEnd = true;
bool detachServo2AtEnd = true;

unsigned long lastServoUpdate = 0;
const unsigned long servoUpdateInterval = 20;

int servo1Target = servo1Close;
int servo2Target = servo2Close;
int servo1Position = servo1Close;
int servo2Position = servo2Close;

unsigned long lastButtonPress = 0;
const unsigned long debounceDelay = 500;

unsigned long detachTime1 = 0;
unsigned long detachTime2 = 0;

// LED Close Delay Variables
unsigned long ledCloseDelayTime = 700;  // Set the desired delay time in milliseconds, this will give you a delay between the faceplate beginning to close, and the eyes turning on
unsigned long ledCloseDelayStart = 0;
bool ledsOn = false;

// User-configurable variables for the flicker effect and related parameters
bool enableFlickerEffect = true; // Toggle flicker effect on/off by setting it to true or false
int flickerTotalCount = 3;       // Total flickers (adjustable)
int flickerInterval = 35;        // Speed of flickering (milliseconds for on/off interval)
int remainingFlickers = 0;

// Flicker Effect Variables
unsigned long flickerStartTime = 0; // When flickering starts
bool flickering = false;
int flickerState = 0; // Track whether LEDs are ON (1) or OFF (0) during the flicker effect

// Wireless Data Receive Variables
unsigned long lastDataReceive = 0;

void onDataReceive(const uint8_t *mac, const uint8_t *incomingData, int len);
void sendSoundEffect(uint8_t effect);
void updateServoPositions();
void attachServos();
void detachServos();
void toggleHelmetState();
void initializeClosedPosition();
void turnOffLEDs(); // Function to turn off LEDs
void startFlickerEffect(); // Function to start the flickering effect
void executeFlickerEffect(); // Handles the flicker transitions

void setup() {
  Serial.begin(115200);
  
  // Display the MAC address of the ESP32
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("ESP32 MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  servo1.attach(SERVO_1_PIN);
  servo2.attach(SERVO_2_PIN);

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  Serial.println("WiFi initialized");

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(onDataReceive);
  Serial.println("ESP-NOW receive callback registered");

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, soundBoardMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  }

  initializeClosedPosition();
}

void loop() {
  if (digitalRead(BUTTON_PIN) == LOW && (millis() - lastButtonPress) > debounceDelay) {
    Serial.println("Button pressed, toggling helmet state.");
    lastButtonPress = millis();
    toggleHelmetState();
  }

  updateServoPositions();

  // Non-blocking LED close delay
  if (currentPosition1 == CLOSE_POSITION && millis() - ledCloseDelayStart >= ledCloseDelayTime && !ledsOn) {
    Serial.println("Turning on LEDs with delay.");
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, HIGH);
    ledsOn = true;

    // Start the flicker effect after LED delay time if it's enabled
    if (enableFlickerEffect) {
      startFlickerEffect();
    }
  }

  // Execute flicker effect if active
  if (flickering) {
    executeFlickerEffect();
  }

  // Ensure LEDs remain on after the flicker effect
  if (!enableFlickerEffect && currentPosition1 == CLOSE_POSITION && ledsOn) {
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, HIGH);
  }

  // Turn off LEDs if in open position
  if (currentPosition1 == OPEN_POSITION && ledsOn) {
    Serial.println("Turning off LEDs for open position.");
    turnOffLEDs();
  }
}

void initializeClosedPosition() {
  servo1Position = servo1Close;
  servo2Position = servo2Close;
  servo1.write(servo1Position);
  servo2.write(servo2Position);
  currentPosition1 = CLOSE_POSITION;
  currentPosition2 = CLOSE_POSITION;

  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);

  // Start the non-blocking LED close delay as soon as closed position is initialized
  ledCloseDelayStart = millis();

  // Add initial flicker effect if enabled
  if (enableFlickerEffect) {
    startFlickerEffect();
  }
}

void toggleHelmetState() {
  // Reset detach timers to interrupt ongoing detach functions
  detachTime1 = 0;
  detachTime2 = 0;

  // Safely stop flickering when toggling to the open position
  if (currentPosition1 == CLOSE_POSITION && flickering) {
    flickering = false; // Stop flicker effect
  }

  if (currentPosition1 == CLOSE_POSITION) {
    sendSoundEffect(2); // Open sound effect

    servo1Target = servo1Open;
    servo2Target = servo2Open;
    currentPosition1 = OPEN_POSITION;
    currentPosition2 = OPEN_POSITION;

    attachServos();
  } else {
    sendSoundEffect(3); // Close sound effect

    // Reset LED control
    ledsOn = false;
    turnOffLEDs();

    servo1Target = servo1Close;
    servo2Target = servo2Close;
    currentPosition1 = CLOSE_POSITION;
    currentPosition2 = CLOSE_POSITION;

    // Start LED close delay when entering close position
    ledCloseDelayStart = millis();

    attachServos();
  }
}

void updateServoPositions() {
  unsigned long currentTime = millis();

  if (currentTime - lastServoUpdate >= servoUpdateInterval) {
    lastServoUpdate = currentTime;

    if (servo1Position < servo1Target) {
      servo1Position += (currentPosition1 == OPEN_POSITION) ? openSpeed : closeSpeed;
      if (servo1Position > servo1Target) servo1Position = servo1Target;
    } else if (servo1Position > servo1Target) {
      servo1Position -= (currentPosition1 == OPEN_POSITION) ? openSpeed : closeSpeed;
      if (servo1Position < servo1Target) servo1Position = servo1Target;
    }

    if (servo2Position < servo2Target) {
      servo2Position += (currentPosition2 == OPEN_POSITION) ? openSpeed : closeSpeed;
      if (servo2Position > servo2Target) servo2Position = servo2Target;
    } else if (servo2Position > servo2Target) {
      servo2Position -= (currentPosition2 == OPEN_POSITION) ? openSpeed : closeSpeed;
      if (servo2Position < servo2Target) servo2Position = servo2Target;
    }

    servo1.write(servo1Position);
    servo2.write(servo2Position);

    if (servo1Position == servo1Target && detachServo1AtEnd && detachTime1 == 0) {
      detachTime1 = currentTime + 2000;
    }
    if (servo2Position == servo2Target && detachServo2AtEnd && detachTime2 == 0) {
      detachTime2 = currentTime + 2000;
    }

    if (currentTime >= detachTime1 && detachTime1 > 0) {
      servo1.detach();
      detachTime1 = 0;
    }

    if (currentTime >= detachTime2 && detachTime2 > 0) {
      servo2.detach();
      detachTime2 = 0;
    }
  }
}

void attachServos() {
  if (!servo1.attached()) {
    servo1.attach(SERVO_1_PIN);
    servo1.write(servo1Target);
  }
  if (!servo2.attached()) {
    servo2.attach(SERVO_2_PIN);
    servo2.write(servo2Target);
  }
}

void sendSoundEffect(uint8_t effect) {
  esp_err_t result = esp_now_send(soundBoardMAC, &effect, sizeof(effect));
  if (result == ESP_OK) {
    Serial.print("Sound effect ");
    Serial.print(effect);
    Serial.println(" sent successfully");
  } else {
    Serial.print("Failed to send sound effect ");
    Serial.println(effect);
  }
}

void onDataReceive(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Print the MAC address in a readable format
  Serial.print("Received data from MAC: ");
  for (int i = 0; i < 6; ++i) {
    if (i != 0) Serial.print(":");
    Serial.print(mac[i], HEX);
  }
  Serial.println();

  unsigned long currentTime = millis();
  if ((currentTime - lastDataReceive) > debounceDelay && len > 0) {
    lastDataReceive = currentTime;

    uint8_t data = incomingData[0];
    if (data == 101) {
      toggleHelmetState();
    }
  }
}

void startFlickerEffect() {
  flickering = true;
  remainingFlickers = flickerTotalCount;
  flickerStartTime = millis();
}

void executeFlickerEffect() {
  unsigned long currentTime = millis();
  if (currentTime - flickerStartTime >= flickerInterval) {
    flickerState = (flickerState == 0) ? 1 : 0;

    digitalWrite(LED1_PIN, flickerState);
    digitalWrite(LED2_PIN, flickerState);

    if (flickerState == 0) {
      remainingFlickers--;
      if (remainingFlickers == 0) {
        flickering = false;
        digitalWrite(LED1_PIN, HIGH);
        digitalWrite(LED2_PIN, HIGH);
      }
    }
    flickerStartTime = currentTime;
  }
}

void turnOffLEDs() {
  flickering = false;
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  ledsOn = false; // Ensure future operations don't check for "LEDs being ON."
}

// draw_bot_nodemcu.ino
// NodeMCU ESP8266 side of the finger-drawn-path robot
//
// The ESP8266 does NO path maths.
// It:
//   1. creates its own Wi-Fi access point (soft AP),
//   2. receives a list of motor steps from the laptop,
//   3. replays the steps at the specified times.
//
// CONNECTING FROM THE LAPTOP
//   Wi-Fi name : DrawBot
//   Password   : drawbot123
//   ESP address: 192.168.4.1
//   TCP port   : 8080
//
// WIRING: NodeMCU -> L298N
//
// D1 -> ENA  (left motor PWM)
// D2 -> IN1  (left motor direction A)
// D3 -> IN2  (left motor direction B)   (D3 = GPIO0 is a boot pin; if the
//                                        board ever fails to boot, move this
//                                        wire to D0 and change IN2 below)
//
// D5 -> ENB  (right motor PWM)
// D6 -> IN3  (right motor direction A)
// D7 -> IN4  (right motor direction B)
//
// NodeMCU GND, L298N GND and battery (-) MUST be common.
//
// LAPTOP <-> ESP8266 PROTOCOL
// ASCII lines over TCP, port 8080:
//
// CLR
// S,<left_pwm>,<right_pwm>,<duration_ms>
// RUN,<number_of_steps>
// STOP
//
// ESP8266 replies:
//
// CLEARED
// RUNNING n
// STOPPED
// DONE
// ERR ...
//
// SAFETY:
// If the laptop disconnects (or silently drops off Wi-Fi) while the
// robot is running, the motors stop.
//
// LED (NodeMCU LED is active LOW):
// Slow blink = waiting for laptop
// Solid      = laptop connected
// Fast blink = robot driving

#include <ESP8266WiFi.h>

// ============================================================
// Wi-Fi Access Point
// ============================================================

const char*   AP_SSID        = "DrawBot";
const char*   AP_PASS        = "drawbot123";   // must be at least 8 characters
const uint8_t AP_CHANNEL     = 1;
const uint8_t AP_MAX_CLIENTS = 2;

IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GATEWAY(192, 168, 4, 1);
IPAddress AP_SUBNET(255, 255, 255, 0);

const uint16_t TCP_PORT = 8080;

// ============================================================
// Pin definitions
// ============================================================

// Left motor
#define ENA D1
#define IN1 D2
#define IN2 D3

// Right motor
#define ENB D5
#define IN3 D6
#define IN4 D7

// Built-in NodeMCU LED
#define LED_PIN LED_BUILTIN

// ============================================================
// PWM
// ============================================================
//
// The ESP8266 has no hardware PWM; analogWrite() is software PWM.
// Range is set to 255 so PWM values are 0-255.
// 1 kHz stays stable while Wi-Fi is active and suits the L298N.
//

#define PWM_FREQ 1000

// ============================================================
// Program storage
// ============================================================

#define MAX_STEPS 1500

struct Step {
  int16_t l;      // left PWM: -255 to +255
  int16_t r;      // right PWM: -255 to +255
  uint16_t ms;    // duration in milliseconds
};

Step program[MAX_STEPS];

uint16_t nSteps = 0;
uint16_t curStep = 0;

bool running = false;

uint32_t stepStart = 0;

// ============================================================
// Wi-Fi / TCP
// ============================================================

WiFiServer server(TCP_PORT);
WiFiClient client;

String rxLine;

// ============================================================
// Motor functions
// ============================================================

void writePwm(int pin, int duty)
{
  if (duty < 0)
    duty = 0;

  if (duty > 255)
    duty = 255;

  analogWrite(pin, duty);
}


// signedPwm:
//   > 0 : forward
//   < 0 : reverse
//   = 0 : coast

void driveMotor(int enPin, int a, int b, int signedPwm)
{
  int pwm = abs(signedPwm);

  if (pwm > 255)
    pwm = 255;

  // Motor stopped
  if (pwm == 0)
  {
    digitalWrite(a, LOW);
    digitalWrite(b, LOW);

    writePwm(enPin, 0);

    return;
  }

  // Forward
  if (signedPwm > 0)
  {
    digitalWrite(a, HIGH);
    digitalWrite(b, LOW);
  }

  // Reverse
  else
  {
    digitalWrite(a, LOW);
    digitalWrite(b, HIGH);
  }

  writePwm(enPin, pwm);
}


// Set both motors

void setMotors(int left, int right)
{
  driveMotor(ENA, IN1, IN2, left);
  driveMotor(ENB, IN3, IN4, right);
}


// Stop both motors

void stopMotors()
{
  setMotors(0, 0);
}

// ============================================================
// Command handling
// ============================================================

void handleLine(const String &s)
{
  // ----------------------------------------------------------
  // CLEAR
  // ----------------------------------------------------------

  if (s == "CLR")
  {
    if (!running)
    {
      nSteps = 0;
      curStep = 0;
    }

    client.print("CLEARED\n");
  }


  // ----------------------------------------------------------
  // ADD STEP
  //
  // Format:
  // S,left_pwm,right_pwm,duration
  //
  // Example:
  // S,150,150,500
  // S,-150,150,300
  // ----------------------------------------------------------

  else if (s.startsWith("S,"))
  {
    int l = 0;
    int r = 0;

    unsigned int ms = 0;

    if (
      !running &&
      nSteps < MAX_STEPS &&
      sscanf(s.c_str(), "S,%d,%d,%u", &l, &r, &ms) == 3
    )
    {
      // Limit left PWM

      if (l > 255)
        l = 255;

      if (l < -255)
        l = -255;

      // Limit right PWM

      if (r > 255)
        r = 255;

      if (r < -255)
        r = -255;

      // Limit duration

      if (ms > 65535u)
        ms = 65535u;

      program[nSteps].l = (int16_t)l;
      program[nSteps].r = (int16_t)r;
      program[nSteps].ms = (uint16_t)ms;

      nSteps++;
    }
  }


  // ----------------------------------------------------------
  // RUN
  //
  // Example:
  // RUN,100
  // ----------------------------------------------------------

  else if (s.startsWith("RUN,"))
  {
    unsigned int n = 0;

    if (running)
    {
      client.print("ERR busy\n");
    }

    else if (
      sscanf(s.c_str(), "RUN,%u", &n) == 1 &&
      n == nSteps &&
      nSteps > 0
    )
    {
      curStep = 0;

      // Start first step

      setMotors(program[0].l, program[0].r);

      stepStart = millis();

      running = true;

      client.printf("RUNNING %u\n", (unsigned int)nSteps);

      Serial.printf("Running %u steps\n", (unsigned int)nSteps);
    }

    else
    {
      client.printf("ERR count %u\n", (unsigned int)nSteps);
    }
  }


  // ----------------------------------------------------------
  // STOP
  // ----------------------------------------------------------

  else if (s == "STOP")
  {
    stopMotors();

    running = false;

    client.print("STOPPED\n");

    Serial.println("STOP received");
  }
}

// ============================================================
// LED
// ============================================================

void updateLed()
{
  if (running)
  {
    // Fast blink while driving
    digitalWrite(LED_PIN, (millis() / 100) % 2 ? LOW : HIGH);
  }

  else if (client.connected())
  {
    // Solid LED when connected
    digitalWrite(LED_PIN, LOW);
  }

  else
  {
    // Slow blink when waiting
    digitalWrite(LED_PIN, (millis() / 600) % 2 ? LOW : HIGH);
  }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(300);

  Serial.println();
  Serial.println("================================");
  Serial.println("      DrawBot NodeMCU ESP8266");
  Serial.println("================================");


  // ----------------------------------------------------------
  // GPIO configuration
  // ----------------------------------------------------------

  pinMode(LED_PIN, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);


  // ----------------------------------------------------------
  // ESP8266 PWM configuration
  // ----------------------------------------------------------

  analogWriteRange(255);
  analogWriteFreq(PWM_FREQ);


  // ----------------------------------------------------------
  // Stop motors initially
  // ----------------------------------------------------------

  stopMotors();


  // ----------------------------------------------------------
  // Start Wi-Fi Access Point
  // ----------------------------------------------------------

  WiFi.persistent(false);      // don't wear out flash with Wi-Fi settings
  WiFi.mode(WIFI_AP);

  WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);

  bool apOk = WiFi.softAP(
    AP_SSID,
    AP_PASS,
    AP_CHANNEL,
    false,                     // visible network
    AP_MAX_CLIENTS
  );

  Serial.print("Access point : ");
  Serial.print(AP_SSID);
  Serial.println(apOk ? "  (started)" : "  (FAILED - check password length)");

  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());


  // ----------------------------------------------------------
  // Start TCP server
  // ----------------------------------------------------------

  server.begin();

  Serial.printf(
    "TCP server on port %u - ready\n",
    (unsigned int)TCP_PORT
  );
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
  // ==========================================================
  // 1. Accept laptop connection
  // ==========================================================

  if (!client.connected())
  {
    // Safety:
    // If laptop disappeared while robot was moving,
    // immediately stop motors.

    if (running)
    {
      stopMotors();

      running = false;

      Serial.println("Laptop disconnected -> STOPPED");
    }


    // Check for new client

    WiFiClient c = server.available();

    if (c)
    {
      client = c;

      client.setNoDelay(true);

      // Detect a silently dropped Wi-Fi link within a few seconds
      // (idle 3 s, probe every 1 s, give up after 3 misses)
      client.keepAlive(3, 1, 3);

      rxLine = "";

      Serial.println("Laptop connected");
    }
  }


  // ==========================================================
  // 2. Read commands from laptop
  // ==========================================================

  while (client.connected() && client.available() > 0)
  {
    char ch = (char)client.read();


    // End of command

    if (ch == '\n')
    {
      handleLine(rxLine);

      rxLine = "";
    }

    // Ignore carriage return

    else if (ch != '\r' && rxLine.length() < 48)
    {
      rxLine += ch;
    }
  }


  // ==========================================================
  // 3. Replay stored program
  // ==========================================================
  //
  // Step boundaries use cumulative timestamps:
  //
  //     stepStart += duration
  //
  // rather than:
  //
  //     stepStart = millis()
  //
  // This prevents timing errors from accumulating.
  //

  while (
    running &&
    (uint32_t)(millis() - stepStart) >= program[curStep].ms
  )
  {
    // Move timestamp to next boundary

    stepStart += program[curStep].ms;

    curStep++;


    // --------------------------------------------------------
    // Program finished
    // --------------------------------------------------------

    if (curStep >= nSteps)
    {
      stopMotors();

      running = false;

      client.print("DONE\n");

      Serial.println("Program finished");
    }

    // --------------------------------------------------------
    // Start next step
    // --------------------------------------------------------

    else
    {
      setMotors(program[curStep].l, program[curStep].r);
    }
  }


  // ==========================================================
  // 4. Update status LED
  // ==========================================================

  updateLed();
}

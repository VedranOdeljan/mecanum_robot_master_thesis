#include <Arduino.h>

float speedScale = 1.0;

const int dirPins[4] = {2, 4, 6, 8};
const int pulPins[4] = {3, 5, 7, 9};

bool dirInverted[4] = {false, true, false, true};

const int encA[4] = {20, 18, 16, 14};
const int encB[4] = {21, 19, 17, 15};

const int pulsesPerRev = 1024 * 4;
volatile long encoderCount[4] = {0, 0, 0, 0};

float motorFreq[4] = {0, 0, 0, 0};
bool pulseState[4] = {false, false, false, false};
unsigned long lastPulseMicros[4] = {0, 0, 0, 0};
unsigned long halfPeriod[4] = {0, 0, 0, 0};

int kp = 20, kd = 12, ki = 0, ko = 50;

String rxBuffer = "";

// ENKODERI
void updateEncoder(int idx) {
    bool A = digitalRead(encA[idx]);
    bool B = digitalRead(encB[idx]);
    if (A == B) encoderCount[idx]++;
    else encoderCount[idx]--;
}

void encoderISR0() { updateEncoder(0); }
void encoderISR1() { updateEncoder(1); }
void encoderISR2() { updateEncoder(2); }
void encoderISR3() { updateEncoder(3); }

// MOTOR LOOP
void runMotors() {
    unsigned long now = micros();
    for (int i = 0; i < 4; i++) {
        if (motorFreq[i] > 0 && (now - lastPulseMicros[i]) >= halfPeriod[i]) {
            pulseState[i] = !pulseState[i];
            digitalWrite(pulPins[i], pulseState[i]);
            lastPulseMicros[i] = now;
        }
    }
}

//Čitanje ros2 komandi
void parseCommand(String &cmd)
{
    cmd.trim();

    if (cmd == "e") {
        Serial.print(encoderCount[0]); Serial.print(" ");
        Serial.print(encoderCount[1]); Serial.print(" ");
        Serial.print(encoderCount[2]); Serial.print(" ");
        Serial.println(encoderCount[3]);
        return;
    }


    // CMD: PID u kp:kd:ki:ko
    if (cmd.startsWith("u ")) {
        sscanf(cmd.c_str(), "u %d:%d:%d:%d", &kp, &kd, &ki, &ko);
        return;
    }

   
    // CMD: MOTORI m fl fr rl rr
    if (cmd.startsWith("m ")) {

        int fl, fr, rl, rr;
        int matched = sscanf(cmd.c_str(), "m %d %d %d %d", &fl, &fr, &rl, &rr);

        if (matched == 4) {

            int pwm[4] = {fl, fr, rl, rr};

            for (int i = 0; i < 4; i++) {

                int val = pwm[i];
                if (val < 0) {
                    digitalWrite(dirPins[i], dirInverted[i] ? LOW : HIGH);
                    val = -val;
                } else {
                    digitalWrite(dirPins[i], dirInverted[i] ? HIGH : LOW);
                }

                motorFreq[i] = constrain(val, 0, 4000);
                halfPeriod[i] = motorFreq[i] > 0 ? 1000000 / (motorFreq[i] * 2) : 0;
            }
        }
        return;
    }
}

void readSerial()
{
    while (Serial.available()) {

        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (rxBuffer.length() > 0) {
                parseCommand(rxBuffer);
                rxBuffer = "";
            }
        }
        else {
            rxBuffer += c;

            // zaštita: ako je buffer prevelik → reset
            if (rxBuffer.length() > 100) {
                rxBuffer = "";
            }
        }
    }
}

void setup()
{
    Serial.begin(115200);   //57600
    for (int i = 0; i < 4; i++) {
        pinMode(dirPins[i], OUTPUT);
        pinMode(pulPins[i], OUTPUT);
        digitalWrite(pulPins[i], LOW);
        digitalWrite(dirPins[i], dirInverted[i] ? HIGH : LOW);
    }
    for (int i = 0; i < 4; i++) {
        pinMode(encA[i], INPUT_PULLUP);
        pinMode(encB[i], INPUT_PULLUP);
    }

    attachInterrupt(digitalPinToInterrupt(encA[0]), encoderISR0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encA[1]), encoderISR1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encA[2]), encoderISR2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encA[3]), encoderISR3, CHANGE);
}
void loop()
{
    readSerial();
    runMotors();
}

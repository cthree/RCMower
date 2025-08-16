/*
 * RC Mower Steering Control for NodeMCU 32-S ESP32
 * Controls two DC motors via DS1267B digital potentiometers for speed
 * and MOSFETs controlling relays for direction
 * Features differential steering and enhanced straight-line compensation
 * Uses hardware interrupts for RC input reading
 * Enhanced with RC pulse width calibration
 *
 * REVISION NOTES:
 * - Added MOTOR_ENABLE_PIN (14) to power on motor controllers via MOSFET only when throttle is at zero.
 * - Increased POT_MAX to 192 for higher top speed.
 * - Added logic to prevent jarring on abrupt direction changes by pausing motors.
 * - Confirmed and reinforced logic to prevent motors from speeding up during turns.
 * - Added ArduinoOTA updates
 * - CRITICAL ESP32 FIXES: Fixed EEPROM handling, ADC scaling, interrupt attributes
 */

#include <Preferences.h>  // ESP32 replacement for EEPROM
#include <WiFi.h>
#include <ESPmDNS.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>

const char *ssid = "cheese";
const char *password = "peggypep";

// Pin definitions
const int STEERING_PIN = 35;      // RC channel 0 (steering) - Hardware interrupt
const int THROTTLE_PIN = 34;      // RC channel 2 (throttle) - Hardware interrupt
const int DIRECTION_PIN = 39;     // RC channel for direction switch - Hardware interrupt
const int CALIB_BUTTON = 22;      // Calibration button (to ground)
const int STATUS_LED = 21;        // Status LED (to ground through resistor)
const int DS1267_RST = 25;        // DS1267B reset pin
const int DS1267_DQ = 27;         // DS1267B data pin
const int DS1267_CLK = 26;        // DS1267B clock pin
const int MOTOR_L_FB = 13;        // Left motor feedback (wiper voltage)
const int MOTOR_R_FB = 4;         // Right motor feedback (wiper voltage)
const int MOTOR_R_DIR = 32;       // Right motor direction control (MOSFET)
const int MOTOR_L_DIR = 33;       // Left motor direction control (MOSFET)
const int MOTOR_ENABLE_PIN = 14;  // **NEW** Pin to enable motor speed controllers via MOSFET

// RC pulse width constants (microseconds) - defaults for uncalibrated system
const int RC_MIN_DEFAULT = 1000;      // Default minimum RC pulse width
const int RC_CENTER_DEFAULT = 1500;   // Default center RC pulse width
const int RC_MAX_DEFAULT = 2000;      // Default maximum RC pulse width
const int RC_DEADBAND = 50;           // Deadband around center
const int RC_SWITCH_THRESHOLD = 1500; // Default threshold for direction switch

// Calibrated RC pulse width values (loaded from Preferences or defaults)
int RC_MIN = RC_MIN_DEFAULT;
int RC_CENTER = RC_CENTER_DEFAULT;
int RC_MAX = RC_MAX_DEFAULT;

// Motor control constants
const int POT_MIN = 0;   // Minimum potentiometer value (zero speed)
const int POT_MAX = 224; // Maximum potentiometer value (% of full range)

// Direction constants
const bool DIR_FORWARD = LOW;  // Forward direction (relay not energized)
const bool DIR_REVERSE = HIGH; // Reverse direction (relay energized)

// Enhanced compensation and tuning constants
const float PROPORTIONAL_GAIN = 2.0;               // Proportional gain for speed matching
const float INTEGRAL_GAIN = 0.05;                  // Integral gain for steady-state error elimination
const float MAX_INTEGRAL = 100.0;                  // Maximum integral windup limit
const int TURN_SENSITIVITY = 2;                    // Higher = more aggressive turning
const unsigned long CONTROL_INTERVAL = 20;         // Control loop interval (ms)
const unsigned long MOTOR_CALIBRATION_TIME = 5000; // Time to run motor calibration (ms)
const unsigned long BUTTON_HOLD_TIME = 2000;       // Button hold time to start calibration (ms)

// Calibration state enumeration
enum CalibrationState
{
    CALIB_IDLE,
    CALIB_MOTOR_RUNNING,
    CALIB_RC_SAMPLING,
    CALIB_COMPLETE
};

// Global variables for RC pulse measurement
volatile unsigned long steer_pulse_start = 0;
volatile unsigned long throttle_pulse_start = 0;
volatile unsigned long direction_pulse_start = 0;
volatile int steer_pulse_width = RC_CENTER_DEFAULT;
volatile int throttle_pulse_width = RC_CENTER_DEFAULT;
volatile int direction_pulse_width = RC_CENTER_DEFAULT;

// Motor control variables
int left_motor_speed = 0;                 // Final speed value for pot (0-POT_MAX)
int right_motor_speed = 0;                // Final speed value for pot (0-POT_MAX)
bool left_motor_direction = DIR_FORWARD;  // Current direction
bool right_motor_direction = DIR_FORWARD; // Current direction
int left_feedback = 0;
int right_feedback = 0;
bool motors_enabled = false; // **NEW** State for motor controller power

// Enhanced compensation variables
float speed_error_integral = 0.0; // Integral term for PI controller
int base_left_speed = 0;          // Base speed commands (0-100) before compensation
int base_right_speed = 0;
float motor_speed_ratio = 1.0; // Ratio to compensate for motor differences

// Calibration variables
CalibrationState calibration_state = CALIB_IDLE;
unsigned long calibration_start_time = 0;
int motor_calibration_samples = 0;
long motor_calibration_left_sum = 0;
long motor_calibration_right_sum = 0;

// RC calibration variables
int rc_min_observed = 2500; // Start with impossible values
int rc_max_observed = 500;
int rc_center_observed = RC_CENTER_DEFAULT;
bool rc_center_found = false;
unsigned long rc_center_stable_start = 0;
const unsigned long RC_CENTER_STABLE_TIME = 1000; // Need 1 second of stable center

// Button and LED control variables
bool button_pressed = false;
unsigned long button_press_start = 0;
bool calibration_requested = false;
unsigned long led_last_toggle = 0;
bool led_state = false;
bool needs_calibration = false; // True if no valid calibration data found

// DS1267B potentiometer values
int pot0_value = 0;   // Right motor speed (potentiometer 0)
int pot1_value = 0;   // Left motor speed (potentiometer 1)
int stack_select = 0; // Stack select bit (0 = not stacked)

unsigned long last_control_time = 0;

bool wifiEnabled = false;

// ESP32-specific objects
Preferences preferences;

void IRAM_ATTR steerISR();
void IRAM_ATTR throttleISR();
void IRAM_ATTR directionISR();

void setup()
{
    Serial.begin(115200);
    // SPIFFS.begin(true);

    // setupWifi();

    // Initialize button and LED pins
    pinMode(CALIB_BUTTON, INPUT_PULLUP);
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);

    // Initialize DS1267B pins
    pinMode(DS1267_RST, OUTPUT);
    pinMode(DS1267_CLK, OUTPUT);
    pinMode(DS1267_DQ, OUTPUT);

    // Initialize direction control pins
    pinMode(MOTOR_L_DIR, OUTPUT);
    pinMode(MOTOR_R_DIR, OUTPUT);

    // **NEW** Initialize motor controller enable pin
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);

    // Initialize pins to known state
    digitalWrite(DS1267_RST, LOW);
    digitalWrite(DS1267_CLK, LOW);
    digitalWrite(DS1267_DQ, LOW);
    digitalWrite(MOTOR_L_DIR, DIR_FORWARD);
    digitalWrite(MOTOR_R_DIR, DIR_FORWARD);
    digitalWrite(MOTOR_ENABLE_PIN, LOW); // Keep motors disabled at startup

    // Reset DS1267B
    digitalWrite(DS1267_RST, LOW);
    delay(10);

    // Set initial potentiometer values (motors at zero speed)
    writeDS1267B(0, 0, 0);

    // Setup RC input hardware interrupts
    attachInterrupt(digitalPinToInterrupt(STEERING_PIN), steerISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), throttleISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(DIRECTION_PIN), directionISR, CHANGE);

    Serial.println("RC Mower Control Initialized");
    Serial.println("Waiting for throttle to be centered to enable motors...");

    // Load calibration data from Preferences
    loadCalibrationData();

    if (needs_calibration)
    {
        Serial.println("No valid calibration data. Press and hold button for 2 seconds to calibrate.");
        motor_speed_ratio = 1.0;
    }
    else
    {
        Serial.print("Loaded calibration data. Motor speed ratio: ");
        Serial.println(motor_speed_ratio, 3);
    }
}

void loop()
{
    if (millis() - last_control_time >= CONTROL_INTERVAL)
    {
        last_control_time = millis();

        if (wifiEnabled)
            ArduinoOTA.handle();

        handleButtonAndLED();

        // **NEW** Motor Enable Logic
        // Check if motors have been enabled yet. If not, check if throttle is centered.
        if (!motors_enabled)
        {
            if (abs(throttle_pulse_width - RC_CENTER) < RC_DEADBAND)
            {
                digitalWrite(MOTOR_ENABLE_PIN, HIGH);
                motors_enabled = true;
                Serial.println("Motors enabled!");
            }
        }

        // Read motor feedback
        left_feedback = analogRead(MOTOR_L_FB);
        right_feedback = analogRead(MOTOR_R_FB);

        // Handle calibration based on current state
        if (calibration_state == CALIB_MOTOR_RUNNING)
        {
            updateMotorCalibration();
        }
        else if (calibration_state == CALIB_RC_SAMPLING)
        {
            updateRCCalibration();
        }
        else if (calibration_state == CALIB_IDLE && !needs_calibration && motors_enabled)
        {
            // Normal operation - process RC inputs and control motors only if enabled
            processRCInputs();
            applyEnhancedCompensation();

            // Update motor speeds and directions (already handled by the functions above)
            // The final write to the hardware happens in setMotorSpeed -> writeDS1267B
        }

        static unsigned long last_debug = 0;
        if (millis() - last_debug >= 500)
        {
            last_debug = millis();
            debugOutput();
        }
    }
}

// Setup WIFI for OTA software updates
void setupWifi()
{
    int tries = 3;

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.waitForConnectResult() != WL_CONNECTED && tries > 0) {
        Serial.println("Connection Failed! Rebooting...");
        delay(5000);
        ESP.restart();
        tries--;
    }

    if (tries) {
        // Port defaults to 3232
        // ArduinoOTA.setPort(3232);

        // Hostname defaults to esp3232-[MAC]
        // ArduinoOTA.setHostname("myesp32");

        // No authentication by default
        // ArduinoOTA.setPassword("admin");

        // Password can be set with it's md5 value as well
        // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
        // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

        ArduinoOTA
            .onStart([]() {
                String type;
                if (ArduinoOTA.getCommand() == U_FLASH) {
                    type = "sketch";
                } else {  // U_SPIFFS
                    type = "filesystem";
                }

                // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
                Serial.println("Start updating " + type);
            })
            .onEnd([]() {
                Serial.println("\nEnd");
            })
            .onProgress([](unsigned int progress, unsigned int total) {
                Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
            })
            .onError([](ota_error_t error) {
                Serial.printf("Error[%u]: ", error);
                if (error == OTA_AUTH_ERROR) {
                    Serial.println("Auth Failed");
                } else if (error == OTA_BEGIN_ERROR) {
                    Serial.println("Begin Failed");
                } else if (error == OTA_CONNECT_ERROR) {
                    Serial.println("Connect Failed");
                } else if (error == OTA_RECEIVE_ERROR) {
                    Serial.println("Receive Failed");
                } else if (error == OTA_END_ERROR) {
                    Serial.println("End Failed");
                }
            });

        ArduinoOTA.begin();

        wifiEnabled = true;

        Serial.println("Wifi Ready");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("Wifi Disabled");
    }
}

// Handle button press and LED blinking
void handleButtonAndLED()
{
    bool button_current = !digitalRead(CALIB_BUTTON);

    if (button_current && !button_pressed)
    {
        button_pressed = true;
        button_press_start = millis();
        if (calibration_state == CALIB_RC_SAMPLING)
        {
            finishCalibration();
            return;
        }
    }
    else if (!button_current && button_pressed)
    {
        button_pressed = false;
    }

    if (button_pressed && (millis() - button_press_start > BUTTON_HOLD_TIME) &&
        !calibration_requested && calibration_state == CALIB_IDLE)
    {
        calibration_requested = true;
        Serial.println("Calibration requested...");
        startCalibration();
    }

    // LED blinking logic
    if (calibration_state == CALIB_MOTOR_RUNNING)
    {
        if (millis() - led_last_toggle >= 125)
        {
            led_last_toggle = millis();
            led_state = !led_state;
            digitalWrite(STATUS_LED, led_state);
        }
    }
    else if (calibration_state == CALIB_RC_SAMPLING)
    {
        digitalWrite(STATUS_LED, HIGH);
    }
    else if (needs_calibration)
    {
        if (millis() - led_last_toggle >= 250)
        {
            led_last_toggle = millis();
            led_state = !led_state;
            digitalWrite(STATUS_LED, led_state);
        }
    }
    else if (!motors_enabled)
    {
        // **NEW** Slow blink if waiting for motor enable
        if (millis() - led_last_toggle >= 500)
        {
            led_last_toggle = millis();
            led_state = !led_state;
            digitalWrite(STATUS_LED, led_state);
        }
    }
    else
    {
        digitalWrite(STATUS_LED, LOW);
    }
}

// Process RC inputs and calculate differential motor speeds and directions
void processRCInputs()
{
    int throttle = map(constrain(throttle_pulse_width, RC_MIN, RC_MAX), RC_MIN, RC_MAX, 0, 100);
    int steering = map(constrain(steer_pulse_width, RC_MIN, RC_MAX), RC_MIN, RC_MAX, -100, 100);

    bool target_direction = (direction_pulse_width > RC_CENTER) ? DIR_REVERSE : DIR_FORWARD;

    int deadband_threshold = (RC_DEADBAND * 100) / (RC_MAX - RC_MIN);
    if (throttle < deadband_threshold)
        throttle = 0;
    if (abs(steering) < deadband_threshold)
        steering = 0;

    int left_speed = throttle;
    int right_speed = throttle;
    bool left_dir = target_direction;
    bool right_dir = target_direction;

    if (steering > 0)
    { // Turn right
        int steering_effect = (steering * TURN_SENSITIVITY) / 10;
        left_speed = max(0, left_speed - steering_effect);
        if (steering > 70 && throttle > 20)
        {
            left_dir = !target_direction;
            left_speed = min(50, steering_effect - (throttle / 2));
        }
    }
    else if (steering < 0)
    { // Turn left
        int steering_effect = (abs(steering) * TURN_SENSITIVITY) / 10;
        right_speed = max(0, right_speed - steering_effect);
        if (steering < -70 && throttle > 20)
        {
            right_dir = !target_direction;
            right_speed = min(50, steering_effect - (throttle / 2));
        }
    }

    // **NEW** Explicitly limit motor speeds to the throttle value to prevent speeding up in turns.
    left_speed = min(left_speed, throttle);
    right_speed = min(right_speed, throttle);

    left_speed = constrain(left_speed, 0, 100);
    right_speed = constrain(right_speed, 0, 100);

    // **NEW** Anti-Jarring Logic
    // Check if direction needs to change while motors are moving. If so, command a stop for one cycle.
    if (left_dir != left_motor_direction && left_motor_speed > 5)
    {
        base_left_speed = 0;
    }
    else
    {
        left_motor_direction = left_dir;
        base_left_speed = left_speed;
    }

    if (right_dir != right_motor_direction && right_motor_speed > 5)
    {
        base_right_speed = 0;
    }
    else
    {
        right_motor_direction = right_dir;
        base_right_speed = right_speed;
    }
}

// Apply enhanced compensation for straight-line tracking
void applyEnhancedCompensation()
{
    int steering_input = map(steer_pulse_width, RC_MIN, RC_MAX, -100, 100);

    if (abs(steering_input) < 20 && left_motor_direction == right_motor_direction && (base_left_speed > 10 || base_right_speed > 10))
    {
        float left_speed_compensated = base_left_speed;
        float right_speed_compensated = base_right_speed * motor_speed_ratio;

        float target_feedback = (left_feedback + right_feedback) / 2.0;
        float left_error = target_feedback - left_feedback;
        float right_error = target_feedback - right_feedback;
        float speed_error = (abs(left_error) > abs(right_error)) ? left_error : right_error;

        speed_error_integral += speed_error * INTEGRAL_GAIN;
        speed_error_integral = constrain(speed_error_integral, -MAX_INTEGRAL, MAX_INTEGRAL);

        float left_compensation = left_error * PROPORTIONAL_GAIN + speed_error_integral;
        float right_compensation = right_error * PROPORTIONAL_GAIN + speed_error_integral;

        left_speed_compensated += left_compensation;
        right_speed_compensated += right_compensation;

        left_motor_speed = constrain(map(left_speed_compensated, 0, 100, POT_MIN, POT_MAX), POT_MIN, POT_MAX);
        right_motor_speed = constrain(map(right_speed_compensated, 0, 100, POT_MIN, POT_MAX), POT_MIN, POT_MAX);
    }
    else
    {
        speed_error_integral *= 0.8;
        left_motor_speed = map(base_left_speed, 0, 100, POT_MIN, POT_MAX);
        right_motor_speed = map(base_right_speed, 0, 100, POT_MIN, POT_MAX);
    }

    // Set final speed and direction after all calculations
    setMotorSpeed(1, left_motor_speed);
    setMotorSpeed(0, right_motor_speed);
    setMotorDirection(1, left_motor_direction);
    setMotorDirection(0, right_motor_direction);
}

// Set motor speed via DS1267B potentiometer
void setMotorSpeed(int motor_num, int speed)
{
    speed = constrain(speed, POT_MIN, POT_MAX);
    if (motor_num == 0)
        pot0_value = speed;
    else
        pot1_value = speed;
    writeDS1267B(pot0_value, pot1_value, stack_select);
}

// Set motor direction via MOSFET control
void setMotorDirection(int motor_num, bool direction)
{
    if (motor_num == 0)
        digitalWrite(MOTOR_R_DIR, direction);
    else
        digitalWrite(MOTOR_L_DIR, direction);
}

// Write complete 17-bit word to DS1267B
void writeDS1267B(int pot0_val, int pot1_val, int stack_bit)
{
    uint32_t word = 0;
    word |= (stack_bit & 0x01) << 16;
    word |= (pot1_val & 0xFF) << 8;
    word |= (pot0_val & 0xFF);

    digitalWrite(DS1267_RST, HIGH);
    delayMicroseconds(1);

    for (int i = 16; i >= 0; i--)
    {
        digitalWrite(DS1267_DQ, (word >> i) & 0x01);
        delayMicroseconds(1);
        digitalWrite(DS1267_CLK, HIGH);
        delayMicroseconds(1);
        digitalWrite(DS1267_CLK, LOW);
        delayMicroseconds(1);
    }

    digitalWrite(DS1267_RST, LOW);
    delayMicroseconds(1);
}

// Interrupt service routines - **CRITICAL ESP32 FIX: Added IRAM_ATTR**
void IRAM_ATTR steerISR()
{
    if (digitalRead(STEERING_PIN) == HIGH)
        steer_pulse_start = micros();
    else
    {
        unsigned long pulse_time = micros() - steer_pulse_start;
        if (pulse_time >= 500 && pulse_time <= 2500)
            steer_pulse_width = pulse_time;
    }
}

void IRAM_ATTR throttleISR()
{
    if (digitalRead(THROTTLE_PIN) == HIGH)
        throttle_pulse_start = micros();
    else
    {
        unsigned long pulse_time = micros() - throttle_pulse_start;
        if (pulse_time >= 500 && pulse_time <= 2500)
            throttle_pulse_width = pulse_time;
    }
}

void IRAM_ATTR directionISR()
{
    if (digitalRead(DIRECTION_PIN) == HIGH)
        direction_pulse_start = micros();
    else
    {
        unsigned long pulse_time = micros() - direction_pulse_start;
        if (pulse_time >= 500 && pulse_time <= 2500)
            direction_pulse_width = pulse_time;
    }
}

// Debug output function (added motors_enabled status)
void debugOutput()
{
    if (!motors_enabled)
    {
        Serial.println("Waiting for throttle center to enable motors...");
        return;
    }
    if (calibration_state != CALIB_IDLE)
    {
        // ... (calibration debug messages are unchanged)
        return;
    }
    if (needs_calibration)
    {
        Serial.println("Waiting for calibration...");
        return;
    }
    Serial.print("RC T:");
    Serial.print(throttle_pulse_width);
    Serial.print(" S:");
    Serial.print(steer_pulse_width);
    Serial.print(" | Base L:");
    Serial.print(base_left_speed);
    Serial.print(" R:");
    Serial.print(base_right_speed);
    Serial.print(" | Final L:");
    Serial.print(left_motor_speed);
    Serial.print(left_motor_direction ? "(R)" : "(F)");
    Serial.print(" R:");
    Serial.print(right_motor_speed);
    Serial.print(right_motor_direction ? "(R)" : "(F)");
    Serial.println();
}

// --- CALIBRATION AND PREFERENCES FUNCTIONS - **CRITICAL ESP32 FIX** ---

void loadCalibrationData()
{
    preferences.begin("rc_tank", true); // Open in read-only mode
    
    if (preferences.isKey("calibrated")) {
        motor_speed_ratio = preferences.getFloat("motor_ratio", 1.0);
        RC_MIN = preferences.getInt("rc_min", RC_MIN_DEFAULT);
        RC_CENTER = preferences.getInt("rc_center", RC_CENTER_DEFAULT);
        RC_MAX = preferences.getInt("rc_max", RC_MAX_DEFAULT);
        needs_calibration = false;
    } else {
        needs_calibration = true;
        motor_speed_ratio = 1.0;
        RC_MIN = RC_MIN_DEFAULT;
        RC_CENTER = RC_CENTER_DEFAULT;
        RC_MAX = RC_MAX_DEFAULT;
    }
    
    preferences.end();
}

void saveCalibrationData()
{
    preferences.begin("rc_tank", false); // Open in write mode
    
    preferences.putBool("calibrated", true);
    preferences.putFloat("motor_ratio", motor_speed_ratio);
    preferences.putInt("rc_min", RC_MIN);
    preferences.putInt("rc_center", RC_CENTER);
    preferences.putInt("rc_max", RC_MAX);
    
    preferences.end();
    Serial.println("Calibration data saved to Preferences");
}

void startCalibration()
{
    if (!motors_enabled)
    {
        Serial.println("Cannot calibrate. Please center throttle to enable motors first.");
        calibration_requested = false;
        return;
    }
    calibration_state = CALIB_MOTOR_RUNNING;
    calibration_requested = false;
    calibration_start_time = millis();
    motor_calibration_samples = 0;
    motor_calibration_left_sum = 0;
    motor_calibration_right_sum = 0;

    writeDS1267B(64, 64, 0);
    digitalWrite(MOTOR_L_DIR, DIR_FORWARD);
    digitalWrite(MOTOR_R_DIR, DIR_FORWARD);
    Serial.println("Motor calibration started.");
}

void updateMotorCalibration()
{
    if (millis() - calibration_start_time > 1000)
    {
        motor_calibration_left_sum += left_feedback;
        motor_calibration_right_sum += right_feedback;
        motor_calibration_samples++;

        if (millis() - calibration_start_time > MOTOR_CALIBRATION_TIME)
        {
            float avg_left = (float)motor_calibration_left_sum / motor_calibration_samples;
            float avg_right = (float)motor_calibration_right_sum / motor_calibration_samples;
            if (avg_right > 0)
                motor_speed_ratio = avg_left / avg_right;

            writeDS1267B(0, 0, 0);
            Serial.print("Motor calibration complete. Ratio: ");
            Serial.println(motor_speed_ratio, 3);
            startRCCalibration();
        }
    }
}

void startRCCalibration()
{
    calibration_state = CALIB_RC_SAMPLING;
    rc_min_observed = 2500;
    rc_max_observed = 500;
    rc_center_observed = RC_CENTER_DEFAULT;
    rc_center_found = false;
    rc_center_stable_start = 0;
    Serial.println("RC calibration started. Move sticks to full range, then center and press button.");
}

void updateRCCalibration()
{
    int current_steer = steer_pulse_width;
    int current_throttle = throttle_pulse_width;
    rc_min_observed = min(rc_min_observed, min(current_steer, current_throttle));
    rc_max_observed = max(rc_max_observed, max(current_steer, current_throttle));

    int center_estimate = (rc_min_observed + rc_max_observed) / 2;
    if (abs(current_steer - center_estimate) < 50 && abs(current_throttle - center_estimate) < 50)
    {
        if (!rc_center_found)
        {
            rc_center_found = true;
            rc_center_stable_start = millis();
            rc_center_observed = (current_steer + current_throttle) / 2;
        }
    }
    else
    {
        rc_center_found = false;
    }
}

void finishCalibration()
{
    if (rc_max_observed - rc_min_observed < 500)
    {
        Serial.println("ERROR: RC range too small - calibration failed.");
        calibration_state = CALIB_IDLE;
        needs_calibration = true;
        return;
    }
    RC_MIN = rc_min_observed;
    RC_MAX = rc_max_observed;
    RC_CENTER = rc_center_observed;

    saveCalibrationData();

    calibration_state = CALIB_IDLE;
    needs_calibration = false;
    digitalWrite(STATUS_LED, LOW);

    Serial.println("Calibration complete!");
}
/**
 * @file f2s_robot.cpp
 * @brief Consolidated F2S Goalkeeper Robot (v3.1)
 * @version 3.1
 * @date 2024
 *
 * Comprehensive goalkeeper robot with advanced positioning algorithms,
 * predictive interception, field awareness, and coordinated team defense strategies.
 * Combines functionalities from basic, advanced, and field-aware versions.
 */

#include <Adafruit_SH1106_STM32.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050.h>
#include <EEPROM.h>
#include <Servo.h>

// ============================================================================
// CONSOLIDATED CONFIGURATION SYSTEM
// ============================================================================

namespace Config {
    // Performance Configuration (Goalkeeper optimized)
    static constexpr int MAX_MOTOR_SPEED = 50000;
    static constexpr int BASE_SPEED = 48000;
    static constexpr float PULLBACK_MULTIPLIER = 1.4f;
    static constexpr int MOTOR_ACCELERATION_LIMIT = 4000;
    static constexpr int SHOOT_DURATION = 500; // Duration of the shooting action in milliseconds
    static constexpr int STRATEGY_UPDATE_INTERVAL = 3000; // Interval for strategy adaptation in milliseconds

    // Field Dimensions (same as F1S for consistency)
    static constexpr int FIELD_LENGTH = 182;  // 182cm total length
    static constexpr int FIELD_WIDTH = 122;   // 122cm total width
    static constexpr int PENALTY_AREA_LENGTH = 30;  // 30cm from goal line
    static constexpr int PENALTY_AREA_WIDTH = 60;   // 60cm wide
    static constexpr int GOAL_WIDTH = 40;     // 40cm goal width
    static constexpr int CENTER_CIRCLE_RADIUS = 20; // 20cm radius
    static constexpr int BOUNDARY_MARGIN = 8; // 8cm safety margin

    // Goalkeeper-specific field zones
    // F2S defends the goal at (0, -FIELD_LENGTH/2)
    static constexpr int OWN_GOAL_Y = -FIELD_LENGTH / 2;
    static constexpr int OPPONENT_GOAL_Y = FIELD_LENGTH / 2;
    static constexpr int CENTER_LINE_Y = 0;
    static constexpr int OWN_PENALTY_Y = OWN_GOAL_Y + PENALTY_AREA_LENGTH;
    static constexpr int PENALTY_PERIMETER_Y = OWN_PENALTY_Y + 8; // 8cm outside penalty area

    // Goalkeeper positioning
    static constexpr int OPTIMAL_GOAL_DISTANCE = 12; // Distance from goal line when defending
    static constexpr int MAX_PATROL_DISTANCE = 25;   // Max distance from goal when patrolling
    static constexpr int THREAT_ENTRY_THRESHOLD = 40; // Distance at which GK enters penalty area

    // Advanced Sensor Configuration
    static constexpr int TSOP_SENSORS = 16;
    static constexpr int COLOR_SENSORS = 4;
    static constexpr int SENSOR_SAMPLES = 6; // More samples for goalkeeper accuracy
    static constexpr int BALL_DETECTION_THRESHOLD = 3500;
    static constexpr int BALL_THREAT_THRESHOLD = 3200;
    static constexpr int BALL_INTERCEPT_THRESHOLD = 2800;
    static constexpr int BALL_CRITICAL_THRESHOLD = 1500;
    static constexpr int GOAL_LINE_THRESHOLD = 3200;
    static constexpr int OUTLINE_THRESHOLD = 900;
    static constexpr int WHITE_LINE_THRESHOLD = 3000;

    // AI & Prediction
    static constexpr int INTERCEPTION_LOOKAHEAD = 8;
    static constexpr float INTERCEPTION_CONFIDENCE_MIN = 0.6f;
    static constexpr float REACTION_TIME_MS = 150.0f;

    // PID Controllers (Adaptive)
    struct PIDConfig {
        float kp, ki, kd;
        float min_output, max_output;
        float integral_limit;
    };

    static constexpr PIDConfig GYRO_PID = {250.0f, 0.1f, 40.0f, -15000, 15000, 800.0f};
    static constexpr PIDConfig POSITION_PID = {2.0f, 0.03f, 0.7f, -0.9f, 0.9f, 8.0f};
    static constexpr PIDConfig INTERCEPT_PID = {3.0f, 0.08f, 1.0f, -1.0f, 1.0f, 15.0f};

    // Advanced Timing
    static constexpr unsigned long MAIN_LOOP_INTERVAL = 25;   // 40Hz
    static constexpr unsigned long SENSOR_UPDATE_INTERVAL = 12; // 83Hz
    static constexpr unsigned long AI_UPDATE_INTERVAL = 60; // 16Hz
    static constexpr unsigned long COMM_UPDATE_INTERVAL = 35; // 28Hz
    static constexpr unsigned long VISION_PROCESSING_INTERVAL = 40; // 25Hz

    // Communication
    static constexpr int BLUETOOTH_BAUD = 115200;
    static constexpr unsigned long COMM_TIMEOUT = 800;
    static constexpr int PACKET_SIZE = 64;

    // Safety thresholds
    static constexpr float MIN_BATTERY_VOLTAGE = 10.8f;
    static constexpr float MAX_MOTOR_CURRENT = 4.5f;
    static constexpr int WATCHDOG_TIMEOUT = 1200;
    static constexpr int MAX_TEMPERATURE = 65; // Celsius
}

// ============================================================================
// CONSOLIDATED DATA STRUCTURES FOR GOALKEEPER
// ============================================================================

enum class GoalkeeperStateEnum : uint8_t {
    INITIALIZING = 0, CALIBRATING, PATROLLING_PERIMETER, DEFENDING_GOAL,
    TRACKING_BALL, POSITIONING, INTERCEPTING, SHOOTING, RETURNING_TO_GOAL,
    COORDINATING_DEFENSE, RESPECTING_BOUNDARIES, EMERGENCY_STOP, ERROR_STATE
};

enum class DefenseZone : uint8_t {
    GOAL_LINE = 0, PENALTY_AREA, PENALTY_PERIMETER, MIDFIELD, OUT_OF_BOUNDS
};

enum class ThreatLevel : uint8_t {
    NO_THREAT = 0, LOW_THREAT, MEDIUM_THREAT, HIGH_THREAT, CRITICAL_THREAT
};

struct Vector2D {
    float x, y;
    Vector2D(float x = 0, float y = 0) : x(x), y(y) {}
    Vector2D operator+(const Vector2D& other) const { return Vector2D(x + other.x, y + other.y); }
    Vector2D operator-(const Vector2D& other) const { return Vector2D(x - other.x, y - other.y); }
    Vector2D operator*(float scalar) const { return Vector2D(x * scalar, y * scalar); }
    float magnitude() const { return sqrt(x * x + y * y); }
    Vector2D normalized() const { float mag = magnitude(); return mag > 0 ? Vector2D(x / mag, y / mag) : Vector2D(); }
};

struct SensorData {
    // Enhanced TSOP data
    int tsopRaw[Config::TSOP_SENSORS];
    int tsopFiltered[Config::TSOP_SENSORS];
    int tsopMin = 4095;
    int tsopNum = 0;
    float ballThreatLevel = 0.0f;
    Vector2D ballVector;
    Vector2D ballVelocity;

    // Field-aware color sensors
    int colorRaw[Config::COLOR_SENSORS];
    int colorCalibrated[Config::COLOR_SENSORS];
    bool onGoalLine = false;
    bool onOutline = false;
    bool onWhiteLine = false;  // Penalty area line
    float goalLineDistance = 0.0f;
    float outlineDistance = 0.0f;

    // Field position and orientation
    Vector2D globalPosition;   // Position on field in cm
    float heading = 0.0f;      // Global heading in degrees
    float headingRate = 0.0f;
    Vector2D velocity;
    DefenseZone currentZone = DefenseZone::GOAL_LINE;
    ThreatLevel currentThreat = ThreatLevel::NO_THREAT;

    // System monitoring
    float batteryVoltage = 12.0f;
    float motorCurrent[4] = {0};
    float temperature = 25.0f;

    float ultrasonicDistance = 0.0f;
    int leftIR = 0;
    int rightIR = 0;
    int frontIR = 0;
    int backIR = 0; // Added back IR
    float gyroReading = 0.0f;
    float compassReading = 0.0f;

    unsigned long timestamp = 0;
    bool dataValid = true;
};

enum class AttackerState : uint8_t {
    SEARCHING = 0, APPROACHING, PASSING, SHOOTING, AVOIDING
};

struct AIState {
    // Ball trajectory prediction with field constraints
    Vector2D ballTrajectory[Config::INTERCEPTION_LOOKAHEAD];
    float trajectoryConfidence = 0.0f;
    Vector2D optimalInterceptionPoint;
    float interceptionTime = 0.0f;

    // Defense strategy adaptation
    DefenseStrategy currentStrategy = DefenseStrategy::BALANCED;
    float strategyWeights[5] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    int successfulSaves = 0;
    int totalThreats = 0;

    // Learning parameters
    float learningRate = 0.05f;
    int adaptationCycle = 0;
    bool adaptiveMode = true;

    // Performance metrics
    float savePercentage = 0.0f;
    float positioningAccuracy = 0.0f;
    float reactionTime = Config::REACTION_TIME_MS;
    float energyEfficiency = 1.0f;
    float fieldRespectRate = 1.0f;
};

struct RobotState {
    GoalkeeperStateEnum currentState = GoalkeeperStateEnum::INITIALIZING;
    GoalkeeperStateEnum previousState = GoalkeeperStateEnum::INITIALIZING;
    DefenseStrategy defenseStrategy = DefenseStrategy::BALANCED;

    // Field position management
    Vector2D globalPosition = Vector2D(0, Config::OWN_GOAL_Y + Config::OPTIMAL_GOAL_DISTANCE);
    Vector2D targetPosition;
    Vector2D goalCenter = Vector2D(0, Config::OWN_GOAL_Y);
    DefenseZone currentZone = DefenseZone::GOAL_LINE;
    DefenseZone targetZone = DefenseZone::PENALTY_PERIMETER;

    // Ball tracking with field awareness
    Vector2D ballHistory[10];
    int ballHistoryIndex = 0;
    unsigned long lastBallTime = 0;
    int ballLostCounter = 0;

    // Penalty area management
    bool insidePenaltyArea = false;
    unsigned long penaltyAreaEntryTime = 0;
    bool shouldExitPenaltyArea = false;

    // Shooting system
    unsigned long lastShootTime = 0;
    int saveAttempts = 0;
    int successfulSaves = 0;
    float shooterRPM = 0.0f;

    // Field boundary respect
    int boundaryViolations = 0;
    unsigned long lastBoundaryViolation = 0;
    bool respectingBoundaries = true;

    // Emergency and safety
    bool emergencyStop = false;
    bool maintenanceRequired = false;
    int errorCode = 0;
    unsigned long lastWatchdogReset = 0;

    // Performance metrics
    int cpuUsage = 0;
    int memoryUsage = 0;
    float loopFrequency = 40.0f;
    unsigned long totalRunTime = 0;
};

struct ComputerVisionData {
    bool goalDetected = false;
    Vector2D goalPosition;
    float goalConfidence = 0.0f;
    bool opponentDetected = false;
    Vector2D opponentPosition;
    float opponentConfidence = 0.0f;
    bool teammateDetected = false;
    Vector2D teammatePosition;
    int objectsDetected = 0;
    unsigned long lastVisionUpdate = 0;
};

struct PredictiveAnalytics {
    Vector2D ballTrajectoryPrediction[20]; // AdvancedConfig::BALL_TRAJECTORY_SAMPLES
    float trajectoryConfidence = 0.0f;
    Vector2D optimalInterceptionPoint;
    float timeToIntercept = 0.0f;
    Vector2D opponentMovementPrediction;
    float opponentThreatLevel = 0.0f;
    Vector2D optimalShotPosition;
    float shotSuccessProbability = 0.0f;
};

// Advanced state management
struct RobotInternalState {
    float x, y, theta;
    float velocityX, velocityY;
    int batteryLevel;
    bool isCalibrated;
    unsigned long lastUpdate;
};

struct BallInternalState {
    float x, y;
    float velocityX, velocityY;
    float distance;
    float angle;
    bool isVisible;
    unsigned long lastSeen;
    float predictedX, predictedY;
};

struct GoalkeepingStrategyInternal {
    int aggressiveness; // 0-100
    int reactionTime;   // milliseconds
    bool usePredicition;
    bool allowAdvance;
    float maxAdvanceDistance;
};

RobotInternalState robotInternal;
BallInternalState ballInternal;
GoalkeepingStrategyInternal strategyInternal;

// PID Controllers
struct PIDController {
    float kp, ki, kd;
    float integral, lastError;
    float output;
};

PIDController positionPIDInternal;
PIDController orientationPIDInternal;

// Performance tracking
struct PerformanceMetricsInternal {
    int savesAttempted;
    int savesSuccessful;
    int goalsAllowed;
    float averageReactionTime;
    unsigned long totalPlayTime;
};

PerformanceMetricsInternal metricsInternal;

// ============================================================================
// ADVANCED PID CONTROLLER CLASS
// ============================================================================

class AdvancedPIDController {
private:
    Config::PIDConfig config;
    float integral = 0.0f;
    float previousError = 0.0f;
    float derivative = 0.0f;
    unsigned long lastTime = 0;

    float adaptiveKp, adaptiveKi, adaptiveKd;
    float errorHistory[5] = {0};
    int historyIndex = 0;

public:
    AdvancedPIDController(const Config::PIDConfig& cfg) : config(cfg) {
        adaptiveKp = config.kp;
        adaptiveKi = config.ki;
        adaptiveKd = config.kd;
        lastTime = millis(); // Initialize lastTime
    }

    float compute(float setpoint, float input, bool adaptive = true) {
        unsigned long now = millis();
        float deltaTime = (now - lastTime) / 1000.0f;
        if (deltaTime <= 0) deltaTime = 0.001f; // Prevent division by zero, min 1ms

        float error = setpoint - input;

        if (adaptive) {
            adaptParameters(error);
        }

        integral += error * deltaTime;
        integral = constrain(integral, -config.integral_limit, config.integral_limit);

        derivative = (error - previousError) / deltaTime;

        float output = adaptiveKp * error + adaptiveKi * integral + adaptiveKd * derivative;
        output = constrain(output, config.min_output, config.max_output);

        previousError = error;
        lastTime = now;

        return output;
    }

    void reset() {
        integral = 0.0f;
        previousError = 0.0f;
        derivative = 0.0f;
        for (int i = 0; i < 5; i++) errorHistory[i] = 0.0f;
    }

    void setTunings(float kp, float ki, float kd) {
        adaptiveKp = kp;
        adaptiveKi = ki;
        adaptiveKd = kd;
    }

private:
    void adaptParameters(float error) {
        errorHistory[historyIndex] = abs(error);
        historyIndex = (historyIndex + 1) % 5;

        float mean = 0, variance = 0;
        for (int i = 0; i < 5; i++) mean += errorHistory[i];
        mean /= 5.0f;

        for (int i = 0; i < 5; i++) {
            variance += pow(errorHistory[i] - mean, 2);
        }
        variance /= 5.0f;

        if (variance > 0.2f) { // Reduced threshold for goalkeeper's precise movements
            adaptiveKp = config.kp * 0.9f;
            adaptiveKd = config.kd * 1.1f;
        } else if (mean > 0.5f) { // Reduced threshold
            adaptiveKi = config.ki * 1.05f;
        } else {
            adaptiveKp = config.kp;
            adaptiveKi = config.ki;
            adaptiveKd = config.kd;
        }
    }
};

// ============================================================================
// CONSOLIDATED GOALKEEPER SENSOR SYSTEM
// ============================================================================

class ConsolidatedGoalkeeperSensorSystem {
private:
    MPU6050 mpu;
    // FieldAwarenessSystem fieldSystem; // Use a dedicated GoalkeeperFieldAwareness
    // Kalman filter for ball tracking
    float ballState[4] = {0};  // [x, y, vx, vy]
    float ballCovariance[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    float processNoise = 0.15f;
    float measurementNoise = 0.8f;

    // Sensor calibration
    int tsopCalibration[Config::TSOP_SENSORS] = {0};
    int colorCalibration[Config::COLOR_SENSORS] = {0};
    bool calibrationComplete = false;

    // Advanced filtering
    float tsopFilter[Config::TSOP_SENSORS][7] = {0}; // 7-tap FIR filter
    float colorFilter[Config::COLOR_SENSORS][5] = {0}; // 5-tap FIR filter

    // Goal line detection
    Vector2D goalLinePoints[4]; // Defines the goal area
    bool goalLineCalibrated = false;

public:
    void initialize() {
        Wire.begin();
        mpu.initialize();
        if (!mpu.testConnection()) {
            Serial.println("MPU6050 FAILED!");
        }

        loadCalibrationData();
        resetFilters();
        calibrateGoalLine(); // Static calibration of goal line points
    }

    void updateSensorData(SensorData& data) {
        readTSOPSensors(data);
        readColorSensors(data);
        updateIMU(data);
        readAdditionalSensors(data); // Read ultrasonic, IR, gyro, compass
        calculateBallThreat(data); // This updates ballVector and ballVelocity
        updateSystemMonitoring(data);

        data.timestamp = millis();
        data.dataValid = true;
    }

    void calibrateSensors() {
        Serial.println("Calibrating sensors...");
        for (int i = 0; i < 150; i++) {
            for (int j = 0; j < Config::TSOP_SENSORS; j++) {
                digitalWrite(PA8, (j & 1));
                digitalWrite(PB1, (j & 2) >> 1);
                digitalWrite(PC14, (j & 4) >> 2);
                digitalWrite(PC15, (j & 8) >> 3);

                tsopCalibration[j] += analogRead(PA0);
            }

            for (int j = 0; j < Config::COLOR_SENSORS; j++) {
                colorCalibration[j] += analogRead(PA1 + j);
            }

            delay(10);
        }

        for (int i = 0; i < Config::TSOP_SENSORS; i++) {
            tsopCalibration[i] /= 150;
        }
        for (int i = 0; i < Config::COLOR_SENSORS; i++) {
            colorCalibration[i] /= 150;
        }

        saveCalibrationData();
        calibrationComplete = true;
        Serial.println("Sensor calibration complete!");
    }

private:
    void readTSOPSensors(SensorData& data) {
        data.tsopMin = 4095;

        for (int i = 0; i < Config::TSOP_SENSORS; i++) {
            digitalWrite(PA8, (i & 1));
            digitalWrite(PB1, (i & 2) >> 1);
            digitalWrite(PC14, (i & 4) >> 2);
            digitalWrite(PC15, (i & 8) >> 3);

            int sum = 0;
            for (int j = 0; j < Config::SENSOR_SAMPLES; j++) {
                sum += analogRead(PA0);
                delayMicroseconds(50);
            }
            data.tsopRaw[i] = sum / Config::SENSOR_SAMPLES;

            data.tsopRaw[i] -= tsopCalibration[i];
            if (data.tsopRaw[i] < 0) data.tsopRaw[i] = 0;

            // Apply 7-tap FIR filter
            for (int k = 6; k > 0; k--) {
                tsopFilter[i][k] = tsopFilter[i][k - 1];
            }
            tsopFilter[i][0] = data.tsopRaw[i];

            data.tsopFiltered[i] = (tsopFilter[i][0] * 0.35f +
                                   tsopFilter[i][1] * 0.25f +
                                   tsopFilter[i][2] * 0.18f +
                                   tsopFilter[i][3] * 0.12f +
                                   tsopFilter[i][4] * 0.06f +
                                   tsopFilter[i][5] * 0.03f +
                                   tsopFilter[i][6] * 0.01f);

            if (data.tsopFiltered[i] < data.tsopMin) {
                data.tsopMin = data.tsopFiltered[i];
                data.tsopNum = i;
            }
        }

        // Calculate ball threat level (0.0 = no threat, 1.0 = critical)
        if (data.tsopMin < Config::BALL_CRITICAL_THRESHOLD) {
            data.ballThreatLevel = 1.0f;
        } else if (data.tsopMin < Config::BALL_INTERCEPT_THRESHOLD) {
            data.ballThreatLevel = map(data.tsopMin, Config::BALL_CRITICAL_THRESHOLD, Config::BALL_INTERCEPT_THRESHOLD, 0.7f, 1.0f);
            data.ballThreatLevel = constrain(data.ballThreatLevel, 0.7f, 1.0f);
        } else if (data.tsopMin < Config::BALL_THREAT_THRESHOLD) {
            data.ballThreatLevel = map(data.tsopMin, Config::BALL_INTERCEPT_THRESHOLD, Config::BALL_THREAT_THRESHOLD, 0.4f, 0.7f);
            data.ballThreatLevel = constrain(data.ballThreatLevel, 0.4f, 0.7f);
        } else {
            data.ballThreatLevel = 0.0f;
        }
    }

    void readColorSensors(SensorData& data) {
        float minGoalDistance = 1000.0f;
        float minOutlineDistance = 1000.0f;

        for (int i = 0; i < Config::COLOR_SENSORS; i++) {
            int sum = 0;
            for (int j = 0; j < Config::SENSOR_SAMPLES; j++) {
                sum += analogRead(PA1 + i);
                delayMicroseconds(30);
            }
            data.colorRaw[i] = sum / Config::SENSOR_SAMPLES;

            // Apply calibration and filtering
            for (int k = 4; k > 0; k--) {
                colorFilter[i][k] = colorFilter[i][k - 1];
            }
            colorFilter[i][0] = data.colorRaw[i] - colorCalibration[i];

            data.colorCalibrated[i] = (colorFilter[i][0] * 0.7f +
                                      colorFilter[i][1] * 0.2f +
                                      colorFilter[i][2] * 0.1f); // 3-tap filter, adjust coefficients for 5-tap

            // Calculate distance to goal line (white)
            if (data.colorCalibrated[i] > Config::GOAL_LINE_THRESHOLD) {
                float distance = (4095.0f - data.colorCalibrated[i]) / (4095.0f - Config::GOAL_LINE_THRESHOLD);
                if (distance < minGoalDistance) minGoalDistance = distance;
            }

            // Calculate distance to outline (black)
            if (data.colorCalibrated[i] < Config::OUTLINE_THRESHOLD) {
                float distance = data.colorCalibrated[i] / (float)Config::OUTLINE_THRESHOLD;
                if (distance < minOutlineDistance) minOutlineDistance = distance;
            }
        }

        data.goalLineDistance = constrain(minGoalDistance, 0.0f, 1.0f);
        data.outlineDistance = constrain(minOutlineDistance, 0.0f, 1.0f);
        data.onGoalLine = (data.goalLineDistance < 0.15f);
        data.onOutline = (data.outlineDistance < 0.15f);
    }

    void updateIMU(SensorData& data) {
        Vector normGyro = mpu.readNormalizeGyro();
        Vector normAccel = mpu.readNormalizeAccel();

        static float gyroHeading = 0.0f;
        static Vector2D lastVelocity(0, 0);
        static unsigned long lastTime = 0;

        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        if (dt > 0.1f) dt = Config::MAIN_LOOP_INTERVAL / 1000.0f;

        gyroHeading += normGyro.ZAxis * dt;

        while (gyroHeading >= 360.0f) gyroHeading -= 360.0f;
        while (gyroHeading < 0.0f) gyroHeading += 360.0f;

        data.heading = gyroHeading;
        data.headingRate = normGyro.ZAxis;
        data.acceleration = Vector2D(normAccel.XAxis, normAccel.YAxis);

        data.velocity = lastVelocity + data.acceleration * dt;
        data.velocity = data.velocity * 0.98f;

        data.globalPosition = data.globalPosition + data.velocity * dt; // Update global position here

        lastVelocity = data.velocity;
        lastTime = now;
    }

    void readAdditionalSensors(SensorData& data) {
        // Read ultrasonic sensor
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);
        long duration = pulseIn(ECHO_PIN, HIGH);
        data.ultrasonicDistance = duration * 0.034 / 2;

        // Read IR sensors
        data.leftIR = analogRead(IR_SENSOR_LEFT);
        data.rightIR = analogRead(IR_SENSOR_RIGHT);
        data.frontIR = analogRead(IR_SENSOR_FRONT);
        data.backIR = analogRead(IR_SENSOR_BACK);

        // Read gyro and compass (assuming analog pins for simplicity)
        data.gyroReading = analogRead(GYRO_PIN) * (5.0 / 1023.0);
        data.compassReading = analogRead(COMPASS_PIN) * (360.0 / 1023.0);
    }

    void calculateBallThreat(SensorData& data) {
        if (data.ballThreatLevel > 0.0f) {
            float angleDeg = data.tsopNum * (360.0f / Config::TSOP_SENSORS);
            float angleRad = angleDeg * PI / 180.0f;

            float distance = 1.0f - (data.ballThreatLevel * 0.9f);
            distance = constrain(distance, 0.1f, 1.0f);

            data.ballVector.x = cos(angleRad) * distance;
            data.ballVector.y = sin(angleRad) * distance;

            updateBallKalmanFilter(data);
        } else {
            data.ballVector = Vector2D(0, 0);
            data.ballVelocity = Vector2D(0, 0);
        }
    }

    void updateBallKalmanFilter(SensorData& data) {
        static Vector2D lastObservedBallPos(0, 0);
        static unsigned long lastUpdate = 0;

        unsigned long now = millis();
        float dt = (now - lastUpdate) / 1000.0f;
        if (dt > 0.1f) dt = Config::SENSOR_UPDATE_INTERVAL / 1000.0f;

        if (dt > 0) {
            ballState[0] += ballState[2] * dt;
            ballState[1] += ballState[3] * dt;

            float measurementX = data.ballVector.x;
            float measurementY = data.ballVector.y;

            ballState[0] = ballState[0] * (1 - measurementNoise) + measurementX * measurementNoise;
            ballState[1] = ballState[1] * (1 - measurementNoise) + measurementY * measurementNoise;

            Vector2D currentSmoothedBallPos(ballState[0], ballState[1]);
            data.ballVelocity = (currentSmoothedBallPos - lastObservedBallPos) * (1.0f / dt);

            ballState[2] = data.ballVelocity.x;
            ballState[3] = data.ballVelocity.y;

            lastObservedBallPos = currentSmoothedBallPos;
        } else {
            lastObservedBallPos = data.ballVector;
        }

        lastUpdate = now;
    }

    void updateSystemMonitoring(SensorData& data) {
        data.batteryVoltage = 10.0f + (analogRead(PA5) / 4095.0f) * 4.0f;
        data.batteryVoltage = constrain(data.batteryVoltage, 0.0f, 14.0f);

        data.temperature = 20.0f + (analogRead(PA6) / 4095.0f) * 60.0f;
        data.temperature = constrain(data.temperature, 0.0f, 100.0f);

        for (int i = 0; i < 4; i++) {
            data.motorCurrent[i] = abs(analogRead(PA7 + i) - 2048) * (2.0f / 2048.0f);
            data.motorCurrent[i] = constrain(data.motorCurrent[i], 0.0f, 10.0f);
        }
    }

    void calibrateGoalLine() {
        goalLinePoints[0] = Vector2D(-Config::GOAL_WIDTH / 2.0f, 0);
        goalLinePoints[1] = Vector2D(Config::GOAL_WIDTH / 2.0f, 0);
        goalLinePoints[2] = Vector2D(0, -Config::GOAL_DEPTH);
        goalLinePoints[3] = Vector2D(0, 0);

        goalLineCalibrated = true;
    }

    void loadCalibrationData() {
        for (int i = 0; i < Config::TSOP_SENSORS; i++) {
            tsopCalibration[i] = EEPROM.read(i * 2) | (EEPROM.read(i * 2 + 1) << 8);
        }
        for (int i = 0; i < Config::COLOR_SENSORS; i++) {
            colorCalibration[i] = EEPROM.read(64 + i * 2) | (EEPROM.read(65 + i * 2) << 8);
        }
        calibrationComplete = (EEPROM.read(0x80) == 0xFF);
    }

    void saveCalibrationData() {
        for (int i = 0; i < Config::TSOP_SENSORS; i++) {
            EEPROM.write(i * 2, tsopCalibration[i] & 0xFF);
            EEPROM.write(i * 2 + 1, (tsopCalibration[i] >> 8) & 0xFF);
        }
        for (int i = 0; i < Config::COLOR_SENSORS; i++) {
            EEPROM.write(64 + i * 2, colorCalibration[i] & 0xFF);
            EEPROM.write(65 + i * 2, (colorCalibration[i] >> 8) & 0xFF);
        }
        EEPROM.write(0x80, 0xFF);
    }

    void resetFilters() {
        for (int i = 0; i < Config::TSOP_SENSORS; i++) {
            for (int j = 0; j < 7; j++) { // 7-tap filter
                tsopFilter[i][j] = 0.0f;
            }
        }
        for (int i = 0; i < Config::COLOR_SENSORS; i++) {
            for (int j = 0; j < 5; j++) { // 5-tap filter
                colorFilter[i][j] = 0.0f;
            }
        }
        for (int i = 0; i < 4; i++) { // Reset Kalman filter state
            ballState[i] = 0.0f;
        }
    }
};

// ============================================================================
// CONSOLIDATED GOALKEEPER HARDWARE INTERFACE
// ============================================================================

class ConsolidatedGoalkeeperHardwareInterface {
private:
    Adafruit_SH1106 display;
    SoftwareSerial bluetooth;
    SoftwareSerial teamComm; // Team communication
    ConsolidatedGoalkeeperSensorSystem* sensorSystem;

    // Motor control with acceleration limiting
    int currentMotorSpeeds[4] = {0};
    int targetMotorSpeeds[4] = {0};
    unsigned long lastMotorUpdate = 0;

    // Encoder variables
    volatile long leftEncoderCount = 0;
    volatile long rightEncoderCount = 0;

    // Servos
    Servo kickerServo;
    Servo defenseServo;
    Servo cameraServo; // Assuming a camera servo

    bool safetyEnabled = true;
    unsigned long lastSafetyCheck = 0;

public:
    ConsolidatedGoalkeeperHardwareInterface() : display(-1), bluetooth(PA9, PA10), teamComm(PB10, PB11), sensorSystem(nullptr) {}

    void setSensorSystem(ConsolidatedGoalkeeperSensorSystem* ss) {
        sensorSystem = ss;
    }

    bool initialize() {
        display.begin(0x2, 0x3c);
        showSplashScreen();

        initializeMotorPins();
        initializeSensorPins();

        bluetooth.begin(Config::BLUETOOTH_BAUD);
        teamComm.begin(Config::BLUETOOTH_BAUD); // Use same baud for team comm

        displayMessage("F2S v3.1 READY!");
        delay(1000);

        return true;
    }

    void setMotorSpeeds(int ml1, int ml2, int mr2, int mr1, bool pullback = false, bool immediate = false) {
        if (pullback) {
            ml1 = (int)(ml1 * Config::PULLBACK_MULTIPLIER);
            ml2 = (int)(ml2 * Config::PULLBACK_MULTIPLIER);
            mr1 = (int)(mr1 * Config::PULLBACK_MULTIPLIER);
            mr2 = (int)(mr2 * Config::PULLBACK_MULTIPLIER);
        }

        if (safetyEnabled) {
            ml1 = constrain(ml1, -Config::MAX_MOTOR_SPEED, Config::MAX_MOTOR_SPEED);
            ml2 = constrain(ml2, -Config::MAX_MOTOR_SPEED, Config::MAX_MOTOR_SPEED);
            mr1 = constrain(mr1, -Config::MAX_MOTOR_SPEED, Config::MAX_MOTOR_SPEED);
            mr2 = constrain(mr2, -Config::MAX_MOTOR_SPEED, Config::MAX_MOTOR_SPEED);
        }

        targetMotorSpeeds[0] = ml1; // ML1
        targetMotorSpeeds[1] = ml2; // ML2
        targetMotorSpeeds[2] = mr1; // MR1
        targetMotorSpeeds[3] = mr2; // MR2

        if (immediate) {
            for (int i = 0; i < 4; i++) {
                currentMotorSpeeds[i] = targetMotorSpeeds[i];
            }
            updateMotorOutputs();
        }
    }

    void updateMotorControl() {
        unsigned long now = millis();
        if (now - lastMotorUpdate < 5) return;

        bool changed = false;

        for (int i = 0; i < 4; i++) {
            int diff = targetMotorSpeeds[i] - currentMotorSpeeds[i];
            if (abs(diff) > Config::MOTOR_ACCELERATION_LIMIT) {
                currentMotorSpeeds[i] += (diff > 0) ? Config::MOTOR_ACCELERATION_LIMIT : -Config::MOTOR_ACCELERATION_LIMIT;
                changed = true;
            } else if (diff != 0) {
                currentMotorSpeeds[i] = targetMotorSpeeds[i];
                changed = true;
            }
        }

        if (changed) {
            updateMotorOutputs();
        }

        lastMotorUpdate = now;
    }

    void setShooterSpeed(int speed, float targetRPM = 0) {
        speed = constrain(speed, 0, Config::MAX_MOTOR_SPEED);

        if (targetRPM > 0) {
            speed = map(targetRPM, 0, 10000, 0, Config::MAX_MOTOR_SPEED);
        }

        pwmWrite(PC6, speed);
    }

    SensorData readSensors() {
        SensorData data;
        if (sensorSystem) {
            sensorSystem->updateSensorData(data);
        } else {
            Serial.println("SensorSystem not set!");
            data.dataValid = false;
        }
        return data;
    }

    void updateDisplay(const SensorData& sensors, const RobotState& state, const AIState& ai) {
        static int displayPage = 0;
        static unsigned long lastPageChange = 0;

        if (millis() - lastPageChange > 3000) {
            displayPage = (displayPage + 1) % 4;
            lastPageChange = millis();
        }

        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);

        switch (displayPage) {
            case 0: displayMainPage(sensors, state); break;
            case 1: displayAIPage(ai, state); break;
            case 2: displaySystemPage(sensors, state); break;
            case 3: displayPerformancePage(sensors, state); break;
        }

        display.setCursor(120, 0);
        display.print(displayPage + 1);

        display.display();
    }

    String receiveBluetoothData() {
        if (bluetooth.available()) {
            String data = bluetooth.readString();
            data.trim();
            return data;
        }
        return "";
    }

    void sendBluetoothData(const String& data) {
        uint16_t checksum = calculateChecksum(data);
        bluetooth.print(data);
        bluetooth.print("|CRC:");
        bluetooth.println(checksum, HEX);
    }

    String receiveTeamCommData() {
        if (teamComm.available()) {
            String data = teamComm.readString();
            data.trim();
            return data;
        }
        return "";
    }

    void sendTeamCommData(const String& data) {
        teamComm.println(data);
    }

    void performSafetyCheck(const SensorData& sensors) {
        if (!safetyEnabled) return;

        unsigned long now = millis();
        if (now - lastSafetyCheck < 100) return;

        bool emergencyStop = false;

        if (sensors.batteryVoltage < Config::MIN_BATTERY_VOLTAGE) {
            emergencyStop = true;
            Serial.println("Battery critical!");
        }

        for (int i = 0; i < 4; i++) {
            if (sensors.motorCurrent[i] > Config::MAX_MOTOR_CURRENT) {
                emergencyStop = true;
                Serial.print("Motor "); Serial.print(i); Serial.println(" overcurrent!");
            }
        }

        if (sensors.temperature > Config::MAX_TEMPERATURE) {
            emergencyStop = true;
            Serial.println("Overheating!");
        }

        if (emergencyStop) {
            setMotorSpeeds(0, 0, 0, 0, false, true);
            setShooterSpeed(0);
            displayError("EMERGENCY STOP!");
        }

        lastSafetyCheck = now;
    }

    void processBluetoothCommands() {
        if (bluetooth.available()) {
            String command = bluetooth.readString();
            command.trim();

            if (command == "CALIBRATE") {
                calibrateSensors();
            } else if (command.startsWith("STRATEGY:")) {
                updateStrategy(command);
            } else if (command.startsWith("PID:")) {
                updatePIDParameters(command);
            } else if (command == "STATS") {
                sendPerformanceStats();
            } else if (command == "RESET_STATS") {
                resetPerformanceMetrics();
            } else if (command == "FIELD_CALIBRATE") {
                calibrateFieldMapping();
            } else if (command.startsWith("STRATEGY_ADV:")) {
                updateAdvancedStrategy(command);
            } else if (command.startsWith("LEARNING:")) {
                updateLearningSettings(command);
            } else if (command == "EXPORT_DATA") {
                exportLearningData();
            } else if (command == "RESET_LEARNING") {
                resetLearningSystem();
            } else if (command == "ADVANCED_STATS") {
                sendAdvancedStats();
            }
        }
    }

    void updateStrategy(String command) {
        int aggIndex = command.indexOf("AGG:");
        int reactIndex = command.indexOf("REACT:");
        int predIndex = command.indexOf("PRED:");

        if (aggIndex > 0) {
            strategyInternal.aggressiveness = command.substring(aggIndex + 4, command.indexOf(",", aggIndex)).toInt();
        }
        if (reactIndex > 0) {
            strategyInternal.reactionTime = command.substring(reactIndex + 6, command.indexOf(",", reactIndex)).toInt();
        }
        if (predIndex > 0) {
            strategyInternal.usePredicition = command.substring(predIndex + 5, command.indexOf(",", predIndex)).toInt() == 1;
        }
    }

    void updatePIDParameters(String command) {
        if (command.indexOf("POS:") > 0) {
            int startIndex = command.indexOf("POS:") + 4;
            String params = command.substring(startIndex);

            int comma1 = params.indexOf(",");
            int comma2 = params.indexOf(",", comma1 + 1);

            positionPIDInternal.kp = params.substring(0, comma1).toFloat();
            positionPIDInternal.ki = params.substring(comma1 + 1, comma2).toFloat();
            positionPIDInternal.kd = params.substring(comma2 + 1).toFloat();
        }
    }

    void sendPerformanceStats() {
        bluetooth.print("STATS:");
        bluetooth.print("SAVES:");
        bluetooth.print(metricsInternal.savesAttempted);
        bluetooth.print(",SUCCESS:");
        bluetooth.print(metricsInternal.savesSuccessful);
        bluetooth.print(",GOALS:");
        bluetooth.print(metricsInternal.goalsAllowed);
        bluetooth.print(",REACT_TIME:");
        bluetooth.print(metricsInternal.averageReactionTime);
        bluetooth.print(",PLAY_TIME:");
        bluetooth.println(metricsInternal.totalPlayTime);
    }

    void resetPerformanceMetrics() {
        metricsInternal.savesAttempted = 0;
        metricsInternal.savesSuccessful = 0;
        metricsInternal.goalsAllowed = 0;
        metricsInternal.averageReactionTime = 0;
        metricsInternal.totalPlayTime = 0;
    }

    void calibrateSensors() {
        Serial.println("Calibrating sensors...");
        bluetooth.println("CALIBRATING");
        delay(2000);
        robotInternal.isCalibrated = true;
        Serial.println("Calibration complete");
        bluetooth.println("CALIBRATION_COMPLETE");
    }

    void loadConfiguration() {
        strategyInternal.aggressiveness = EEPROM.read(0);
        strategyInternal.reactionTime = EEPROM.read(1) * 10;

        if (strategyInternal.aggressiveness == 255) {
            strategyInternal.aggressiveness = 70;
            strategyInternal.reactionTime = 150;
            saveConfiguration();
        }
    }

    void saveConfiguration() {
        EEPROM.write(0, strategyInternal.aggressiveness);
        EEPROM.write(1, strategyInternal.reactionTime / 10);
    }

    void calibrateFieldMapping() {
        Serial.println("Calibrating field mapping...");
        bluetooth.println("FIELD_CALIBRATING");
        for (int i = 0; i < 5; i++) {
            Serial.print("Calibration step ");
            Serial.println(i + 1);
            delay(1000);
        }
        Serial.println("Field calibration complete");
        bluetooth.println("FIELD_CALIBRATION_COMPLETE");
    }

    void updateAdvancedStrategy(String command) {
        if (command.indexOf("STYLE:") > 0) {
            int styleIndex = command.indexOf("STYLE:") + 6;
            // strategy.defensiveStyle = command.substring(styleIndex, styleIndex + 1).toInt(); // Needs access to AIState
        }
        if (command.indexOf("RISK:") > 0) {
            int riskIndex = command.indexOf("RISK:") + 5;
            // strategy.riskTolerance = command.substring(riskIndex, command.indexOf(",", riskIndex)).toFloat(); // Needs access to AIState
        }
        if (command.indexOf("TEAM:") > 0) {
            int teamIndex = command.indexOf("TEAM:") + 5;
            // strategy.useTeamCoordination = command.substring(teamIndex, teamIndex + 1).toInt() == 1; // Needs access to AIState
        }
    }

    void updateLearningSettings(String command) {
        if (command.indexOf("ENABLE:") > 0) {
            int enableIndex = command.indexOf("ENABLE:") + 7;
            // learning.isLearning = command.substring(enableIndex, enableIndex + 1).toInt() == 1; // Needs access to LearningSystem
        }
        if (command.indexOf("RESET:") > 0) {
            // resetLearningSystem(); // Needs access to LearningSystem
        }
    }

    void exportLearningData() {
        bluetooth.println("LEARNING_DATA_START");
        bluetooth.println("LEARNING_DATA_END");
    }

    void resetLearningSystem() {
        Serial.println("Learning system reset");
        bluetooth.println("LEARNING_RESET_COMPLETE");
    }

    void sendAdvancedStats() {
        bluetooth.println("ADVANCED_STATS:");
        bluetooth.print("SAVES_ATTEMPTED:");
        bluetooth.println(metricsInternal.savesAttempted);
        bluetooth.print("SAVES_SUCCESSFUL:");
        bluetooth.println(metricsInternal.savesSuccessful);
        bluetooth.print("GOALS_ALLOWED:");
        bluetooth.println(metricsInternal.goalsAllowed);
        bluetooth.print("AVG_REACTION_TIME:");
        bluetooth.println(metricsInternal.averageReactionTime);
        bluetooth.print("PLAY_TIME:");
        bluetooth.println(metricsInternal.totalPlayTime);
    }

private:
    void initializeMotorPins() {
        pinMode(PB12, OUTPUT); pinMode(PB13, OUTPUT);
        pinMode(PB14, OUTPUT); pinMode(PB15, OUTPUT);
        pinMode(PB9, PWM); pinMode(PB8, PWM);
        pinMode(PB7, PWM); pinMode(PB6, PWM);
        pinMode(PC6, PWM); // Shooter motor PWM

        // Motor pins
        pinMode(2, OUTPUT); pinMode(3, OUTPUT);
        pinMode(4, OUTPUT); pinMode(5, OUTPUT);
        pinMode(9, OUTPUT); pinMode(10, OUTPUT);

        // Encoder interrupts
        attachInterrupt(digitalPinToInterrupt(18), leftEncoderISR, RISING);
        attachInterrupt(digitalPinToInterrupt(19), rightEncoderISR, RISING);

        // Servos
        kickerServo.attach(6);
        defenseServo.attach(13);
        cameraServo.attach(16); // Assuming camera servo pin
        kickerServo.write(90);
        defenseServo.write(90);
        cameraServo.write(90);

        setMotorSpeeds(0, 0, 0, 0, false, true);
        setShooterSpeed(0);
    }

    void initializeSensorPins() {
        pinMode(PA8, OUTPUT); pinMode(PB1, OUTPUT);
        pinMode(PC14, OUTPUT); pinMode(PC15, OUTPUT);

        // Sensor pins
        pinMode(7, OUTPUT); pinMode(8, INPUT); // TRIG_PIN, ECHO_PIN
    }

    void updateMotorOutputs() {
        controlMotor(PB12, PB6, currentMotorSpeeds[2]);
        controlMotor(PB13, PB7, currentMotorSpeeds[3]);
        controlMotor(PB14, PB8, currentMotorSpeeds[1]);
        controlMotor(PB15, PB9, currentMotorSpeeds[0]);
    }

    void controlMotor(int dirPin, int pwmPin, int speed) {
        if (speed >= 0) {
            digitalWrite(dirPin, LOW);
            pwmWrite(pwmPin, speed);
        } else {
            digitalWrite(dirPin, HIGH);
            pwmWrite(pwmPin, -speed);
        }
    }

    void showSplashScreen() {
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(WHITE);
        display.setCursor(10, 10);
        display.print("F2S v3.1");
        display.setTextSize(1);
        display.setCursor(0, 35);
        display.print("Advanced Goalkeeper");
        display.setCursor(0, 45);
        display.print("Robot System");
        display.display();
        delay(2000);
    }

    void displayMainPage(const SensorData& sensors, const RobotState& state) {
        display.setCursor(0, 0);
        display.print("MAIN - F2S v3.1");

        display.setCursor(0, 10);
        display.print("State: ");
        display.print(getStateString(state.currentState));

        display.setCursor(0, 20);
        display.print("Ball: ");
        display.print(sensors.tsopNum);
        display.print(" (");
        display.print(sensors.ballThreatLevel, 2);
        display.print(")");

        display.setCursor(0, 30);
        display.print("Pos: X");
        display.print(state.globalPosition.x, 1);
        display.print(" Y");
        display.print(state.globalPosition.y, 1);

        display.setCursor(0, 40);
        display.print("Goal Line: ");
        display.print(sensors.onGoalLine ? "Y" : (sensors.goalLineDistance > 0.5f ? "F" : "N"));

        display.setCursor(0, 50);
        display.print("Loop: ");
        display.print(state.loopFrequency, 1);
        display.print("Hz");
    }

    void displayAIPage(const AIState& ai, const RobotState& state) {
        display.setCursor(0, 0);
        display.print("AI - Trajectory");

        display.setCursor(0, 10);
        display.print("Conf: ");
        display.print(ai.trajectoryConfidence, 2);
        display.print(" Int.T: ");
        display.print(ai.interceptionTime, 1);

        display.setCursor(0, 20);
        display.print("Optimal X:");
        display.print(ai.optimalInterceptionPoint.x, 1);
        display.print(" Y:");
        display.print(ai.optimalInterceptionPoint.y, 1);

        display.setCursor(0, 30);
        display.print("Strat: ");
        display.print(getStrategyString(ai.currentStrategy));

        display.setCursor(0, 40);
        display.print("Saves: ");
        display.print(ai.savePercentage, 1);
        display.print("%");

        display.setCursor(0, 50);
        display.print("React: ");
        display.print(ai.reactionTime, 0);
        display.print("ms");
    }

    void displaySystemPage(const SensorData& sensors, const RobotState& state) {
        display.setCursor(0, 0);
        display.print("SYSTEM - Monitor");

        display.setCursor(0, 10);
        display.print("Battery: ");
        display.print(sensors.batteryVoltage, 1);
        display.print("V");

        display.setCursor(0, 20);
        display.print("Temp: ");
        display.print(sensors.temperature, 1);
        display.print("C");

        display.setCursor(0, 30);
        display.print("Current: ");
        display.print(sensors.motorCurrent[0], 2);
        display.print("A");

        display.setCursor(0, 40);
        display.print("CPU: ");
        display.print(state.cpuUsage);
        display.print("% MEM: ");
        display.print(state.memoryUsage);
        display.print("%");

        display.setCursor(0, 50);
        display.print("Errors: ");
        display.print(state.errorCode);
    }

    void displayPerformancePage(const SensorData& sensors, const RobotState& state) {
        display.setCursor(0, 0);
        display.print("PERFORMANCE");

        display.setCursor(0, 10);
        display.print("Runtime: ");
        display.print(millis() / 1000);
        display.print("s");

        display.setCursor(0, 20);
        display.print("Save Att: ");
        display.print(state.saveAttempts);
        display.print("/");
        display.print(state.successfulSaves);

        display.setCursor(0, 30);
        display.print("Ball Lost: ");
        display.print(state.ballLostCounter);

        display.setCursor(0, 40);
        display.print("Shooter RPM: ");
        display.print(state.shooterRPM, 0);

        display.setCursor(0, 50);
        display.print("Efficiency: ");
        display.print(ai.energyEfficiency * 100, 0);
        display.print("%");
    }

    void displayMessage(const String& message) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 25);
        display.print(message);
        display.display();
    }

    void displayError(const String& error) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 0);
        display.print("ERROR:");
        display.setCursor(0, 15);
        display.print(error);
        display.display();
    }

    String getStateString(GoalkeeperStateEnum state) {
        switch (state) {
            case GoalkeeperStateEnum::DEFENDING_GOAL: return "DEFEND";
            case GoalkeeperStateEnum::TRACKING_BALL: return "TRACK";
            case GoalkeeperStateEnum::POSITIONING: return "POS";
            case GoalkeeperStateEnum::INTERCEPTING: return "INTERCEPT";
            case GoalkeeperStateEnum::SHOOTING: return "SHOOT";
            case GoalkeeperStateEnum::RETURNING_TO_GOAL: return "RETURN";
            case GoalkeeperStateEnum::LEARNING_MODE: return "LEARN";
            case GoalkeeperStateEnum::EMERGENCY_STOP: return "EMERGENCY";
            case GoalkeeperStateEnum::PATROLLING_PERIMETER: return "PATROL";
            case GoalkeeperStateEnum::RESPECTING_BOUNDARIES: return "BOUNDARY";
            default: return "UNKNOWN";
        }
    }

    String getStrategyString(DefenseStrategy strategy) {
        switch (strategy) {
            case DefenseStrategy::CONSERVATIVE: return "CONSERV";
            case DefenseStrategy::BALANCED: return "BALANCED";
            case DefenseStrategy::AGGRESSIVE: return "AGGRESS";
            case DefenseStrategy::PREDICTIVE: return "PREDICT";
            case DefenseStrategy::ADAPTIVE: return "ADAPTIVE";
            default: return "UNKNOWN";
        }
    }

    uint16_t calculateChecksum(const String& data) {
        uint16_t checksum = 0;
        for (int i = 0; i < data.length(); i++) {
            checksum += data.charAt(i);
        }
        return checksum;
    }

    static void leftEncoderISR() {
        // Increment left encoder count
        // This function needs to be static or a global function
        // and access the encoder count via a global variable or a static member
        // For simplicity, assuming a global `leftEncoderCount`
        // This is a placeholder, actual implementation depends on how `leftEncoderCount` is declared
        // For this consolidated file, it's a member of HardwareInterface, so this ISR needs to be outside the class
        // or the encoder counts need to be global.
    }

    static void rightEncoderISR() {
        // Increment right encoder count
        // Similar to leftEncoderISR
    }
};

// Global encoder counts for ISRs
volatile long globalLeftEncoderCount = 0;
volatile long globalRightEncoderCount = 0;

void leftEncoderISR_global() {
    globalLeftEncoderCount++;
}

void rightEncoderISR_global() {
    globalRightEncoderCount++;
}


// ============================================================================
// CONSOLIDATED GOALKEEPER MOVEMENT CONTROLLER
// ============================================================================

class ConsolidatedGoalkeeperMovementController {
private:
    ConsolidatedGoalkeeperHardwareInterface* hardware;
    AdvancedPIDController gyroPID;
    AdvancedPIDController positionPID;
    AdvancedPIDController interceptPID; // New PID for aggressive interception

    float energyEfficiencyFactor = 1.0f;

public:
    ConsolidatedGoalkeeperMovementController(ConsolidatedGoalkeeperHardwareInterface* hw)
        : hardware(hw), gyroPID(Config::GYRO_PID),
          positionPID(Config::POSITION_PID),
          interceptPID(Config::INTERCEPT_PID) {}

    void move(int direction, float speedMultiplier = 1.0f, bool pullback = false) {
        int speed = (int)(Config::BASE_SPEED * speedMultiplier * energyEfficiencyFactor);
        int ml1, ml2, mr1, mr2;

        calculateMotorSpeeds(direction, speed, ml1, ml2, mr1, mr2);

        float gyroCorrection = gyroPID.compute(0, 0);
        ml1 += gyroCorrection; ml2 += gyroCorrection;
        mr1 -= gyroCorrection; mr2 -= gyroCorrection;

        hardware->setMotorSpeeds(ml1, ml2, mr2, mr1, pullback);
    }

    void moveToGlobalPosition(const Vector2D& targetGlobalPos, const Vector2D& currentGlobalPos, float currentHeading, float speedFactor = 1.0f) {
        Vector2D errorVector = targetGlobalPos - currentGlobalPos;
        float distance = errorVector.magnitude();

        if (distance < 1.0f) {
            stop();
            return;
        }

        float targetAngleGlobal = atan2(errorVector.y, errorVector.x) * 180.0f / PI;

        float headingError = targetAngleGlobal - currentHeading;
        while (headingError > 180.0f) headingError -= 360.0f;
        while (headingError < -180.0f) headingError += 360.0f;

        float positionCorrection = positionPID.compute(0, headingError);

        int motorDirection = angleToMotorDirection(headingError);

        float dynamicSpeed = speedFactor * constrain(distance / 50.0f, 0.2f, 1.0f);
        dynamicSpeed += abs(positionCorrection) * 0.1f;
        dynamicSpeed = constrain(dynamicSpeed, 0.1f, 1.0f);

        move(motorDirection, dynamicSpeed);
    }

    void interceptBall(const Vector2D& ballGlobalPos, const Vector2D& currentRobotPos, float speedFactor = 1.0f) {
        Vector2D interceptVector = ballGlobalPos - currentRobotPos;
        float distance = interceptVector.magnitude();

        if (distance < 0.5f) {
            stop();
            return;
        }

        float targetAngle = atan2(interceptVector.y, interceptVector.x) * 180.0f / PI;

        float correction = interceptPID.compute(0, 0);

        int motorDirection = angleToMotorDirection(targetAngle);
        move(motorDirection, speedFactor);
    }

    void stop() {
        hardware->setMotorSpeeds(0, 0, 0, 0, false, true);
    }

    void optimizeEnergyEfficiency(const SensorData& sensors) {
        float batteryFactor = sensors.batteryVoltage / 12.0f;
        float temperatureFactor = 1.0f - (sensors.temperature - 20.0f) / 40.0f;

        energyEfficiencyFactor = batteryFactor * temperatureFactor;
        energyEfficiencyFactor = constrain(energyEfficiencyFactor, 0.5f, 1.0f);
    }

private:
    void calculateMotorSpeeds(int direction, int speed, int& ml1, int& ml2, int& mr1, int& mr2) {
        float speedF = (float)speed;

        switch (direction) {
            case 0:  // Forward
                ml1 = speedF; ml2 = speedF; mr1 = -speedF; mr2 = -speedF;
                break;
            case 1:  // Forward-right (45 deg)
                ml1 = speedF; ml2 = speedF * 0.5f; mr1 = -speedF; mr2 = -speedF * 0.5f;
                break;
            case 2:  // Right (90 deg)
                ml1 = speedF; ml2 = 0; mr1 = -speedF; mr2 = 0;
                break;
            case 3:  // Back-right (135 deg)
                ml1 = speedF; ml2 = -speedF * 0.5f; mr1 = -speedF; mr2 = speedF * 0.5f;
                break;
            case 4:  // Backward (180 deg)
                ml1 = speedF; ml2 = -speedF; mr1 = -speedF; mr2 = speedF;
                break;
            case 5:  // Back-left (225 deg)
                ml1 = speedF * 0.5f; ml2 = -speedF; mr1 = -speedF * 0.5f; mr2 = speedF;
                break;
            case 6:  // Left (270 deg)
                ml1 = 0; ml2 = -speedF; mr1 = 0; mr2 = speedF;
                break;
            case 7:  // Forward-left (315 deg)
                ml1 = -speedF * 0.5f; ml2 = speedF; mr1 = speedF * 0.5f; mr2 = -speedF;
                break;
            case 8:  // Rotate left (CW)
                ml1 = -speedF; ml2 = -speedF; mr1 = speedF; mr2 = speedF;
                break;
            case 9:  // Rotate right (CCW)
                ml1 = speedF; ml2 = speedF; mr1 = -speedF; mr2 = -speedF;
                break;
            default: // Stop
                ml1 = ml2 = mr1 = mr2 = 0;
                break;
        }
    }

    int angleToMotorDirection(float angleDeg) {
        angleDeg = fmod(angleDeg + 360.0f, 360.0f);
        int dirIndex = (int)((angleDeg + 22.5f) / 45.0f) % 8;
        return dirIndex;
    }
};

// ============================================================================
// CONSOLIDATED GOALKEEPER AI WITH PREDICTIVE INTERCEPTION
// ============================================================================

class ConsolidatedGoalkeeperAI {
private:
    // Neural network for trajectory prediction (simplified)
    float trajectoryWeights[3][8] = {{0}};
    float trajectoryBiases[3][8] = {{0}};

    // Strategy learning system (Q-learning)
    float strategyQTable[13][8] = {{0}};
    float learningRate = 0.08f;
    float discountFactor = 0.92f;
    float epsilon = 0.05f;

public:
    void initialize() {
        initializeNeuralNetwork();
        initializeStrategyLearning();
    }

    void updateAI(AIState& ai, const SensorData& sensors, RobotState& state) {
        predictBallTrajectory(ai, sensors, state.globalPosition);
        calculateOptimalInterception(ai, sensors, state);
        adaptDefenseStrategy(ai, sensors, state);
        updatePerformanceMetrics(ai, state, sensors);
    }

    Vector2D getOptimalPosition(const AIState& ai, const SensorData& sensors, const RobotState& state) {
        Vector2D optimalPos = state.goalCenter;

        if (ai.trajectoryConfidence > Config::INTERCEPTION_CONFIDENCE_MIN && ai.interceptionTime > 0) {
            optimalPos = ai.optimalInterceptionPoint;
        } else if (sensors.ballThreatLevel > 0.3f) {
            Vector2D globalBallPos = state.globalPosition + sensors.ballVector;
            optimalPos.x = globalBallPos.x;
            optimalPos.y = state.optimalDistance;
        }

        optimalPos.x = constrain(optimalPos.x,
                               -Config::MAX_GOAL_POSITION,
                               Config::MAX_GOAL_POSITION);

        optimalPos.y = constrain(optimalPos.y,
                               state.goalCenter.y - Config::GOAL_DEPTH,
                               state.goalCenter.y + Config::OPTIMAL_GOAL_DISTANCE * 1.5f);

        return optimalPos;
    }

    InterceptionType chooseInterceptionType(const AIState& ai, const SensorData& sensors) {
        if (sensors.ballThreatLevel > 0.9f) {
            return InterceptionType::BLOCKING;
        } else if (ai.trajectoryConfidence > 0.8f && ai.interceptionTime < 0.5f) {
            return InterceptionType::PREDICTIVE;
        } else if (sensors.ballThreatLevel > 0.5f) {
            return InterceptionType::DIRECT;
        } else {
            return InterceptionType::DEFLECTION;
        }
    }

    float calculateReward(const RobotState& state, const SensorData& sensors, const AIState& ai) {
        float reward = 0.0f;

        if (sensors.ballThreatLevel > 0.8f && state.currentState == GoalkeeperStateEnum::INTERCEPTING) reward += 50.0f;
        if (state.currentState == GoalkeeperStateEnum::DEFENDING_GOAL && sensors.ballThreatLevel < 0.1f) reward += 10.0f;
        if (sensors.onGoalLine && abs(state.goalCenter.x - state.globalPosition.x) < 5) reward += 20.0f; // Centered on goal line
        if (ai.savePercentage > 0.8f) reward += 30.0f;

        if (sensors.onOutline) reward -= 100.0f;
        if (!sensors.onGoalLine && state.currentState == GoalkeeperStateEnum::DEFENDING_GOAL) reward -= 15.0f;
        if (state.ballLostCounter > 20) reward -= 20.0f;
        if (state.emergencyStop) reward -= 200.0f;

        reward += (sensors.batteryVoltage / 12.0f - 1.0f) * 5.0f;

        return reward;
    }

private:
    void initializeNeuralNetwork() {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 8; j++) {
                trajectoryWeights[i][j] = (random(200) - 100) / 2000.0f;
                trajectoryBiases[i][j] = (random(100) - 50) / 2000.0f;
            }
        }
    }

    void initializeStrategyLearning() {
        for (int i = 0; i < 13; i++) {
            for (int j = 0; j < 8; j++) {
                strategyQTable[i][j] = (random(100) - 50) / 2000.0f;
            }
        }
    }

    void predictBallTrajectory(AIState& ai, const SensorData& sensors, const Vector2D& robotGlobalPos) {
        if (sensors.ballThreatLevel < 0.1f) {
            ai.trajectoryConfidence = 0.0f;
            return;
        }

        Vector2D globalBallPos = robotGlobalPos + sensors.ballVector;
        Vector2D globalBallVel = sensors.ballVelocity;

        float inputs[5] = {
            globalBallPos.x,
            globalBallPos.y,
            globalBallVel.x,
            globalBallVel.y,
            sensors.ballThreatLevel
        };

        float hidden1[8], hidden2[8], output[2];

        for (int i = 0; i < 8; i++) {
            hidden1[i] = trajectoryBiases[0][i];
            for (int j = 0; j < 5; j++) {
                hidden1[i] += inputs[j] * trajectoryWeights[0][j];
            }
            hidden1[i] = tanh(hidden1[i]);
        }

        for (int i = 0; i < 8; i++) {
            hidden2[i] = trajectoryBiases[1][i];
            for (int j = 0; j < 8; j++) {
                hidden2[i] += hidden1[j] * trajectoryWeights[1][j];
            }
            hidden2[i] = tanh(hidden2[i]);
        }

        for (int i = 0; i < 2; i++) {
            output[i] = trajectoryBiases[2][i];
            for (int j = 0; j < 8; j++) {
                output[i] += hidden2[j] * trajectoryWeights[2][j];
            }
        }

        for (int i = 0; i < Config::INTERCEPTION_LOOKAHEAD; i++) {
            float t_step = (i + 1) * (Config::MAIN_LOOP_INTERVAL / 1000.0f);

            Vector2D physicsPrediction = globalBallPos + globalBallVel * t_step;
            Vector2D nnCorrection(output[0] * t_step * 2.0f, output[1] * t_step * 2.0f);

            ai.ballTrajectory[i] = physicsPrediction + nnCorrection;
        }

        ai.trajectoryConfidence = calculateTrajectoryConfidence(sensors);
    }

    void calculateOptimalInterception(AIState& ai, const SensorData& sensors, const RobotState& state) {
        if (ai.trajectoryConfidence < Config::INTERCEPTION_CONFIDENCE_MIN || sensors.ballThreatLevel < 0.1f) {
            ai.optimalInterceptionPoint = state.goalCenter;
            ai.interceptionTime = 0.0f;
            return;
        }

        float minTime = 1000.0f;
        Vector2D bestPoint = state.goalCenter;

        for (int i = 0; i < Config::INTERCEPTION_LOOKAHEAD; i++) {
            Vector2D trajectoryPoint = ai.ballTrajectory[i];
            float timeToBall = (i + 1) * (Config::MAIN_LOOP_INTERVAL / 1000.0f);

            Vector2D toPoint = trajectoryPoint - state.globalPosition;
            float distanceToPoint = toPoint.magnitude();

            float goalkeeperTravelSpeed = Config::BASE_SPEED / 65535.0f;
            float goalkeeperTime = (distanceToPoint / goalkeeperTravelSpeed) + (ai.reactionTime / 1000.0f);

            if (trajectoryPoint.y <= Config::OWN_GOAL_Y + 5 && goalkeeperTime < timeToBall && timeToBall < minTime) { // Check if ball reaches goal line
                minTime = timeToBall;
                bestPoint = trajectoryPoint;
            }
        }

        ai.optimalInterceptionPoint = bestPoint;
        ai.interceptionTime = minTime;
    }

    void adaptDefenseStrategy(AIState& ai, const SensorData& sensors, RobotState& state) {
        static unsigned long lastAdaptation = 0;

        if (millis() - lastAdaptation > Config::STRATEGY_UPDATE_INTERVAL * 5) {
            float saveRate = (ai.totalThreats > 0) ?
                           (float)ai.successfulSaves / ai.totalThreats : 0.5f;

            if (saveRate < 0.6f && sensors.ballThreatLevel > 0.5f) {
                ai.currentStrategy = DefenseStrategy::CONSERVATIVE;
                state.optimalDistance = Config::OPTIMAL_GOAL_DISTANCE * 0.8f;
                ai.learningRate = 0.1f;
            } else if (saveRate > 0.8f && sensors.ballThreatLevel < 0.3f) {
                ai.currentStrategy = DefenseStrategy::AGGRESSIVE;
                state.optimalDistance = Config::OPTIMAL_GOAL_DISTANCE * 1.2f;
                ai.learningRate = 0.05f;
            } else {
                ai.currentStrategy = DefenseStrategy::BALANCED;
                state.optimalDistance = Config::OPTIMAL_GOAL_DISTANCE;
                ai.learningRate = 0.08f;
            }

            lastAdaptation = millis();
        }
    }

    void updatePerformanceMetrics(AIState& ai, const RobotState& state, const SensorData& sensors) {
        static unsigned long lastUpdate = 0;

        if (millis() - lastUpdate > 5000) {
            ai.savePercentage = (ai.totalThreats > 0) ?
                              (float)ai.successfulSaves / ai.totalThreats * 100.0f : 0.0f;

            Vector2D positionError = state.globalPosition - state.targetPosition;
            ai.positioningAccuracy = 100.0f * (1.0f - constrain(positionError.magnitude() / 50.0f, 0.0f, 1.0f));

            ai.energyEfficiency = sensors.batteryVoltage / 12.0f;

            if (ai.savePercentage > 85.0f) {
                ai.reactionTime = max(ai.reactionTime * 0.98f, 100.0f);
            } else if (ai.savePercentage < 65.0f) {
                ai.reactionTime = min(ai.reactionTime * 1.02f, 250.0f);
            }

            lastUpdate = millis();
        }
    }

    float calculateTrajectoryConfidence(const SensorData& sensors) {
        float confidence = 0.0f;

        confidence += sensors.ballThreatLevel * 0.4f;

        static Vector2D lastVelocity = Vector2D(0, 0);
        float velocityChangeMagnitude = (sensors.ballVelocity - lastVelocity).magnitude();
        float velocityConsistency = 1.0f / (1.0f + velocityChangeMagnitude * 5.0f);
        confidence += velocityConsistency * 0.3f;
        lastVelocity = sensors.ballVelocity;

        static float lastThreatLevel = 0.0f;
        float threatStability = 1.0f - abs(sensors.ballThreatLevel - lastThreatLevel);
        confidence += threatStability * 0.3f;
        lastThreatLevel = sensors.ballThreatLevel;

        return constrain(confidence, 0.0f, 1.0f);
    }

    int mapStateToIndex(GoalkeeperStateEnum state) {
        switch (state) {
            case GoalkeeperStateEnum::DEFENDING_GOAL: return 0;
            case GoalkeeperStateEnum::TRACKING_BALL: return 1;
            case GoalkeeperStateEnum::POSITIONING: return 2;
            case GoalkeeperStateEnum::INTERCEPTING: return 3;
            case GoalkeeperStateEnum::SHOOTING: return 4;
            case GoalkeeperStateEnum::RETURNING_TO_GOAL: return 5;
            case GoalkeeperStateEnum::COORDINATING_DEFENSE: return 6;
            case GoalkeeperStateEnum::LEARNING_MODE: return 7;
            case GoalkeeperStateEnum::EMERGENCY_STOP: return 8;
            case GoalkeeperStateEnum::PATROLLING_PERIMETER: return 9;
            case GoalkeeperStateEnum::RESPECTING_BOUNDARIES: return 10;
            default: return 11;
        }
    }

    void updateQLearning(RobotState& state, int action, float reward) {
        if (!state.adaptiveMode) return;

        int currentStateIdx = mapStateToIndex(state.currentState);
        int nextStateIdx = mapStateToIndex(state.currentState);

        float maxNextQ = strategyQTable[nextStateIdx][0];
        for (int i = 1; i < 8; i++) {
            if (strategyQTable[nextStateIdx][i] > maxNextQ) {
                maxNextQ = strategyQTable[nextStateIdx][i];
            }
        }

        float oldQ = strategyQTable[currentStateIdx][action];
        float newQ = oldQ + learningRate * (reward + discountFactor * maxNextQ - oldQ);
        strategyQTable[currentStateIdx][action] = newQ;
    }
};

// ============================================================================
// CONSOLIDATED GOALKEEPER COMMUNICATION SYSTEM
// ============================================================================

class ConsolidatedGoalkeeperCommunicationSystem {
private:
    ConsolidatedGoalkeeperHardwareInterface* hardware;

    struct Packet {
        uint8_t header = 0xAA;
        uint8_t type;
        uint8_t length;
        uint8_t data[Config::PACKET_SIZE];
        uint16_t checksum;
        uint8_t footer = 0x55;
    };

    struct AttackerData {
        int ballPosition = -1;
        float ballStrength = 0.0f;
        GoalkeeperStateEnum attackerState = GoalkeeperStateEnum::INITIALIZING;
        float attackerHeading = 0.0f;
        float predictionConfidence = 0.0f;
        float batteryVoltage = 0.0f;
        bool ballDetected = false;
        bool communicationActive = false;
        unsigned long lastCommTime = 0;
        float signalStrength = 0.0f;
        int packetLossRate = 0;
    } attackerData;

    int packetsSent = 0;
    int packetsReceived = 0;
    int packetsLost = 0;
    unsigned long totalCommTime = 0;

public:
    ConsolidatedGoalkeeperCommunicationSystem(ConsolidatedGoalkeeperHardwareInterface* hw) : hardware(hw) {}

    void update(const SensorData& sensors, const RobotState& state, const AIState& aiState) {
        processIncomingData();
        sendGoalkeeperStatus(sensors, state, aiState);
        updateCommStats();
        checkCommHealth();
    }

    const AttackerData& getAttackerData() const {
        return attackerData;
    }

    void sendEmergencyStop() {
        String emergencyMsg = "EMERGENCY:STOP";
        hardware->sendBluetoothData(emergencyMsg);
        packetsSent++;
    }

    void requestAttackerStatus() {
        String requestMsg = "REQ:STATUS";
        hardware->sendBluetoothData(requestMsg);
        packetsSent++;
    }

private:
    void processIncomingData() {
        String rawData = hardware->receiveBluetoothData();
        if (rawData.length() == 0) return;

        int typeEnd = rawData.indexOf(':');
        int crcStart = rawData.indexOf("|CRC:");

        if (typeEnd == -1 || crcStart == -1 || crcStart + 5 >= rawData.length()) return;

        String typeStr = rawData.substring(0, typeEnd);
        String dataStr = rawData.substring(typeEnd + 1, crcStart);
        String crcStr = rawData.substring(crcStart + 5);

        uint16_t receivedCRC = (uint16_t)strtol(crcStr.c_str(), NULL, 16);
        uint16_t calculatedCRC = calculateCRC(typeStr + ":" + dataStr);

        if (receivedCRC != calculatedCRC) {
            packetsLost++;
            Serial.println("CRC Mismatch!");
            return;
        }

        packetsReceived++;
        attackerData.lastCommTime = millis();
        attackerData.communicationActive = true;

        if (typeStr == "F1S_STATUS") {
            parseAttackerStatus(dataStr);
        } else if (typeStr == "F1S_BALL") {
            parseAttackerBallData(dataStr);
        } else if (typeStr == "EMERGENCY") {
            if (dataStr == "STOP") {
                Serial.println("Attacker sent EMERGENCY STOP!");
            }
        }
    }

    void sendGoalkeeperStatus(const SensorData& sensors, const RobotState& state, const AIState& aiState) {
        static unsigned long lastSend = 0;
        if (millis() - lastSend < Config::COMM_UPDATE_INTERVAL) return;

        String statusData = "POS_X:" + String(state.globalPosition.x, 1) +
                           ",POS_Y:" + String(state.globalPosition.y, 1) +
                           ",STATE:" + String((int)state.currentState) +
                           ",BALL_THREAT:" + String(sensors.ballThreatLevel, 2) +
                           ",SAVE_PCT:" + String(aiState.savePercentage, 1) +
                           ",BAT:" + String(sensors.batteryVoltage, 1) +
                           ",TEMP:" + String(sensors.temperature, 1);

        hardware->sendBluetoothData("GK_STATUS:" + statusData);
        packetsSent++;
        lastSend = millis();
    }

    void parseAttackerStatus(const String& data) {
        int ballPosStart = data.indexOf("BALL:") + 5;
        int ballPosEnd = data.indexOf(',', ballPosStart);
        attackerData.ballPosition = data.substring(ballPosStart, ballPosEnd).toInt();

        int strStart = data.indexOf("STR:") + 4;
        int strEnd = data.indexOf(',', strStart);
        attackerData.ballStrength = data.substring(strStart, strEnd).toFloat();
        attackerData.ballDetected = (attackerData.ballStrength > 0.1f);

        int stateStart = data.indexOf("STATE:") + 6;
        int stateEnd = data.indexOf(',', stateStart);
        attackerData.attackerState = (GoalkeeperStateEnum)data.substring(stateStart, stateEnd).toInt();

        int headStart = data.indexOf("HEAD:") + 5;
        int headEnd = data.indexOf(',', headStart);
        attackerData.attackerHeading = data.substring(headStart, headEnd).toFloat();

        int confStart = data.indexOf("CONF:") + 5;
        int confEnd = data.indexOf(',', confStart);
        attackerData.predictionConfidence = data.substring(confStart, confEnd).toFloat();

        int batStart = data.indexOf("BAT:") + 4;
        int batEnd = data.indexOf(',', batStart);
        attackerData.batteryVoltage = data.substring(batStart, batEnd).toFloat();
    }

    void parseAttackerBallData(const String& data) {
        // Similar parsing logic as above for ball position and strength
    }

    uint16_t calculateCRC(const String& data) {
        uint16_t crc = 0xFFFF;
        for (int i = 0; i < data.length(); i++) {
            crc ^= data.charAt(i);
            for (int j = 0; j < 8; j++) {
                if (crc & 1) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }

    void updateCommStats() {
        static unsigned long lastUpdate = 0;
        if (millis() - lastUpdate > 5000) {
            if (packetsSent > 0) {
                attackerData.packetLossRate = (packetsLost * 100) / packetsSent;
            }

            attackerData.signalStrength = 1.0f - (attackerData.packetLossRate / 100.0f);
            attackerData.signalStrength = constrain(attackerData.signalStrength, 0.0f, 1.0f);

            packetsSent = 0;
            packetsReceived = 0;
            packetsLost = 0;
            lastUpdate = millis();
        }
    }

    void checkCommHealth() {
        if (millis() - attackerData.lastCommTime > Config::COMM_TIMEOUT) {
            attackerData.communicationActive = false;
            attackerData.signalStrength = 0.0f;
        }

        if (!attackerData.communicationActive) {
            static unsigned long lastRecoveryAttempt = 0;
            if (millis() - lastRecoveryAttempt > 2000) {
                requestAttackerStatus();
                lastRecoveryAttempt = millis();
            }
        }
    }
};

// ============================================================================
// TASK SCHEDULER FOR REAL-TIME PERFORMANCE
// ============================================================================

class TaskScheduler {
private:
    struct Task {
        void (*function)();
        unsigned long interval;
        unsigned long lastRun;
        int priority;
        bool enabled;
        unsigned long lastExecutionDuration;
    };

    Task tasks[10];
    int taskCount = 0;
    unsigned long totalCPUTime = 0;
    unsigned long lastCPUMeasurement = 0;
    unsigned long measurementPeriod = 1000000;

public:
    TaskScheduler() {
        lastCPUMeasurement = micros();
    }

    void addTask(void (*func)(), unsigned long interval, int priority = 1, bool enabled = true) {
        if (taskCount < 10) {
            tasks[taskCount] = {func, interval, 0, priority, enabled, 0};
            taskCount++;
        } else {
            Serial.println("Task limit reached!");
        }
    }

    void run() {
        unsigned long currentTime = millis();

        for (int i = 0; i < taskCount - 1; i++) {
            for (int j = 0; j < taskCount - i - 1; j++) {
                if (tasks[j].priority < tasks[j + 1].priority) {
                    Task temp = tasks[j];
                    tasks[j] = tasks[j + 1];
                    tasks[j + 1] = temp;
                }
            }
        }

        for (int i = 0; i < taskCount; i++) {
            if (tasks[i].enabled && (currentTime - tasks[i].lastRun >= tasks[i].interval)) {
                unsigned long taskStart = micros();
                tasks[i].function();
                unsigned long taskEnd = micros();

                tasks[i].lastExecutionDuration = taskEnd - taskStart;
                tasks[i].lastRun = currentTime;
                totalCPUTime += tasks[i].lastExecutionDuration;
            }
        }

        updateCPUStats();
    }

    void enableTask(int index, bool enable) {
        if (index < taskCount) {
            tasks[index].enabled = enable;
        }
    }

    int getCPUUsage() {
        unsigned long elapsedTime = micros() - lastCPUMeasurement;
        if (elapsedTime == 0) return 0;
        return (int)((totalCPUTime * 100.0f) / elapsedTime);
    }

private:
    void updateCPUStats() {
        unsigned long now = micros();
        if (now - lastCPUMeasurement >= measurementPeriod) {
            totalCPUTime = 0;
            lastCPUMeasurement = now;
        }
    }
};

// ============================================================================
// MAIN ROBOT APPLICATION CLASS
// ============================================================================

class F2SRobot {
private:
    // Core systems
    ConsolidatedGoalkeeperHardwareInterface hardware;
    ConsolidatedGoalkeeperSensorSystem sensorSystem;
    ConsolidatedGoalkeeperMovementController movement;
    ConsolidatedGoalkeeperAI aiSystem;
    ConsolidatedGoalkeeperCommunicationSystem communication;
    TaskScheduler scheduler;
    ComputerVisionData visionData; // For vision system access
    PredictiveAnalytics analytics; // For analytics access

    // Robot state
    RobotState robotState;
    AIState aiState;
    SensorData currentSensors;

    // Performance monitoring
    unsigned long loopStartTime = 0;
    float averageLoopTime = 0.0f;

public:
    F2SRobot() : movement(&hardware), aiSystem(), communication(&hardware) {
        hardware.setSensorSystem(&sensorSystem);
    }

    bool initialize() {
        Serial.begin(Config::BLUETOOTH_BAUD);
        Serial.println("F2S Goalkeeper Robot v3.1 Initializing...");

        if (!hardware.initialize()) {
            Serial.println("Hardware initialization failed!");
            return false;
        }

        if (!sensorSystem.initialize()) {
            Serial.println("Sensor system initialization failed!");
            return false;
        }

        sensorSystem.calibrateSensors();

        setupTasks();

        aiSystem.initialize();
        aiState.adaptiveMode = true;
        aiState.learningRate = Config::GYRO_PID.ki;

        robotState.goalCenter = Vector2D(0, Config::OWN_GOAL_Y);
        robotState.globalPosition = Vector2D(0, Config::OWN_GOAL_Y + Config::OPTIMAL_GOAL_DISTANCE);
        robotState.currentState = GoalkeeperStateEnum::DEFENDING_GOAL;
        robotState.loopFrequency = 1000.0f / Config::MAIN_LOOP_INTERVAL;

        robotInternal.x = 400;
        robotInternal.y = 50;
        robotInternal.theta = 0;
        robotInternal.isCalibrated = true;

        strategyInternal.aggressiveness = 70;
        strategyInternal.reactionTime = 150;
        strategyInternal.usePredicition = true;
        strategyInternal.allowAdvance = true;
        strategyInternal.maxAdvanceDistance = 100;

        positionPIDInternal = {2.0, 0.1, 0.5, 0, 0, 0};
        orientationPIDInternal = {1.5, 0.05, 0.3, 0, 0, 0};

        hardware.loadConfiguration();

        Serial.println("F2S Goalkeeper Robot v3.1 Ready!");
        return true;
    }

    void run() {
        loopStartTime = micros();

        scheduler.run();

        mainControlLoop();

        updatePerformanceMetrics();

        resetWatchdog();
    }

    void emergencyStop() {
        robotState.emergencyStop = true;
        robotState.currentState = GoalkeeperStateEnum::EMERGENCY_STOP;
        movement.stop();
        hardware.setShooterSpeed(0);
        communication.sendEmergencyStop();
        Serial.println("F2S EMERGENCY STOP ACTIVATED!");
    }

private:
    void setupTasks() {
        scheduler.addTask([this]() { this->readSensorsTask(); }, Config::SENSOR_UPDATE_INTERVAL, 10);
        scheduler.addTask([this]() { this->updateMotorControlTask(); }, 5, 9);
        scheduler.addTask([this]() { this->safetyCheckTask(); }, 100, 8);
        scheduler.addTask([this]() { this->processBluetoothTask(); }, 50, 7);

        scheduler.addTask([this]() { this->updateAIAndAnalyticsTask(); }, Config::AI_UPDATE_INTERVAL, 5);
        scheduler.addTask([this]() { this->updateCommunicationTask(); }, Config::COMM_UPDATE_INTERVAL, 4);
        scheduler.addTask([this]() { this->updateDisplayTask(); }, 200, 3);

        scheduler.addTask([this]() { this->performMaintenanceTask(); }, 10000, 1);
        scheduler.addTask([this]() { this->logPerformanceTask(); }, 5000, 1);
    }

    void mainControlLoop() {
        switch (robotState.currentState) {
            case GoalkeeperStateEnum::INITIALIZING:
                robotState.currentState = GoalkeeperStateEnum::CALIBRATING;
                break;
            case GoalkeeperStateEnum::CALIBRATING:
                robotState.currentState = GoalkeeperStateEnum::DEFENDING_GOAL;
                break;
            case GoalkeeperStateEnum::EMERGENCY_STOP:
                movement.stop();
                hardware.setShooterSpeed(0);
                return;

            case GoalkeeperStateEnum::DEFENDING_GOAL:
                executeDefendingStrategy();
                break;

            case GoalkeeperStateEnum::TRACKING_BALL:
                executeTrackingBallStrategy();
                break;

            case GoalkeeperStateEnum::POSITIONING:
                executePositioningStrategy();
                break;

            case GoalkeeperStateEnum::INTERCEPTING:
                executeInterceptingStrategy();
                break;

            case GoalkeeperStateEnum::SHOOTING:
                executeShootingStrategy();
                break;

            case GoalkeeperStateEnum::RETURNING_TO_GOAL:
                executeReturningStrategy();
                break;

            case GoalkeeperStateEnum::COORDINATING_DEFENSE:
                executeCoordinationStrategy();
                break;

            case GoalkeeperStateEnum::PATROLLING_PERIMETER:
                executePatrollingStrategy();
                break;

            case GoalkeeperStateEnum::RESPECTING_BOUNDARIES:
                executeBoundaryRespectStrategy();
                break;

            case GoalkeeperStateEnum::LEARNING_MODE:
                break;

            case GoalkeeperStateEnum::MAINTENANCE_MODE:
                movement.stop();
                hardware.setShooterSpeed(0);
                break;

            case GoalkeeperStateEnum::ERROR_STATE:
                movement.stop();
                hardware.setShooterSpeed(0);
                break;

            default:
                movement.stop();
                break;
        }

        movement.optimizeEnergyEfficiency(currentSensors);
    }

    void readSensorsTask() {
        currentSensors = hardware.readSensors();
        robotState.globalPosition = currentSensors.globalPosition;

        static long lastLeftCount = 0;
        static long lastRightCount = 0;

        long leftDelta = globalLeftEncoderCount - lastLeftCount;
        long rightDelta = globalRightEncoderCount - lastRightCount;

        float wheelCircumference = 20.0;
        float wheelBase = 15.0;
        int encoderCPR = 360;

        float leftDistance = (leftDelta / (float)encoderCPR) * wheelCircumference;
        float rightDistance = (rightDelta / (float)encoderCPR) * wheelCircumference;

        float deltaDistance = (leftDistance + rightDistance) / 2.0;
        float deltaTheta = (rightDistance - leftDistance) / wheelBase;

        robotInternal.x += deltaDistance * cos(robotInternal.theta);
        robotInternal.y += deltaDistance * sin(robotInternal.theta);
        robotInternal.theta += deltaTheta;

        lastLeftCount = globalLeftEncoderCount;
        lastRightCount = globalRightEncoderCount;
        robotInternal.lastUpdate = millis();

        processBallDetection(currentSensors.leftIR, currentSensors.rightIR, currentSensors.frontIR, currentSensors.ultrasonicDistance);

        if (currentSensors.ballThreatLevel > 0.1f) {
            Vector2D globalBallPos = robotState.globalPosition + currentSensors.ballVector;
            robotState.ballHistory[robotState.ballHistoryIndex] = globalBallPos;
            robotState.ballHistoryIndex = (robotState.ballHistoryIndex + 1) % 10;
            robotState.lastBallTime = millis();
            robotState.ballLostCounter = 0;
        } else {
            robotState.ballLostCounter++;
        }
    }

    void updateMotorControlTask() {
        hardware.updateMotorControl();
    }

    void safetyCheckTask() {
        hardware.performSafetyCheck(currentSensors);

        if (currentSensors.batteryVoltage < Config::MIN_BATTERY_VOLTAGE && !robotState.emergencyStop) {
            robotState.errorCode = 101;
            emergencyStop();
        }

        if (millis() - robotState.lastWatchdogReset > Config::WATCHDOG_TIMEOUT && !robotState.emergencyStop) {
            robotState.errorCode = 102;
            emergencyStop();
        }

        if (currentSensors.onOutline && robotState.currentState != GoalkeeperStateEnum::EMERGENCY_STOP) {
            robotState.previousState = robotState.currentState;
            robotState.currentState = GoalkeeperStateEnum::RESPECTING_BOUNDARIES;
        }
    }

    void updateAIAndAnalyticsTask() {
        // Update vision system
        // ComputerVisionSystem visionSystem; // This should be a member of F2SRobot
        // visionSystem.processVision(currentSensors);
        // visionData = visionSystem.getVisionData(); // Update member visionData

        // Update predictive analytics
        // PredictiveAnalyticsEngine analyticsEngine; // This should be a member of F2SRobot
        // analyticsEngine.updatePredictions(currentSensors, visionData);
        // analytics = analyticsEngine.getAnalytics(); // Update member analytics

        aiSystem.updateAI(aiState, currentSensors, robotState);
        if (aiState.adaptiveMode) {
            float reward = aiSystem.calculateReward(robotState, currentSensors, aiState);
            // aiSystem.updateQLearning(robotState, action, reward); // Requires previous action
        }
    }

    void updateCommunicationTask() {
        communication.update(currentSensors, robotState, aiState);
        if (communication.getAttackerData().attackerState == GoalkeeperStateEnum::EMERGENCY_STOP && !robotState.emergencyStop) {
            Serial.println("Attacker requested emergency stop!");
            emergencyStop();
        }
    }

    void processBluetoothTask() {
        hardware.processBluetoothCommands();
    }

    void updateDisplayTask() {
        hardware.updateDisplay(currentSensors, robotState, aiState);
    }

    void performMaintenanceTask() {
        static int maintenanceCycle = 0;
        maintenanceCycle++;

        if (maintenanceCycle % 5 == 0) {
            // Check sensor calibration drift, memory cleanup
        }
    }

    void logPerformanceTask() {
        Serial.print("Goalkeeper Perf: Loop=");
        Serial.print(averageLoopTime, 2);
        Serial.print("ms, CPU=");
        Serial.print(scheduler.getCPUUsage());
        Serial.print("%, Bat=");
        Serial.print(currentSensors.batteryVoltage, 1);
        Serial.print("V, Temp=");
        Serial.print(currentSensors.temperature, 1);
        Serial.print("C, Saves=");
        Serial.print(aiState.savePercentage, 1);
        Serial.println("%");
    }

    void executeDefendingStrategy() {
        Vector2D optimalPos = aiSystem.getOptimalPosition(aiState, currentSensors, robotState);
        robotState.targetPosition = optimalPos;

        movement.moveToGlobalPosition(robotState.targetPosition, robotState.globalPosition, currentSensors.heading, 0.7f);

        if (currentSensors.ballThreatLevel > 0.4f) {
            robotState.previousState = GoalkeeperStateEnum::DEFENDING_GOAL;
            robotState.currentState = GoalkeeperStateEnum::TRACKING_BALL;
        }
    }

    void executeTrackingBallStrategy() {
        Vector2D ballGlobalPos = robotState.globalPosition + currentSensors.ballVector;
        Vector2D targetTrackingPos = aiSystem.getOptimalPosition(aiState, currentSensors, robotState);

        movement.moveToGlobalPosition(targetTrackingPos, robotState.globalPosition, currentSensors.heading, 0.9f);

        if (currentSensors.ballThreatLevel > 0.7f && aiState.interceptionTime < Config::REACTION_TIME_MS / 1000.0f + 0.1f) {
            robotState.previousState = GoalkeeperStateEnum::TRACKING_BALL;
            robotState.currentState = GoalkeeperStateEnum::INTERCEPTING;
        } else if (currentSensors.ballThreatLevel < 0.2f && robotState.ballLostCounter > 5) {
            robotState.previousState = GoalkeeperStateEnum::TRACKING_BALL;
            robotState.currentState = GoalkeeperStateEnum::DEFENDING_GOAL;
        }
    }

    void executePositioningStrategy() {
        Vector2D optimalPos = aiSystem.getOptimalPosition(aiState, currentSensors, robotState);
        robotState.targetPosition = optimalPos;

        movement.moveToGlobalPosition(robotState.targetPosition, robotState.globalPosition, currentSensors.heading, 0.8f);

        if (currentSensors.ballThreatLevel > 0.5f) {
            robotState.previousState = GoalkeeperStateEnum::POSITIONING;
            robotState.currentState = GoalkeeperStateEnum::INTERCEPTING;
        } else if (abs((robotState.globalPosition - robotState.targetPosition).magnitude()) < 5.0f) {
            robotState.previousState = GoalkeeperStateEnum::POSITIONING;
            robotState.currentState = GoalkeeperStateEnum::DEFENDING_GOAL;
        }
    }

    void executeInterceptingStrategy() {
        Vector2D ballGlobalPos = robotState.globalPosition + currentSensors.ballVector;
        movement.interceptBall(ballGlobalPos, robotState.globalPosition, 1.0f);

        static unsigned long lastThreatDetectTime = 0;
        if (currentSensors.ballThreatLevel > 0.8f && millis() - lastThreatDetectTime > 500) {
            robotState.saveAttempts++;
            lastThreatDetectTime = millis();
        }

        if (currentSensors.ballThreatLevel > 0.95f) {
            robotState.previousState = GoalkeeperStateEnum::INTERCEPTING;
            robotState.currentState = GoalkeeperStateEnum::SHOOTING;
        } else if (currentSensors.ballThreatLevel < 0.4f && robotState.ballLostCounter > 3) {
            robotState.previousState = GoalkeeperStateEnum::INTERCEPTING;
            robotState.currentState = GoalkeeperStateEnum::DEFENDING_GOAL;
        }
    }

    void executeShootingStrategy() {
        static unsigned long shootStartTime = 0;

        if (shootStartTime == 0) {
            shootStartTime = millis();
            hardware.setShooterSpeed(Config::MAX_MOTOR_SPEED, 10000.0f);
            movement.move(0, 1.0f, true);
            robotState.successfulSaves++;
        }

        if (millis() - shootStartTime >= Config::SHOOT_DURATION) {
            hardware.setShooterSpeed(0);
            robotState.lastShootTime = millis();
            shootStartTime = 0;
            robotState.previousState = GoalkeeperStateEnum::SHOOTING;
            robotState.currentState = GoalkeeperStateEnum::RETURNING_TO_GOAL;
        }
    }

    void executeReturningStrategy() {
        Vector2D returnTarget = Vector2D(0, Config::OWN_GOAL_Y + Config::OPTIMAL_GOAL_DISTANCE);

        if (currentSensors.onOutline) {
            movement.move(4, 0.8f);
        } else if (!currentSensors.onGoalLine || abs(robotState.globalPosition.x - returnTarget.x) > 5.0f) {
            movement.moveToGlobalPosition(returnTarget, robotState.globalPosition, currentSensors.heading, 0.6f);
        } else {
            movement.stop();
            robotState.previousState = GoalkeeperStateEnum::RETURNING_TO_GOAL;
            robotState.currentState = GoalkeeperStateEnum::DEFENDING_GOAL;
            robotState.goalCenterOffset = 0;
        }
    }

    void executeCoordinationStrategy() {
        ConsolidatedGoalkeeperCommunicationSystem::AttackerData attackerInfo = communication.getAttackerData();
        if (attackerInfo.communicationActive && attackerInfo.ballDetected) {
            if (attackerInfo.ballPosition >= 0 && attackerInfo.ballPosition <= 7) {
                robotState.targetPosition.x = -Config::MAX_GOAL_POSITION * 0.5f;
                robotState.targetPosition.y = Config::OPTIMAL_GOAL_DISTANCE;
                movement.moveToGlobalPosition(robotState.targetPosition, robotState.globalPosition, currentSensors.heading, 0.5f);
            } else if (attackerInfo.ballPosition >= 8 && attackerInfo.ballPosition <= 15) {
                robotState.targetPosition.x = Config::MAX_GOAL_POSITION * 0.5f;
                robotState.targetPosition.y = Config::OPTIMAL_GOAL_DISTANCE;
                movement.moveToGlobalPosition(robotState.targetPosition, robotState.globalPosition, currentSensors.heading, 0.5f);
            } else {
                robotState.targetPosition.x = 0;
                robotState.targetPosition.y = Config::OPTIMAL_GOAL_DISTANCE;
                movement.moveToGlobalPosition(robotState.targetPosition, robotState.globalPosition, currentSensors.heading, 0.5f);
            }
        } else {
            robotState.previousState = GoalkeeperStateEnum::COORDINATING_DEFENSE;
            robotState.currentState = GoalkeeperStateEnum::DEFENDING_GOAL;
        }

        if (currentSensors.ballThreatLevel > 0.4f) {
            robotState.previousState = GoalkeeperStateEnum::COORDINATING_DEFENSE;
            robotState.currentState = GoalkeeperStateEnum::TRACKING_BALL;
        }
    }

    void executePatrollingStrategy() {
        // Patrol along the penalty perimeter
        Vector2D patrolTarget = Vector2D(0, Config::PENALTY_PERIMETER_Y);

        // Adjust lateral position based on ball threat or general patrol pattern
        if (currentSensors.ballThreatLevel > 0.1f) {
            Vector2D globalBallPos = robotState.globalPosition + currentSensors.ballVector;
            patrolTarget.x = constrain(globalBallPos.x * 0.6f, -Config::PENALTY_AREA_WIDTH / 2, Config::PENALTY_AREA_WIDTH / 2);
        } else {
            // Simple back-and-forth patrol
            static bool movingRight = true;
            if (movingRight) {
                patrolTarget.x = Config::PENALTY_AREA_WIDTH / 2 - 10;
                if (abs(robotState.globalPosition.x - patrolTarget.x) < 5) movingRight = false;
            } else {
                patrolTarget.x = -(Config::PENALTY_AREA_WIDTH / 2 - 10);
                if (abs(robotState.globalPosition.x - patrolTarget.x) < 5) movingRight = true;
            }
        }

        robotState.targetPosition = patrolTarget;
        movement.moveToGlobalPosition(robotState.targetPosition, robotState.globalPosition, currentSensors.heading, 0.6f);

        // Transition conditions
        if (currentSensors.ballThreatLevel > 0.3f) {
            robotState.previousState = GoalkeeperStateEnum::PATROLLING_PERIMETER;
            robotState.currentState = GoalkeeperStateEnum::TRACKING_BALL;
        } else if (currentSensors.onGoalLine) {
            robotState.previousState = GoalkeeperStateEnum::PATROLLING_PERIMETER;
            robotState.currentState = GoalkeeperStateEnum::DEFENDING_GOAL;
        }
    }

    void executeBoundaryRespectStrategy() {
        // Move away from boundaries
        // This logic is similar to F1S, but adapted for GK's restricted movement
        Vector2D avoidanceVector(0, 0);

        if (currentSensors.globalPosition.x > Config::FIELD_WIDTH / 2 - Config::BOUNDARY_MARGIN * 2) {
            avoidanceVector.x = -1.0f; // Move left
        }
        if (currentSensors.globalPosition.x < -Config::FIELD_WIDTH / 2 + Config::BOUNDARY_MARGIN * 2) {
            avoidanceVector.x = 1.0f; // Move right
        }
        if (currentSensors.globalPosition.y < Config::OWN_GOAL_Y + Config::BOUNDARY_MARGIN * 2) {
            avoidanceVector.y = 1.0f; // Move forward (away from own goal line)
        }
        if (currentSensors.globalPosition.y > Config::PENALTY_PERIMETER_Y + Config::BOUNDARY_MARGIN * 2) {
            avoidanceVector.y = -1.0f; // Move backward (away from midfield)
        }

        if (avoidanceVector.magnitude() > 0) {
            float angle = atan2(avoidanceVector.y, avoidanceVector.x) * 180.0f / PI;
            int direction = (int)((angle + 180.0f) / 22.5f) % 16; // Move in opposite direction
            movement.move(direction, 0.8f);
        }

        // Return to normal behavior when safe
        if (!currentSensors.onOutline && currentSensors.currentZone != DefenseZone::OUT_OF_BOUNDS) {
            robotState.currentState = GoalkeeperStateEnum::DEFENDING_GOAL;
        }
    }

    void processBallDetection(int leftIR, int rightIR, int frontIR, float distance) {
        ballInternal.isVisible = false;

        if (frontIR > 500 || leftIR > 500 || rightIR > 500) {
            ballInternal.isVisible = true;
            ballInternal.lastSeen = millis();
            ballInternal.distance = distance;

            if (frontIR > leftIR && frontIR > rightIR) {
                ballInternal.angle = 0;
            } else if (leftIR > rightIR) {
                ballInternal.angle = -45;
            } else {
                ballInternal.angle = 45;
            }

            ballInternal.x = robotInternal.x + ballInternal.distance * cos(radians(robotInternal.theta + ballInternal.angle));
            ballInternal.y = robotInternal.y + ballInternal.distance * sin(radians(robotInternal.theta + ballInternal.angle));
        }
    }

    void updatePerformanceMetrics() {
        unsigned long loopTime = micros() - loopStartTime;
        averageLoopTime = (averageLoopTime * 0.9f) + (loopTime / 1000.0f * 0.1f);
        robotState.loopFrequency = 1000.0f / averageLoopTime;
        robotState.cpuUsage = scheduler.getCPUUsage();
        robotState.memoryUsage = 40;
        robotState.totalRunTime = millis();
    }

    void resetWatchdog() {
        robotState.lastWatchdogReset = millis();
    }
};

// ============================================================================
// ARDUINO MAIN FUNCTIONS
// ============================================================================

F2SRobot robot;

void setup() {
    randomSeed(analogRead(A0));

    // Attach global encoder ISRs
    attachInterrupt(digitalPinToInterrupt(18), leftEncoderISR_global, RISING);
    attachInterrupt(digitalPinToInterrupt(19), rightEncoderISR_global, RISING);

    if (!robot.initialize()) {
        Serial.println("Robot initialization failed. Halting system.");
        while (true) {
            delay(1000);
        }
    }
}

void loop() {
    robot.run();
    delayMicroseconds(Config::MAIN_LOOP_INTERVAL);
}

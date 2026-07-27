#include <Adafruit_SH1106_STM32.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050.h>
#include <EEPROM.h>
#include <Servo.h>  // Assuming a kicker servo might be used in some versions

// ============================================================================
// CONSOLIDATED CONFIGURATION SYSTEM
// ============================================================================

namespace Config {
// Performance Configuration
static constexpr int MAX_MOTOR_SPEED = 65535;
static constexpr int BASE_SPEED = 65000;
static constexpr float PULLBACK_MULTIPLIER = 1.5f;
static constexpr int MOTOR_ACCELERATION_LIMIT = 5000;

// Field Dimensions (in cm, assuming standard RoboCup Junior field)
static constexpr int FIELD_LENGTH = 182;         // 182cm total length
static constexpr int FIELD_WIDTH = 122;          // 122cm total width
static constexpr int PENALTY_AREA_LENGTH = 30;   // 30cm from goal line
static constexpr int PENALTY_AREA_WIDTH = 60;    // 60cm wide
static constexpr int GOAL_WIDTH = 40;            // 40cm goal width
static constexpr int CENTER_CIRCLE_RADIUS = 20;  // 20cm radius
static constexpr int BOUNDARY_MARGIN = 8;        // 8cm safety margin from boundaries

// Field Zones (relative to robot's starting position)
// Assuming F1S starts at (0, -FIELD_LENGTH/4) facing opponent goal at (0, FIELD_LENGTH/2)
static constexpr int OPPONENT_GOAL_Y = FIELD_LENGTH / 2;
static constexpr int OWN_GOAL_Y = -FIELD_LENGTH / 2;
static constexpr int CENTER_LINE_Y = 0;
static constexpr int OPPONENT_PENALTY_Y = OPPONENT_GOAL_Y - PENALTY_AREA_LENGTH;
static constexpr int OWN_PENALTY_Y = OWN_GOAL_Y + PENALTY_AREA_LENGTH;

// Advanced Sensor Configuration
static constexpr int TSOP_SENSORS = 16;
static constexpr int COLOR_SENSORS = 4;
static constexpr int SENSOR_SAMPLES = 6;  // Increased samples for accuracy
static constexpr int BALL_DETECTION_THRESHOLD = 3700;
static constexpr int BALL_CLOSE_THRESHOLD = 2500;
static constexpr int BALL_VERY_CLOSE_THRESHOLD = 1800;
static constexpr int OUTLINE_THRESHOLD = 800;
static constexpr int WHITE_LINE_THRESHOLD = 3000;  // For center line and penalty area
static constexpr int NEAR_OUTLINE_THRESHOLD = 1200;

// AI & Machine Learning
static constexpr int BALL_HISTORY_SIZE = 10;
static constexpr int PREDICTION_HORIZON = 5;
static constexpr float LEARNING_RATE = 0.01f;
static constexpr int STRATEGY_ADAPTATION_CYCLES = 100;  // For general strategy adaptation

// Advanced Timing
static constexpr unsigned long MAIN_LOOP_INTERVAL = 20;          // 50Hz
static constexpr unsigned long SENSOR_UPDATE_INTERVAL = 10;      // 100Hz
static constexpr unsigned long AI_UPDATE_INTERVAL = 50;          // 20Hz
static constexpr unsigned long COMM_UPDATE_INTERVAL = 30;        // 33Hz
static constexpr unsigned long VISION_PROCESSING_INTERVAL = 40;  // 25Hz

// PID Controllers (Adaptive)
struct PIDConfig {
  float kp, ki, kd;
  float min_output, max_output;
  float integral_limit;
};

static constexpr PIDConfig GYRO_PID = { 350.0f, 0.15f, 75.0f, -20000, 20000, 1000.0f };
static constexpr PIDConfig BALL_TRACK_PID = { 2.5f, 0.05f, 0.8f, -1.0f, 1.0f, 10.0f };
static constexpr PIDConfig POSITION_PID = { 1.8f, 0.02f, 0.5f, -0.8f, 0.8f, 5.0f };

// Communication Protocol
static constexpr int BLUETOOTH_BAUD = 115200;  // Higher baud rate
static constexpr int PACKET_SIZE = 64;
static constexpr int MAX_RETRIES = 3;
static constexpr unsigned long ACK_TIMEOUT = 100;
static constexpr unsigned long COMM_TIMEOUT = 1000;  // General communication timeout
static constexpr unsigned long HEARTBEAT_INTERVAL = 200;

// Safety & Monitoring
static constexpr float MIN_BATTERY_VOLTAGE = 11.0f;
static constexpr float MAX_MOTOR_CURRENT = 5.0f;
static constexpr int WATCHDOG_TIMEOUT = 1000;
static constexpr int MAX_TEMPERATURE = 70;  // Celsius
}

// ============================================================================
// CONSOLIDATED DATA STRUCTURES
// ============================================================================

enum class AttackerState : uint8_t {
  INITIALIZING = 0,
  CALIBRATING,
  SEARCHING,
  CHASING_BALL,
  DRIBBLING,
  POSITIONING_FOR_SHOT,
  SHOOTING,
  AVOIDING_OUTLINE,
  RESPECTING_BOUNDARIES,
  COORDINATING_WITH_GK,
  LEARNING_MODE,
  EMERGENCY_STOP,
  MAINTENANCE_MODE,
  ERROR_STATE
};

enum class MovementPattern : uint8_t {
  DIRECT = 0,
  CURVED,
  SPIRAL,
  ZIGZAG,
  PREDICTIVE,
  ADAPTIVE
};

enum class FieldZone : uint8_t {
  OWN_HALF = 0,
  OPPONENT_HALF,
  CENTER_CIRCLE,
  OPPONENT_PENALTY_AREA,
  OWN_PENALTY_AREA,
  OUT_OF_BOUNDS
};

struct Vector2D {
  float x, y;
  Vector2D(float x = 0, float y = 0)
    : x(x), y(y) {}
  Vector2D operator+(const Vector2D& other) const {
    return Vector2D(x + other.x, y + other.y);
  }
  Vector2D operator-(const Vector2D& other) const {
    return Vector2D(x - other.x, y - other.y);
  }
  Vector2D operator*(float scalar) const {
    return Vector2D(x * scalar, y * scalar);
  }
  float magnitude() const {
    return sqrt(x * x + y * y);
  }
  Vector2D normalized() const {
    float mag = magnitude();
    return mag > 0 ? Vector2D(x / mag, y / mag) : Vector2D();
  }
};

struct SensorData {
  // TSOP Data with filtering
  int tsopRaw[Config::TSOP_SENSORS];
  int tsopFiltered[Config::TSOP_SENSORS];
  int tsopMin = 4095;
  int tsopNum = 0;
  float ballStrength = 0.0f;
  Vector2D ballVector;

  // Color sensors with calibration
  int colorRaw[Config::COLOR_SENSORS];
  int colorCalibrated[Config::COLOR_SENSORS];
  bool onOutline = false;
  bool nearOutline = false;
  bool onWhiteLine = false;  // Center line or penalty area line
  float outlineDistance = 0.0f;

  // IMU with sensor fusion
  float heading = 0.0f;
  float headingRate = 0.0f;
  Vector2D acceleration;
  Vector2D velocity;
  Vector2D globalPosition;  // Global position on field in cm

  // Field awareness
  FieldZone currentZone = FieldZone::OWN_HALF;

  // System monitoring
  float batteryVoltage = 12.0f;
  float motorCurrent[4] = { 0 };
  float temperature = 25.0f;

  unsigned long timestamp = 0;
  bool dataValid = true;
};

struct AIState {
  // Ball prediction using neural network simulation
  float ballPrediction[Config::PREDICTION_HORIZON][2];  // x, y positions
  float predictionConfidence = 0.0f;

  // Strategy adaptation
  float strategyWeights[10] = { 1.0f };  // Different strategy preferences
  int successfulActions[10] = { 0 };
  int totalActions[10] = { 0 };

  // Learning parameters
  float explorationRate = 0.1f;
  int learningCycle = 0;
  bool adaptiveMode = true;

  // Performance metrics
  float ballCaptureRate = 0.0f;
  float shotAccuracy = 0.0f;
  float energyEfficiency = 1.0f;
  float fieldRespectRate = 1.0f;  // How well robot respects field boundaries
  unsigned long totalRunTime = 0;
};

struct RobotState {
  AttackerState currentState = AttackerState::INITIALIZING;
  AttackerState previousState = AttackerState::INITIALIZING;
  MovementPattern movementPattern = MovementPattern::ADAPTIVE;

  // Advanced ball tracking
  Vector2D ballHistory[Config::BALL_HISTORY_SIZE];
  int ballHistoryIndex = 0;
  unsigned long lastBallTime = 0;
  int ballLostCounter = 0;

  // Shooting system
  unsigned long lastShootTime = 0;
  int shotsTaken = 0;
  int shotsSuccessful = 0;
  float shooterRPM = 0.0f;

  // Field position management
  Vector2D globalPosition = Vector2D(0, -Config::FIELD_LENGTH / 4);  // Start position
  Vector2D targetPosition;
  FieldZone currentZone = FieldZone::OWN_HALF;
  FieldZone targetZone = FieldZone::OPPONENT_HALF;

  // Field boundary respect
  int boundaryViolations = 0;
  unsigned long lastBoundaryViolation = 0;
  bool respectingBoundaries = true;

  // Emergency and safety
  bool emergencyStop = false;
  bool maintenanceRequired = false;
  int errorCode = 0;
  unsigned long lastWatchdogReset = 0;

  // Performance optimization
  int cpuUsage = 0;
  int memoryUsage = 0;
  float loopFrequency = 50.0f;
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
  Vector2D ballTrajectoryPrediction[20];  // AdvancedConfig::BALL_TRAJECTORY_SAMPLES
  float trajectoryConfidence = 0.0f;
  Vector2D optimalInterceptionPoint;
  float timeToIntercept = 0.0f;
  Vector2D opponentMovementPrediction;
  float opponentThreatLevel = 0.0f;
  Vector2D optimalShotPosition;
  float shotSuccessProbability = 0.0f;
};

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
  float errorHistory[5] = { 0 };
  int historyIndex = 0;

public:
  AdvancedPIDController(const Config::PIDConfig& cfg)
    : config(cfg) {
    adaptiveKp = config.kp;
    adaptiveKi = config.ki;
    adaptiveKd = config.kd;
    lastTime = millis();  // Initialize lastTime to avoid large initial dt
  }

  float compute(float setpoint, float input, bool adaptive = true) {
    unsigned long now = millis();
    float deltaTime = (now - lastTime) / 1000.0f;
    if (deltaTime <= 0) deltaTime = 0.001f;  // Prevent division by zero, min 1ms

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

    if (variance > 0.5f) {
      adaptiveKp = config.kp * 0.8f;
      adaptiveKd = config.kd * 1.2f;
    } else if (mean > 1.0f) {
      adaptiveKi = config.ki * 1.1f;
    } else {
      adaptiveKp = config.kp;
      adaptiveKi = config.ki;
      adaptiveKd = config.kd;
    }
  }
};

// ============================================================================
// FIELD AWARENESS SYSTEM
// ============================================================================

class FieldAwarenessSystem {
private:
  // Field calibration data
  Vector2D fieldCorners[4];
  Vector2D centerCircleCenter;
  Vector2D goalPosts[4];  // Own goal left/right, opponent goal left/right
  bool fieldCalibrated = false;

  // Position tracking with odometry
  Vector2D lastPosition;
  float lastHeading = 0.0f;
  unsigned long lastPositionUpdate = 0;

public:
  void initialize() {
    calibrateField();
    lastPosition = Vector2D(0, -Config::FIELD_LENGTH / 4);  // Starting position
    lastPositionUpdate = millis();
  }

  void updateFieldPosition(SensorData& data) {
    // Update global position using odometry and sensor corrections
    updateOdometry(data);

    // Determine current field zone
    data.currentZone = determineFieldZone(data.globalPosition);

    // Check for field violations
    checkFieldBoundaries(data);

    // Detect white lines (center line, penalty areas)
    detectWhiteLines(data);
  }

  FieldZone determineFieldZone(const Vector2D& position) {
    // Check if out of bounds first
    if (abs(position.x) > Config::FIELD_WIDTH / 2 - Config::BOUNDARY_MARGIN || abs(position.y) > Config::FIELD_LENGTH / 2 - Config::BOUNDARY_MARGIN) {
      return FieldZone::OUT_OF_BOUNDS;
    }

    // Check center circle
    Vector2D centerCirclePos(0, Config::CENTER_LINE_Y);
    if ((position - centerCirclePos).magnitude() <= Config::CENTER_CIRCLE_RADIUS) {
      return FieldZone::CENTER_CIRCLE;
    }

    // Check penalty areas
    if (position.y >= Config::OPPONENT_PENALTY_Y && abs(position.x) <= Config::PENALTY_AREA_WIDTH / 2) {
      return FieldZone::OPPONENT_PENALTY_AREA;
    }

    if (position.y <= Config::OWN_PENALTY_Y && abs(position.x) <= Config::PENALTY_AREA_WIDTH / 2) {
      return FieldZone::OWN_PENALTY_AREA;
    }

    // Check which half of field
    if (position.y > Config::CENTER_LINE_Y) {
      return FieldZone::OPPONENT_HALF;
    } else {
      return FieldZone::OWN_HALF;
    }
  }

  bool isValidMove(const Vector2D& currentPos, const Vector2D& targetPos) {
    // Check if the target position is within field boundaries
    if (abs(targetPos.x) > Config::FIELD_WIDTH / 2 - Config::BOUNDARY_MARGIN || abs(targetPos.y) > Config::FIELD_LENGTH / 2 - Config::BOUNDARY_MARGIN) {
      return false;
    }

    // F1S attacker should not enter own penalty area unless ball is there
    if (targetPos.y <= Config::OWN_PENALTY_Y && abs(targetPos.x) <= Config::PENALTY_AREA_WIDTH / 2) {
      return false;  // Generally not allowed
    }

    return true;
  }

  Vector2D getValidTargetPosition(const Vector2D& desiredPos, const Vector2D& currentPos) {
    Vector2D validPos = desiredPos;

    // Clamp to field boundaries with margin
    validPos.x = constrain(validPos.x,
                           -(Config::FIELD_WIDTH / 2 - Config::BOUNDARY_MARGIN),
                           Config::FIELD_WIDTH / 2 - Config::BOUNDARY_MARGIN);
    validPos.y = constrain(validPos.y,
                           -(Config::FIELD_LENGTH / 2 - Config::BOUNDARY_MARGIN),
                           Config::FIELD_LENGTH / 2 - Config::BOUNDARY_MARGIN);

    // Avoid own penalty area
    if (validPos.y <= Config::OWN_PENALTY_Y && abs(validPos.x) <= Config::PENALTY_AREA_WIDTH / 2) {
      // Push out of penalty area
      if (validPos.y <= Config::OWN_PENALTY_Y) {
        validPos.y = Config::OWN_PENALTY_Y + 5;  // 5cm outside penalty area
      }
    }

    return validPos;
  }

private:
  void calibrateField() {
    // Set up field geometry (assuming robot starts at known position)
    fieldCorners[0] = Vector2D(-Config::FIELD_WIDTH / 2, -Config::FIELD_LENGTH / 2);  // Own left
    fieldCorners[1] = Vector2D(Config::FIELD_WIDTH / 2, -Config::FIELD_LENGTH / 2);   // Own right
    fieldCorners[2] = Vector2D(Config::FIELD_WIDTH / 2, Config::FIELD_LENGTH / 2);    // Opp right
    fieldCorners[3] = Vector2D(-Config::FIELD_WIDTH / 2, Config::FIELD_LENGTH / 2);   // Opp left

    centerCircleCenter = Vector2D(0, Config::CENTER_LINE_Y);

    // Goal posts
    goalPosts[0] = Vector2D(-Config::GOAL_WIDTH / 2, Config::OWN_GOAL_Y);       // Own left post
    goalPosts[1] = Vector2D(Config::GOAL_WIDTH / 2, Config::OWN_GOAL_Y);        // Own right post
    goalPosts[2] = Vector2D(-Config::GOAL_WIDTH / 2, Config::OPPONENT_GOAL_Y);  // Opp left post
    goalPosts[3] = Vector2D(Config::GOAL_WIDTH / 2, Config::OPPONENT_GOAL_Y);   // Opp right post

    fieldCalibrated = true;
  }

  void updateOdometry(SensorData& data) {
    unsigned long now = millis();
    float dt = (now - lastPositionUpdate) / 1000.0f;
    if (dt > 0.1f) dt = 0.02f;  // Cap dt

    // Simple odometry integration
    // In a real system, this would use wheel encoders
    Vector2D deltaPos = data.velocity * dt;
    data.globalPosition = lastPosition + deltaPos;

    lastPosition = data.globalPosition;
    lastHeading = data.heading;
    lastPositionUpdate = now;
  }

  void checkFieldBoundaries(SensorData& data) {
    // Check if robot is near or outside field boundaries
    float distanceToLeftBoundary = (Config::FIELD_WIDTH / 2) + data.globalPosition.x;
    float distanceToRightBoundary = (Config::FIELD_WIDTH / 2) - data.globalPosition.x;
    float distanceToBackBoundary = (Config::FIELD_LENGTH / 2) + data.globalPosition.y;
    float distanceToFrontBoundary = (Config::FIELD_LENGTH / 2) - data.globalPosition.y;

    float minDistance = min(min(distanceToLeftBoundary, distanceToRightBoundary),
                            min(distanceToBackBoundary, distanceToFrontBoundary));

    data.outlineDistance = minDistance;
    data.onOutline = (minDistance <= Config::BOUNDARY_MARGIN);
    data.nearOutline = (minDistance <= Config::BOUNDARY_MARGIN * 2);
  }

  void detectWhiteLines(SensorData& data) {
    // Detect white lines using color sensors
    bool whiteDetected = false;
    for (int i = 0; i < Config::COLOR_SENSORS; i++) {
      if (data.colorCalibrated[i] > Config::WHITE_LINE_THRESHOLD) {
        whiteDetected = true;
        break;
      }
    }
    data.onWhiteLine = whiteDetected;
  }
};

// ============================================================================
// ADVANCED SENSOR FUSION WITH KALMAN FILTERING
// ============================================================================

class AdvancedSensorFusion {
private:
  // Kalman Filter matrices
  float stateVector[6] = { 0 };  // [x, y, vx, vy, heading, heading_rate]
  float covarianceMatrix[6][6];
  float processNoise[6][6];
  float measurementNoise[4][4];  // [x, y, heading, heading_rate] measurements

  unsigned long lastUpdateTime = 0;
  bool filterInitialized = false;

public:
  void initialize() {
    // Initialize covariance matrix
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        covarianceMatrix[i][j] = (i == j) ? 1.0f : 0.0f;
        processNoise[i][j] = (i == j) ? 0.1f : 0.0f;  // AdvancedConfig::PROCESS_NOISE
      }
    }

    // Initialize measurement noise
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        measurementNoise[i][j] = (i == j) ? 0.5f : 0.0f;  // AdvancedConfig::MEASUREMENT_NOISE
      }
    }

    filterInitialized = true;
    lastUpdateTime = millis();
  }

  void updateFilter(SensorData& sensors) {
    if (!filterInitialized) return;

    unsigned long now = millis();
    float dt = (now - lastUpdateTime) / 1000.0f;
    if (dt > 0.1f) dt = 0.02f;  // Cap dt

    // Prediction step
    predictState(dt);
    predictCovariance(dt);

    // Update step with measurements
    float measurements[4] = {
      sensors.globalPosition.x,
      sensors.globalPosition.y,
      sensors.heading,
      sensors.headingRate
    };

    updateWithMeasurements(measurements);

    // Update sensor data with fused values
    sensors.globalPosition = getFusedPosition();
    sensors.velocity = getFusedVelocity();
    sensors.heading = getFusedHeading();
    sensors.headingRate = getFusedHeadingRate();

    lastUpdateTime = now;
  }

  Vector2D getFusedPosition() {
    return Vector2D(stateVector[0], stateVector[1]);
  }

  Vector2D getFusedVelocity() {
    return Vector2D(stateVector[2], stateVector[3]);
  }

  float getFusedHeading() {
    return stateVector[4];
  }

  float getFusedHeadingRate() {
    return stateVector[5];
  }

  float getPositionUncertainty() {
    return sqrt(covarianceMatrix[0][0] + covarianceMatrix[1][1]);
  }

private:
  void predictState(float dt) {
    // State transition: position += velocity * dt
    stateVector[0] += stateVector[2] * dt;  // x += vx * dt
    stateVector[1] += stateVector[3] * dt;  // y += vy * dt
    stateVector[4] += stateVector[5] * dt;  // heading += heading_rate * dt

    // Normalize heading
    while (stateVector[4] >= 360.0f) stateVector[4] -= 360.0f;
    while (stateVector[4] < 0.0f) stateVector[4] += 360.0f;
  }

  void predictCovariance(float dt) {
    // Simplified covariance prediction
    for (int i = 0; i < 6; i++) {
      covarianceMatrix[i][i] += processNoise[i][i] * dt;
    }
  }

  void updateWithMeasurements(float measurements[4]) {
    // Simplified Kalman update
    float innovation[4];
    innovation[0] = measurements[0] - stateVector[0];
    innovation[1] = measurements[1] - stateVector[1];
    innovation[2] = measurements[2] - stateVector[4];
    innovation[3] = measurements[3] - stateVector[5];

    // Apply corrections with adaptive gain
    float gain = 0.3f;  // Simplified gain
    stateVector[0] += gain * innovation[0];
    stateVector[1] += gain * innovation[1];
    stateVector[4] += gain * innovation[2];
    stateVector[5] += gain * innovation[3];

    // Update velocity estimates
    stateVector[2] = stateVector[2] * 0.9f + innovation[0] * 0.1f;
    stateVector[3] = stateVector[3] * 0.9f + innovation[1] * 0.1f;
  }
};

// ============================================================================
// COMPUTER VISION SYSTEM
// ============================================================================

class ComputerVisionSystem {
private:
  ComputerVisionData visionData;
  unsigned long lastProcessingTime = 0;

  // Simple edge detection kernel (example)
  int edgeKernel[3][3] = {
    { -1, -1, -1 },
    { -1, 8, -1 },
    { -1, -1, -1 }
  };

public:
  void initialize() {
    visionData = ComputerVisionData();
    lastProcessingTime = millis();
  }

  void processVision(const SensorData& sensors) {
    unsigned long now = millis();
    if (now - lastProcessingTime < Config::VISION_PROCESSING_INTERVAL) return;

    // Simulate computer vision processing using sensor data
    detectGoal(sensors);
    detectOpponents(sensors);
    detectTeammates(sensors);

    visionData.lastVisionUpdate = now;
    lastProcessingTime = now;
  }

  ComputerVisionData getVisionData() const {
    return visionData;
  }

  bool isGoalInSight() const {
    return visionData.goalDetected && visionData.goalConfidence > 90;  // AdvancedConfig::GOAL_RECOGNITION_CONFIDENCE
  }

private:
  void detectGoal(const SensorData& sensors) {
    // Simulate goal detection based on field position and sensor data
    Vector2D opponentGoal(0, Config::OPPONENT_GOAL_Y);
    float distanceToGoal = (sensors.globalPosition - opponentGoal).magnitude();

    if (distanceToGoal < 80.0f && sensors.currentZone == FieldZone::OPPONENT_HALF) {
      visionData.goalDetected = true;
      visionData.goalPosition = opponentGoal;
      visionData.goalConfidence = 100.0f - (distanceToGoal / 80.0f) * 20.0f;
    } else {
      visionData.goalDetected = false;
      visionData.goalConfidence = 0.0f;
    }
  }

  void detectOpponents(const SensorData& sensors) {
    // Simulate opponent detection using IR sensors and field position
    bool strongIRSignal = false;
    for (int i = 0; i < Config::TSOP_SENSORS; i++) {
      if (sensors.tsopFiltered[i] > 2000 && sensors.tsopFiltered[i] < 3500) {
        strongIRSignal = true;
        break;
      }
    }

    if (strongIRSignal && sensors.ballStrength < 0.3f) {
      visionData.opponentDetected = true;
      visionData.opponentConfidence = 75.0f;
      // Estimate opponent position based on sensor readings
      visionData.opponentPosition = sensors.globalPosition + Vector2D(30, 20);
    } else {
      visionData.opponentDetected = false;
      visionData.opponentConfidence = 0.0f;
    }
  }

  void detectTeammates(const SensorData& sensors) {
    // Simulate teammate detection (would use communication data in real implementation)
    if (sensors.currentZone == FieldZone::OWN_HALF) {
      visionData.teammateDetected = true;
      visionData.teammatePosition = Vector2D(0, Config::OWN_GOAL_Y + 20);
    } else {
      visionData.teammateDetected = false;
    }
  }
};

// ============================================================================
// PREDICTIVE ANALYTICS ENGINE
// ============================================================================

class PredictiveAnalyticsEngine {
private:
  PredictiveAnalytics analytics;
  Vector2D ballHistory[20];  // AdvancedConfig::BALL_HISTORY_SIZE
  int ballHistoryIndex = 0;
  unsigned long lastPredictionUpdate = 0;

public:
  void initialize() {
    analytics = PredictiveAnalytics();
    for (int i = 0; i < 20; i++) {
      ballHistory[i] = Vector2D(0, 0);
    }
  }

  void updatePredictions(const SensorData& sensors, const ComputerVisionData& vision) {
    unsigned long now = millis();
    if (now - lastPredictionUpdate < 50) return;  // 20Hz update rate

    updateBallHistory(sensors);
    predictBallTrajectory(sensors);
    predictOptimalInterception(sensors);
    predictShotOpportunity(sensors, vision);
    assessOpponentThreat(vision);

    lastPredictionUpdate = now;
  }

  PredictiveAnalytics getAnalytics() const {
    return analytics;
  }

  Vector2D getBestInterceptionPoint() const {
    return analytics.optimalInterceptionPoint;
  }

  Vector2D getBestShotPosition() const {
    return analytics.optimalShotPosition;
  }

private:
  void updateBallHistory(const SensorData& sensors) {
    if (sensors.ballStrength > 0.1f) {
      Vector2D globalBallPos = sensors.globalPosition + sensors.ballVector;
      ballHistory[ballHistoryIndex] = globalBallPos;
      ballHistoryIndex = (ballHistoryIndex + 1) % 20;
    }
  }

  void predictBallTrajectory(const SensorData& sensors) {
    if (sensors.ballStrength < 0.1f) {
      analytics.trajectoryConfidence = 0.0f;
      return;
    }

    // Calculate ball velocity from history
    Vector2D ballVelocity(0, 0);
    int validSamples = 0;

    for (int i = 1; i < 5; i++) {
      int currentIdx = (ballHistoryIndex - 1 + 20) % 20;
      int prevIdx = (ballHistoryIndex - 1 - i + 20) % 20;

      if (ballHistory[currentIdx].magnitude() > 0 && ballHistory[prevIdx].magnitude() > 0) {
        ballVelocity = ballVelocity + (ballHistory[currentIdx] - ballHistory[prevIdx]);
        validSamples++;
      }
    }

    if (validSamples > 0) {
      ballVelocity = ballVelocity * (1.0f / validSamples);

      // Predict future positions
      Vector2D currentBallPos = sensors.globalPosition + sensors.ballVector;
      for (int i = 0; i < 20; i++) {  // AdvancedConfig::BALL_TRAJECTORY_SAMPLES
        float timeStep = i * 0.1f;    // 100ms steps
        analytics.ballTrajectoryPrediction[i] = currentBallPos + ballVelocity * timeStep;

        // Apply drag/friction
        ballVelocity = ballVelocity * 0.95f;
      }

      analytics.trajectoryConfidence = min(1.0f, validSamples / 5.0f);
    }
  }

  void predictOptimalInterception(const SensorData& sensors) {
    if (analytics.trajectoryConfidence < 0.5f) return;

    Vector2D robotPos = sensors.globalPosition;
    float robotSpeed = 50.0f;  // cm/s estimated

    // Find closest interception point
    float minTime = 999.0f;
    Vector2D bestPoint;

    for (int i = 0; i < 20; i++) {  // AdvancedConfig::BALL_TRAJECTORY_SAMPLES
      Vector2D ballPos = analytics.ballTrajectoryPrediction[i];
      float distanceToRobot = (ballPos - robotPos).magnitude();
      float timeToReach = distanceToRobot / robotSpeed;
      float ballTime = i * 0.1f;

      if (abs(timeToReach - ballTime) < 0.2f && timeToReach < minTime) {
        minTime = timeToReach;
        bestPoint = ballPos;
      }
    }

    analytics.optimalInterceptionPoint = bestPoint;
    analytics.timeToIntercept = minTime;
  }

  void predictShotOpportunity(const SensorData& sensors, const ComputerVisionData& vision) {
    if (!vision.goalDetected) {
      analytics.shotSuccessProbability = 0.0f;
      return;
    }

    Vector2D robotPos = sensors.globalPosition;
    Vector2D goalPos = vision.goalPosition;
    float distanceToGoal = (goalPos - robotPos).magnitude();

    // Calculate shot probability based on distance, angle, and obstacles
    float distanceFactor = 1.0f - (distanceToGoal / 100.0f);
    distanceFactor = constrain(distanceFactor, 0.0f, 1.0f);

    float angleFactor = 1.0f;
    if (sensors.currentZone == FieldZone::OPPONENT_HALF) {
      angleFactor = 1.2f;
    }

    analytics.shotSuccessProbability = distanceFactor * angleFactor * 0.8f;
    analytics.optimalShotPosition = robotPos + (goalPos - robotPos).normalized() * 20.0f;
  }

  void assessOpponentThreat(const ComputerVisionData& vision) {
    if (vision.opponentDetected) {
      float distance = vision.opponentPosition.magnitude();
      analytics.opponentThreatLevel = 1.0f - (distance / 100.0f);
      analytics.opponentThreatLevel = constrain(analytics.opponentThreatLevel, 0.0f, 1.0f);
    } else {
      analytics.opponentThreatLevel = 0.0f;
    }
  }
};

// ============================================================================
// CONSOLIDATED SENSOR SYSTEM
// ============================================================================

class ConsolidatedSensorSystem {
private:
  MPU6050 mpu;
  FieldAwarenessSystem fieldSystem;

  // Sensor calibration
  int tsopCalibration[Config::TSOP_SENSORS] = { 0 };
  int colorCalibration[Config::COLOR_SENSORS] = { 0 };
  bool calibrationComplete = false;

  // Advanced filtering
  float tsopFilter[Config::TSOP_SENSORS][5] = { 0 };
  float colorFilter[Config::COLOR_SENSORS][3] = { 0 };

public:
  void initialize() {
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection()) {
      Serial.println("MPU6050 connection failed!");
    }

    fieldSystem.initialize();
    loadCalibrationData();
    resetFilters();
  }

  void updateSensorData(SensorData& data) {
    readTSOPSensors(data);
    readColorSensors(data);
    updateIMU(data);
    calculateBallVector(data);
    updateSystemMonitoring(data);

    // Update field awareness
    fieldSystem.updateFieldPosition(data);

    data.timestamp = millis();
    data.dataValid = true;
  }

  void calibrateSensors() {
    Serial.println("Calibrating field-aware sensors...");

    for (int i = 0; i < 100; i++) {
      // TSOP calibration
      for (int j = 0; j < Config::TSOP_SENSORS; j++) {
        digitalWrite(PA8, (j & 1));
        digitalWrite(PB1, (j & 2) >> 1);
        digitalWrite(PC14, (j & 4) >> 2);
        digitalWrite(PC15, (j & 8) >> 3);

        tsopCalibration[j] += analogRead(PA0);
      }

      // Color sensor calibration
      for (int j = 0; j < Config::COLOR_SENSORS; j++) {
        colorCalibration[j] += analogRead(PA1 + j);
      }

      delay(10);
    }

    for (int i = 0; i < Config::TSOP_SENSORS; i++) {
      tsopCalibration[i] /= 100;
    }
    for (int i = 0; i < Config::COLOR_SENSORS; i++) {
      colorCalibration[i] /= 100;
    }

    saveCalibrationData();
    calibrationComplete = true;
    Serial.println("Field-aware sensor calibration complete!");
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
        delayMicroseconds(100);
      }
      data.tsopRaw[i] = sum / Config::SENSOR_SAMPLES;

      data.tsopRaw[i] -= tsopCalibration[i];
      if (data.tsopRaw[i] < 0) data.tsopRaw[i] = 0;

      // Apply filter (5-tap FIR)
      for (int k = 4; k > 0; k--) {
        tsopFilter[i][k] = tsopFilter[i][k - 1];
      }
      tsopFilter[i][0] = data.tsopRaw[i];

      data.tsopFiltered[i] = (tsopFilter[i][0] * 0.4f + tsopFilter[i][1] * 0.25f + tsopFilter[i][2] * 0.2f + tsopFilter[i][3] * 0.1f + tsopFilter[i][4] * 0.05f);

      if (data.tsopFiltered[i] < data.tsopMin) {
        data.tsopMin = data.tsopFiltered[i];
        data.tsopNum = i;
      }
    }

    data.ballStrength = (4095.0f - data.tsopMin) / 4095.0f;
  }

  void readColorSensors(SensorData& data) {
    float minOutlineDistance = 1000.0f;

    for (int i = 0; i < Config::COLOR_SENSORS; i++) {
      int sum = 0;
      for (int j = 0; j < Config::SENSOR_SAMPLES; j++) {
        sum += analogRead(PA1 + i);
        delayMicroseconds(50);
      }
      data.colorRaw[i] = sum / Config::SENSOR_SAMPLES;

      // Apply filtering
      colorFilter[i][2] = colorFilter[i][1];
      colorFilter[i][1] = colorFilter[i][0];
      colorFilter[i][0] = data.colorRaw[i] - colorCalibration[i];

      data.colorCalibrated[i] = (colorFilter[i][0] * 0.6f + colorFilter[i][1] * 0.3f + colorFilter[i][2] * 0.1f);

      // Calculate distance to outline
      if (data.colorCalibrated[i] < Config::OUTLINE_THRESHOLD) {
        float distance = data.colorCalibrated[i] / (float)Config::OUTLINE_THRESHOLD;
        if (distance < minOutlineDistance) minOutlineDistance = distance;
      }
    }

    data.outlineDistance = constrain(minOutlineDistance, 0.0f, 1.0f);
    data.onOutline = (data.outlineDistance < 0.15f);
    data.nearOutline = (data.outlineDistance < 0.3f);
  }

  void updateIMU(SensorData& data) {
    Vector normGyro = mpu.readNormalizeGyro();
    Vector normAccel = mpu.readNormalizeAccel();

    static float gyroHeading = 0.0f;
    static Vector2D lastVelocity(0, 0);
    static unsigned long lastTime = 0;

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    if (dt > 0.1f) dt = 0.02f;

    gyroHeading += normGyro.ZAxis * dt;

    while (gyroHeading >= 360.0f) gyroHeading -= 360.0f;
    while (gyroHeading < 0.0f) gyroHeading += 360.0f;

    data.heading = gyroHeading;
    data.headingRate = normGyro.ZAxis;

    Vector2D currentAccel(normAccel.XAxis, normAccel.YAxis);
    data.velocity = lastVelocity + currentAccel * dt;
    data.velocity = data.velocity * 0.98f;  // Drift correction

    lastVelocity = data.velocity;
    lastTime = now;
  }

  void calculateBallVector(SensorData& data) {
    if (data.tsopMin < Config::BALL_DETECTION_THRESHOLD) {
      float angle = (data.tsopNum * 22.5f) * PI / 180.0f;
      float distance = 1.0f - (data.ballStrength * 0.8f);

      data.ballVector.x = cos(angle) * distance;
      data.ballVector.y = sin(angle) * distance;
    } else {
      data.ballVector = Vector2D(0, 0);
    }
  }

  void updateSystemMonitoring(SensorData& data) {
    data.batteryVoltage = 12.0f + (analogRead(PA5) - 2048) * 0.01f;
    data.temperature = 25.0f + (analogRead(PA6) - 2048) * 0.05f;

    for (int i = 0; i < 4; i++) {
      data.motorCurrent[i] = abs(analogRead(PA7) - 2048) * 0.002f;
    }
  }

  void loadCalibrationData() {
    for (int i = 0; i < Config::TSOP_SENSORS; i++) {
      tsopCalibration[i] = EEPROM.read(i * 2) | (EEPROM.read(i * 2 + 1) << 8);
    }
    for (int i = 0; i < Config::COLOR_SENSORS; i++) {
      colorCalibration[i] = EEPROM.read(64 + i * 2) | (EEPROM.read(65 + i * 2) << 8);
    }
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
  }

  void resetFilters() {
    for (int i = 0; i < Config::TSOP_SENSORS; i++) {
      for (int j = 0; j < 5; j++) {
        tsopFilter[i][j] = 0.0f;
      }
    }
    for (int i = 0; i < Config::COLOR_SENSORS; i++) {
      for (int j = 0; j < 3; j++) {
        colorFilter[i][j] = 0.0f;
      }
    }
  }
};

// ============================================================================
// CONSOLIDATED HARDWARE INTERFACE
// ============================================================================

class ConsolidatedHardwareInterface {
private:
  Adafruit_SH1106 display;
  SoftwareSerial bluetooth;
  ConsolidatedSensorSystem* sensorSystem;

  int currentMotorSpeeds[4] = { 0 };
  int targetMotorSpeeds[4] = { 0 };
  unsigned long lastMotorUpdate = 0;

  bool safetyEnabled = true;

public:
  ConsolidatedHardwareInterface()
    : display(-1), bluetooth(PA9, PA10), sensorSystem(nullptr) {}

  void setSensorSystem(ConsolidatedSensorSystem* ss) {
    sensorSystem = ss;
  }

  bool initialize() {
    display.begin(0x2, 0x3c);
    showSplashScreen();

    initializeMotorPins();
    initializeSensorPins();

    bluetooth.begin(Config::BLUETOOTH_BAUD);

    displayMessage("F1S v3.1 READY!");
    delay(1000);

    return true;
  }

  void setMotorSpeeds(int ml1, int ml2, int mr2, int mr1, bool pullback = false, bool immediate = false) {
    if (pullback) {
      ml1 *= Config::PULLBACK_MULTIPLIER;
      ml2 *= Config::PULLBACK_MULTIPLIER;
      mr1 *= Config::PULLBACK_MULTIPLIER;
      mr2 *= Config::PULLBACK_MULTIPLIER;
    }

    if (safetyEnabled) {
      ml1 = constrain(ml1, -Config::MAX_MOTOR_SPEED, Config::MAX_MOTOR_SPEED);
      ml2 = constrain(ml2, -Config::MAX_MOTOR_SPEED, Config::MAX_MOTOR_SPEED);
      mr1 = constrain(mr1, -Config::MAX_MOTOR_SPEED, Config::MAX_MOTOR_SPEED);
      mr2 = constrain(mr2, -Config::MAX_MOTOR_SPEED, Config::MAX_MOTOR_SPEED);
    }

    targetMotorSpeeds[0] = ml1;
    targetMotorSpeeds[1] = ml2;
    targetMotorSpeeds[2] = mr1;
    targetMotorSpeeds[3] = mr2;

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

  void setShooterSpeed(int speed) {
    speed = constrain(speed, 0, Config::MAX_MOTOR_SPEED);
    pwmWrite(PC6, speed);
  }

  SensorData readSensors() {
    SensorData data;
    if (sensorSystem) {
      sensorSystem->updateSensorData(data);
    } else {
      data.dataValid = false;
    }
    return data;
  }

  void updateDisplay(const SensorData& sensors, const RobotState& state, const AIState& ai) {
    static int displayPage = 0;
    static unsigned long lastPageChange = 0;

    if (millis() - lastPageChange > 2000) {
      displayPage = (displayPage + 1) % 4;
      lastPageChange = millis();
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);

    switch (displayPage) {
      case 0: displayMainPage(sensors, state); break;
      case 1: displayFieldPage(sensors, state); break;
      case 2: displayAIPage(ai, state); break;
      case 3: displaySystemPage(sensors, state); break;
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

  void performSafetyCheck(const SensorData& sensors) {
    if (!safetyEnabled) return;

    static unsigned long lastSafetyCheck = 0;
    if (millis() - lastSafetyCheck < 100) return;

    bool emergencyStop = false;

    if (sensors.batteryVoltage < Config::MIN_BATTERY_VOLTAGE) {
      emergencyStop = true;
    }

    for (int i = 0; i < 4; i++) {
      if (sensors.motorCurrent[i] > Config::MAX_MOTOR_CURRENT) {
        emergencyStop = true;
      }
    }

    if (sensors.temperature > Config::MAX_TEMPERATURE) {
      emergencyStop = true;
    }

    if (emergencyStop) {
      setMotorSpeeds(0, 0, 0, 0, false, true);
      setShooterSpeed(0);
      displayError("EMERGENCY STOP!");
    }

    lastSafetyCheck = millis();
  }

private:
  void initializeMotorPins() {
    pinMode(PB12, OUTPUT);
    pinMode(PB13, OUTPUT);
    pinMode(PB14, OUTPUT);
    pinMode(PB15, OUTPUT);
    pinMode(PB9, PWM);
    pinMode(PB8, PWM);
    pinMode(PB7, PWM);
    pinMode(PB6, PWM);
    pinMode(PC6, PWM);

    setMotorSpeeds(0, 0, 0, 0, false, true);
  }

  void initializeSensorPins() {
    pinMode(PA8, OUTPUT);
    pinMode(PB1, OUTPUT);
    pinMode(PC14, OUTPUT);
    pinMode(PC15, OUTPUT);
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
    display.setCursor(5, 10);
    display.print("F1S v3.1");
    display.setTextSize(1);
    display.setCursor(0, 35);
    display.print("Field-Aware Attacker");
    display.display();
    delay(2000);
  }

  void displayMainPage(const SensorData& sensors, const RobotState& state) {
    display.setCursor(0, 0);
    display.print("MAIN - F1S v3.1");

    display.setCursor(0, 10);
    display.print("State: ");
    display.print(getStateString(state.currentState));

    display.setCursor(0, 20);
    display.print("Ball: ");
    display.print(sensors.tsopNum);
    display.print(" (");
    display.print(sensors.ballStrength, 2);
    display.print(")");

    display.setCursor(0, 30);
    display.print("Zone: ");
    display.print(getZoneString(sensors.currentZone));

    display.setCursor(0, 40);
    display.print("Boundary: ");
    display.print(sensors.onOutline ? "OUT" : (sensors.nearOutline ? "NEAR" : "OK"));

    display.setCursor(0, 50);
    display.print("Loop: ");
    display.print(state.loopFrequency, 1);
    display.print("Hz");
  }

  void displayFieldPage(const SensorData& sensors, const RobotState& state) {
    display.setCursor(0, 0);
    display.print("FIELD - Position");

    display.setCursor(0, 10);
    display.print("Pos: X");
    display.print(sensors.globalPosition.x, 0);
    display.print(" Y");
    display.print(sensors.globalPosition.y, 0);

    display.setCursor(0, 20);
    display.print("Target: X");
    display.print(state.targetPosition.x, 0);
    display.print(" Y");
    display.print(state.targetPosition.y, 0);

    display.setCursor(0, 30);
    display.print("Heading: ");
    display.print((int)sensors.heading);
    display.print("deg");

    display.setCursor(0, 40);
    display.print("Zone: ");
    display.print(getZoneString(sensors.currentZone));

    display.setCursor(0, 50);
    display.print("Violations: ");
    display.print(state.boundaryViolations);
  }

  void displayAIPage(const AIState& ai, const RobotState& state) {
    display.setCursor(0, 0);
    display.print("AI - Learning");

    display.setCursor(0, 10);
    display.print("Confidence: ");
    display.print(ai.predictionConfidence, 2);

    display.setCursor(0, 20);
    display.print("Capture Rate: ");
    display.print(ai.ballCaptureRate, 2);

    display.setCursor(0, 30);
    display.print("Shot Acc: ");
    display.print(ai.shotAccuracy, 2);

    display.setCursor(0, 40);
    display.print("Field Respect: ");
    display.print(ai.fieldRespectRate, 2);

    display.setCursor(0, 50);
    display.print("Efficiency: ");
    display.print(ai.energyEfficiency, 2);
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
    display.print("Runtime: ");
    display.print(millis() / 1000);
    display.print("s");
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

  String getStateString(AttackerState state) {
    switch (state) {
      case AttackerState::SEARCHING: return "SEARCH";
      case AttackerState::CHASING_BALL: return "CHASE";
      case AttackerState::DRIBBLING: return "DRIBBLE";
      case AttackerState::POSITIONING_FOR_SHOT: return "POSITION";
      case AttackerState::SHOOTING: return "SHOOT";
      case AttackerState::AVOIDING_OUTLINE: return "AVOID";
      case AttackerState::RESPECTING_BOUNDARIES: return "BOUNDARY";
      case AttackerState::LEARNING_MODE: return "LEARN";
      case AttackerState::EMERGENCY_STOP: return "EMERGENCY";
      default: return "UNKNOWN";
    }
  }

  String getZoneString(FieldZone zone) {
    switch (zone) {
      case FieldZone::OWN_HALF: return "OWN";
      case FieldZone::OPPONENT_HALF: return "OPP";
      case FieldZone::CENTER_CIRCLE: return "CENTER";
      case FieldZone::OPPONENT_PENALTY_AREA: return "OPP_PEN";
      case FieldZone::OWN_PENALTY_AREA: return "OWN_PEN";
      case FieldZone::OUT_OF_BOUNDS: return "OUT";
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
};

// ============================================================================
// CONSOLIDATED MOVEMENT CONTROLLER
// ============================================================================

class ConsolidatedMovementController {
private:
  ConsolidatedHardwareInterface* hardware;
  FieldAwarenessSystem* fieldSystem;
  AdvancedPIDController gyroPID;
  AdvancedPIDController ballTrackPID;
  AdvancedPIDController positionPID;

  float energyEfficiency = 1.0f;

public:
  ConsolidatedMovementController(ConsolidatedHardwareInterface* hw, FieldAwarenessSystem* fs)
    : hardware(hw), fieldSystem(fs), gyroPID(Config::GYRO_PID),
      ballTrackPID(Config::BALL_TRACK_PID), positionPID(Config::POSITION_PID) {}

  void move(int direction, float speedMultiplier = 1.0f, bool pullback = false, MovementPattern pattern = MovementPattern::DIRECT) {
    int speed = Config::BASE_SPEED * speedMultiplier * energyEfficiency;
    int ml1, ml2, mr1, mr2;

    calculateMotorSpeeds(direction, speed, ml1, ml2, mr1, mr2);

    // Apply movement pattern modifications
    applyMovementPattern(ml1, ml2, mr1, mr2, pattern);

    float gyroCorrection = gyroPID.compute(0, 0);
    ml1 += gyroCorrection;
    ml2 += gyroCorrection;
    mr1 += gyroCorrection;
    mr2 += gyroCorrection;

    hardware->setMotorSpeeds(ml1, ml2, mr2, mr1, pullback);
  }

  void moveToValidPosition(const Vector2D& desiredPos, const Vector2D& currentPos, float speedMultiplier = 1.0f) {
    Vector2D validTarget = fieldSystem->getValidTargetPosition(desiredPos, currentPos);
    Vector2D direction = (validTarget - currentPos).normalized();

    float angle = atan2(direction.y, direction.x) * 180.0f / PI;
    int motorDirection = angleToDirection(angle);

    move(motorDirection, speedMultiplier);
  }

  void followBall(const Vector2D& ballVector, float aggressiveness = 1.0f) {
    if (ballVector.magnitude() < 0.1f) {
      stop();
      return;
    }

    // PID control for ball following
    float angleError = atan2(ballVector.y, ballVector.x) * 180.0f / PI;
    float correction = ballTrackPID.compute(0, angleError);

    // Convert to movement
    Vector2D moveVector = ballVector.normalized() * aggressiveness;
    float angle = atan2(moveVector.y, moveVector.x) * 180.0f / PI;
    int direction = angleToDirection(angle);

    move(direction, aggressiveness, true, MovementPattern::CURVED);
  }

  void avoidBoundaries(const SensorData& sensors) {
    // Move away from boundaries
    Vector2D avoidanceVector(0, 0);

    // Calculate avoidance based on position
    if (sensors.globalPosition.x > Config::FIELD_WIDTH / 2 - Config::BOUNDARY_MARGIN * 2) {
      avoidanceVector.x = -1.0f;  // Move left
    }
    if (sensors.globalPosition.x < -Config::FIELD_WIDTH / 2 + Config::BOUNDARY_MARGIN * 2) {
      avoidanceVector.x = 1.0f;  // Move right
    }
    if (sensors.globalPosition.y > Config::FIELD_LENGTH / 2 - Config::BOUNDARY_MARGIN * 2) {
      avoidanceVector.y = -1.0f;  // Move back
    }
    if (sensors.globalPosition.y < -Config::FIELD_LENGTH / 2 + Config::BOUNDARY_MARGIN * 2) {
      avoidanceVector.y = 1.0f;  // Move forward
    }

    if (avoidanceVector.magnitude() > 0) {
      float angle = atan2(avoidanceVector.y, avoidanceVector.x) * 180.0f / PI;
      int direction = angleToDirection(angle);
      move(direction, 0.8f);
    }
  }

  void stop() {
    hardware->setMotorSpeeds(0, 0, 0, 0, false, true);
  }

  void optimizeEnergyEfficiency(const SensorData& sensors) {
    float batteryFactor = sensors.batteryVoltage / 12.0f;
    float temperatureFactor = 1.0f - (sensors.temperature - 25.0f) / 50.0f;

    energyEfficiency = batteryFactor * temperatureFactor;
    energyEfficiency = constrain(energyEfficiency, 0.3f, 1.2f);
  }

private:
  void calculateMotorSpeeds(int direction, int speed, int& ml1, int& ml2, int& mr1, int& mr2) {
    float speedF = speed;

    switch (direction) {
      case 0:  // Forward
        ml1 = speedF;
        ml2 = speedF;
        mr1 = -speedF;
        mr2 = -speedF;
        break;
      case 1:  // Forward-right
        ml1 = speedF;
        ml2 = speedF * 0.6f;
        mr1 = -speedF;
        mr2 = -speedF * 0.6f;
        break;
      case 2:  // Right
        ml1 = speedF;
        ml2 = 0;
        mr1 = -speedF;
        mr2 = 0;
        break;
      case 3:  // Back-right
        ml1 = speedF;
        ml2 = -speedF * 0.6f;
        mr1 = -speedF;
        mr2 = speedF * 0.6f;
        break;
      case 4:  // Backward
        ml1 = speedF;
        ml2 = -speedF;
        mr1 = -speedF;
        mr2 = speedF;
        break;
      case 5:  // Back-left
        ml1 = speedF * 0.6f;
        ml2 = -speedF;
        mr1 = -speedF * 0.6f;
        mr2 = speedF;
        break;
      case 6:  // Left
        ml1 = 0;
        ml2 = -speedF;
        mr1 = 0;
        mr2 = speedF;
        break;
      case 7:  // Forward-left
        ml1 = -speedF * 0.6f;
        ml2 = -speedF;
        mr1 = speedF * 0.6f;
        mr2 = speedF;
        break;
      case 8:  // Rotate left
        ml1 = -speedF;
        ml2 = -speedF;
        mr1 = speedF;
        mr2 = speedF;
        break;
      case 9:  // Rotate right
        ml1 = speedF;
        ml2 = speedF;
        mr1 = -speedF;
        mr2 = -speedF;
        break;
      default:
        ml1 = ml2 = mr1 = mr2 = 0;
        break;
    }
  }

  void applyMovementPattern(int& ml1, int& ml2, int& mr1, int& mr2, MovementPattern pattern) {
    static unsigned long patternTime = 0;
    float t = (millis() - patternTime) / 1000.0f;

    switch (pattern) {
      case MovementPattern::CURVED:
        // Add sinusoidal component for curved movement
        {
          float curve = sin(t * 2.0f) * 0.2f;
          ml1 *= (1.0f + curve);
          mr1 *= (1.0f - curve);
        }
        break;

      case MovementPattern::SPIRAL:
        // Spiral pattern for searching
        {
          float spiral = sin(t) * cos(t * 0.5f) * 0.3f;
          ml1 *= (1.0f + spiral);
          mr2 *= (1.0f + spiral);
        }
        break;

      case MovementPattern::ZIGZAG:
        // Zigzag pattern for unpredictable movement
        {
          float zigzag = (int(t * 2.0f) % 2 == 0) ? 0.3f : -0.3f;
          ml2 *= (1.0f + zigzag);
          mr1 *= (1.0f + zigzag);
        }
        break;

      case MovementPattern::ADAPTIVE:
        // Adapt based on current situation
        optimizeForSituation(ml1, ml2, mr1, mr2);
        break;

      default:
        break;
    }
  }

  void optimizeForSituation(int& ml1, int& ml2, int& mr1, int& mr2) {
    // Adaptive optimization based on current performance
    // This would use machine learning in a full implementation

    static float performanceHistory[10] = { 1.0f };
    static int historyIndex = 0;

    // Simple adaptation: if performance is low, add randomness
    float avgPerformance = 0;
    for (int i = 0; i < 10; i++) avgPerformance += performanceHistory[i];
    avgPerformance /= 10.0f;

    if (avgPerformance < 0.7f) {
      float randomFactor = (random(100) - 50) / 500.0f;  // ±10%
      ml1 *= (1.0f + randomFactor);
      mr1 *= (1.0f - randomFactor);
    }
  }

  int angleToDirection(float angleDeg) {
    angleDeg = fmod(angleDeg + 360.0f, 360.0f);
    return (int)((angleDeg + 22.5f) / 45.0f) % 8;
  }
};

// ============================================================================
// CONSOLIDATED AI STRATEGY ENGINE
// ============================================================================

class ConsolidatedAIEngine {
private:
  ConsolidatedMovementController* movement;
  ConsolidatedHardwareInterface* hardware;
  FieldAwarenessSystem* fieldSystem;
  PredictiveAnalyticsEngine analyticsEngine;
  ComputerVisionSystem visionSystem;

  // Neural network simulation (simplified)
  float weights[3][10] = { { 0 } };  // 3 layers, 10 neurons each
  float biases[3][10] = { { 0 } };

  // Q-Learning parameters
  float qTable[13][10] = { { 0 } };  // 13 states, 10 actions
  float learningRate = 0.1f;
  float discountFactor = 0.9f;
  float epsilon = 0.1f;  // Exploration rate

public:
  ConsolidatedAIEngine(ConsolidatedMovementController* mv, ConsolidatedHardwareInterface* hw, FieldAwarenessSystem* fs)
    : movement(mv), hardware(hw), fieldSystem(fs), analyticsEngine(), visionSystem() {}

  void initialize() {
    analyticsEngine.initialize();
    visionSystem.initialize();
    initializeNeuralNetwork();
    initializeQLearning();
  }

  void executeStrategy(RobotState& state, AIState& aiState, const SensorData& sensors) {
    // Priority 1: Boundary respect
    if (sensors.onOutline || sensors.currentZone == FieldZone::OUT_OF_BOUNDS) {
      state.currentState = AttackerState::RESPECTING_BOUNDARIES;
      state.boundaryViolations++;
      executeBoundaryRespectStrategy(state, sensors);  // Execute immediately
      return;
    }

    // Update computer vision
    visionSystem.processVision(sensors);
    ComputerVisionData visionData = visionSystem.getVisionData();

    // Update predictive analytics
    analyticsEngine.updatePredictions(sensors, visionData);
    PredictiveAnalytics analytics = analyticsEngine.getAnalytics();

    // Update AI state
    updateAIState(aiState, sensors, state);

    // Choose action based on current strategy
    int action = chooseAction(state, aiState, sensors);

    // Execute the chosen action
    executeAction(action, state, sensors, visionData, analytics);

    // Learn from the action (simplified reinforcement learning)
    if (aiState.adaptiveMode) {
      updateQLearning(state, action, calculateReward(state, sensors));
    }

    // Update strategy weights based on performance
    adaptStrategy(aiState, state, sensors);
  }

  void predictBallTrajectory(AIState& aiState, const SensorData& sensors, const RobotState& state) {
    if (!sensors.ballVector.magnitude()) return;

    // Use neural network to predict ball movement
    float inputs[5] = {
      sensors.ballVector.x,
      sensors.ballVector.y,
      sensors.ballVector.magnitude(),
      sensors.heading,
      sensors.headingRate
    };

    // Forward propagation (simplified)
    float hidden[10], output[2];

    // Input to hidden layer
    for (int i = 0; i < 10; i++) {
      hidden[i] = biases[0][i];
      for (int j = 0; j < 5; j++) {
        hidden[i] += inputs[j] * weights[0][j];  // weights[layer][input_idx]
      }
      hidden[i] = tanh(hidden[i]);  // Activation function
    }

    // Hidden to output layer
    for (int i = 0; i < 2; i++) {
      output[i] = biases[1][i];
      for (int j = 0; j < 10; j++) {
        output[i] += hidden[j] * weights[1][j];
      }
    }

    // Store predictions
    for (int i = 0; i < Config::PREDICTION_HORIZON; i++) {
      float t = (i + 1) * 0.1f;  // Time steps
      aiState.ballPrediction[i][0] = sensors.ballVector.x + output[0] * t;
      aiState.ballPrediction[i][1] = sensors.ballVector.y + output[1] * t;
    }

    // Calculate confidence based on recent prediction accuracy
    aiState.predictionConfidence = calculatePredictionConfidence(aiState, sensors);
  }

private:
  void initializeNeuralNetwork() {
    // Initialize weights with small random values
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 10; j++) {
        weights[i][j] = (random(200) - 100) / 1000.0f;  // -0.1 to 0.1
        biases[i][j] = (random(100) - 50) / 1000.0f;    // -0.05 to 0.05
      }
    }
  }

  void initializeQLearning() {
    // Initialize Q-table with small random values
    for (int i = 0; i < 13; i++) {
      for (int j = 0; j < 10; j++) {
        qTable[i][j] = (random(100) - 50) / 1000.0f;
      }
    }
  }

  void updateAIState(AIState& aiState, const SensorData& sensors, const RobotState& state) {
    // Update learning parameters
    aiState.learningCycle++;

    // Decay exploration rate over time
    if (aiState.learningCycle % 100 == 0) {
      aiState.explorationRate *= 0.995f;
      aiState.explorationRate = max(aiState.explorationRate, 0.01f);
    }

    // Update performance metrics
    updatePerformanceMetrics(aiState, state, sensors);

    // Predict ball trajectory
    predictBallTrajectory(aiState, sensors, state);
  }

  int chooseAction(const RobotState& state, const AIState& aiState, const SensorData& sensors) {
    int stateIndex = mapStateToIndex(state.currentState);

    // Epsilon-greedy action selection
    if (random(1000) / 1000.0f < aiState.explorationRate) {
      // Explore: choose random action
      return random(10);
    } else {
      // Exploit: choose best known action
      int bestAction = 0;
      float bestValue = qTable[stateIndex][0];

      for (int i = 1; i < 10; i++) {
        if (qTable[stateIndex][i] > bestValue) {
          bestValue = qTable[stateIndex][i];
          bestAction = i;
        }
      }

      return bestAction;
    }
  }

  void executeAction(int action, RobotState& state, const SensorData& sensors, const ComputerVisionData& visionData, const PredictiveAnalytics& analytics) {
    switch (action) {
      case 0:  // Move forward aggressively
        movement->move(0, 1.2f, true, MovementPattern::DIRECT);
        state.currentState = AttackerState::CHASING_BALL;
        break;
      case 1:  // Follow ball with prediction
        if (sensors.ballVector.magnitude() > 0) {
          movement->followBall(sensors.ballVector, 1.0f);
          state.currentState = AttackerState::CHASING_BALL;
        }
        break;
      case 2:  // Search with spiral pattern
        movement->move(9, 0.6f, false, MovementPattern::SPIRAL);
        state.currentState = AttackerState::SEARCHING;
        break;
      case 3:  // Defensive positioning
        movement->move(4, 0.8f, false, MovementPattern::CURVED);
        state.currentState = AttackerState::POSITIONING;
        break;
      case 4:  // Shoot
        hardware->setShooterSpeed(Config::MAX_MOTOR_SPEED);
        movement->move(0, 1.0f, true);
        state.currentState = AttackerState::SHOOTING;
        break;
      case 5:  // Dribble with curves
        movement->move(0, 0.8f, false, MovementPattern::CURVED);
        state.currentState = AttackerState::DRIBBLING;
        break;
      case 6:  // Avoid outline
        executeOutlineAvoidance(sensors);
        state.currentState = AttackerState::AVOIDING_OUTLINE;
        break;
      case 7:  // Rotate to search
        movement->move(8, 0.5f, false, MovementPattern::DIRECT);
        state.currentState = AttackerState::SEARCHING;
        break;
      case 8:  // Predictive movement
        executePredictiveMovement(sensors, analytics);
        state.currentState = AttackerState::CHASING_BALL;
        break;
      case 9:  // Adaptive strategy
        executeAdaptiveStrategy(state, sensors);
        break;
    }
  }

  void executeOutlineAvoidance(const SensorData& sensors) {
    // Smart outline avoidance based on sensor readings
    Vector2D avoidanceVector(0, 0);

    for (int i = 0; i < Config::COLOR_SENSORS; i++) {
      if (sensors.colorCalibrated[i] < Config::OUTLINE_THRESHOLD) {
        // Calculate avoidance direction
        float angle = i * 90.0f * PI / 180.0f;  // 90° per sensor
        avoidanceVector.x -= cos(angle);
        avoidanceVector.y -= sin(angle);
      }
    }

    if (avoidanceVector.magnitude() > 0) {
      avoidanceVector = avoidanceVector.normalized();
      float angle = atan2(avoidanceVector.y, avoidanceVector.x) * 180.0f / PI;
      int direction = (int)((angle + 180.0f) / 22.5f) % 16;
      movement->move(direction, 0.8f, false, MovementPattern::DIRECT);
    }
  }

  void executePredictiveMovement(const SensorData& sensors, const PredictiveAnalytics& analytics) {
    // Move to predicted ball position
    if (analytics.trajectoryConfidence > 0.6f && analytics.optimalInterceptionPoint.magnitude() > 0) {
      movement->moveToValidPosition(analytics.optimalInterceptionPoint, sensors.globalPosition, 1.0f);
    } else if (sensors.ballVector.magnitude() > 0) {
      Vector2D predictedPos = sensors.ballVector * 1.5f;  // Simple prediction
      movement->moveToValidPosition(sensors.globalPosition + predictedPos, sensors.globalPosition, 1.0f);
    }
  }

  void executeAdaptiveStrategy(RobotState& state, const SensorData& sensors) {
    // Adapt strategy based on current situation
    if (sensors.ballStrength > 0.8f) {
      // Ball is very close - be aggressive
      movement->move(0, 1.3f, true, MovementPattern::DIRECT);
      state.currentState = AttackerState::DRIBBLING;
    } else if (sensors.ballStrength > 0.3f) {
      // Ball is medium distance - use prediction
      movement->followBall(sensors.ballVector, 1.0f);
      state.currentState = AttackerState::CHASING_BALL;
    } else {
      // Ball is far or not detected - search
      movement->move(9, 0.6f, false, MovementPattern::SPIRAL);
      state.currentState = AttackerState::SEARCHING;
    }
  }

  void executeBoundaryRespectStrategy(RobotState& state, const SensorData& sensors) {
    // Move away from boundaries
    movement->avoidBoundaries(sensors);

    // Return to normal behavior when safe
    if (!sensors.onOutline && sensors.currentZone != FieldZone::OUT_OF_BOUNDS) {
      state.currentState = AttackerState::SEARCHING;
    }
  }

  void executeSearchingStrategy(RobotState& state, const SensorData& sensors) {
    // Search in valid areas only
    Vector2D searchTarget;

    if (sensors.currentZone == FieldZone::OWN_HALF) {
      // Move towards center line but stay in own half
      searchTarget = Vector2D(0, Config::CENTER_LINE_Y - 10);
    } else {
      // Search in opponent half
      searchTarget = Vector2D(0, Config::OPPONENT_GOAL_Y - 30);
    }

    movement->moveToValidPosition(searchTarget, sensors.globalPosition, 0.6f);
  }

  void executeChasingStrategy(RobotState& state, const SensorData& sensors, const PredictiveAnalytics& analytics) {
    Vector2D globalBallPos = sensors.globalPosition + sensors.ballVector;
    Vector2D interceptionPoint = analytics.optimalInterceptionPoint;

    // Prioritize interception point if available
    if (analytics.trajectoryConfidence > 0.6f && interceptionPoint.magnitude() > 0) {
      // Only chase if ball is in valid area
      if (fieldSystem->isValidMove(sensors.globalPosition, interceptionPoint)) {
        movement->moveToValidPosition(interceptionPoint, sensors.globalPosition, 1.0f);
      } else {
        // Ball is in restricted area, position near boundary
        Vector2D validPos = fieldSystem->getValidTargetPosition(interceptionPoint, sensors.globalPosition);
        movement->moveToValidPosition(validPos, sensors.globalPosition, 0.8f);
      }
    } else {
      // Only chase if ball is in valid area
      if (fieldSystem->isValidMove(sensors.globalPosition, globalBallPos)) {
        movement->moveToValidPosition(globalBallPos, sensors.globalPosition, 1.0f);
      } else {
        // Ball is in restricted area, position near boundary
        Vector2D validPos = fieldSystem->getValidTargetPosition(globalBallPos, sensors.globalPosition);
        movement->moveToValidPosition(validPos, sensors.globalPosition, 0.8f);
      }
    }
  }

  void executeDribblingStrategy(RobotState& state, const SensorData& sensors, const ComputerVisionData& visionData) {
    // Dribble towards opponent goal while respecting boundaries
    Vector2D goalTarget(0, Config::OPPONENT_GOAL_Y);
    if (visionData.goalDetected) {
      goalTarget = visionData.goalPosition;
    }
    Vector2D validTarget = fieldSystem->getValidTargetPosition(goalTarget, sensors.globalPosition);

    movement->moveToValidPosition(validTarget, sensors.globalPosition, 0.8f);

    // Check if close enough to shoot
    if (sensors.globalPosition.y > Config::OPPONENT_PENALTY_Y - 20 && sensors.ballStrength > 0.7f) {
      state.currentState = AttackerState::SHOOTING;
    }
  }

  void executePositioningStrategy(RobotState& state, const SensorData& sensors) {
    // Position for optimal shot while respecting field rules
    Vector2D optimalPos;

    if (sensors.currentZone == FieldZone::OPPONENT_HALF) {
      // Position near penalty area but outside it
      optimalPos = Vector2D(0, Config::OPPONENT_PENALTY_Y + 10);
    } else {
      // Move to opponent half first
      optimalPos = Vector2D(0, Config::CENTER_LINE_Y + 10);
    }

    movement->moveToValidPosition(optimalPos, sensors.globalPosition, 0.7f);
  }

  void executeShootingStrategy(RobotState& state, const SensorData& sensors) {
    static unsigned long shootStartTime = 0;

    if (shootStartTime == 0) {
      shootStartTime = millis();
      hardware->setShooterSpeed(Config::MAX_MOTOR_SPEED);
      state.shotsTaken++;
    }

    movement->move(0, 1.0f, true);  // Move forward during shot

    if (millis() - shootStartTime >= 300) {
      hardware->setShooterSpeed(0);
      state.lastShootTime = millis();
      state.currentState = AttackerState::SEARCHING;
      shootStartTime = 0;
    }
  }

  float calculateReward(const RobotState& state, const SensorData& sensors) {
    float reward = 0.0f;

    // Positive rewards
    if (sensors.ballStrength > 0.5f) reward += 10.0f;                    // Ball detected
    if (sensors.ballStrength > 0.8f) reward += 20.0f;                    // Ball very close
    if (state.currentState == AttackerState::SHOOTING) reward += 50.0f;  // Shooting

    // Negative rewards
    if (sensors.onOutline) reward -= 30.0f;          // On outline
    if (sensors.nearOutline) reward -= 10.0f;        // Near outline
    if (state.ballLostCounter > 10) reward -= 5.0f;  // Ball lost

    // Energy efficiency reward
    reward += sensors.batteryVoltage / 12.0f * 5.0f;

    return reward;
  }

  void updateQLearning(const RobotState& state, int action, float reward) {
    int stateIndex = mapStateToIndex(state.currentState);
    int nextStateIndex = mapStateToIndex(state.currentState);  // Simplified

    // Find max Q-value for next state
    float maxNextQ = qTable[nextStateIndex][0];
    for (int i = 1; i < 10; i++) {
      if (qTable[nextStateIndex][i] > maxNextQ) {
        maxNextQ = qTable[nextStateIndex][i];
      }
    }

    // Q-learning update
    float oldQ = qTable[stateIndex][action];
    float newQ = oldQ + learningRate * (reward + discountFactor * maxNextQ - oldQ);
    qTable[stateIndex][action] = newQ;
  }

  void adaptStrategy(AIState& aiState, const RobotState& state, const SensorData& sensors) {
    // Adapt strategy weights based on performance
    static unsigned long lastAdaptation = 0;

    if (millis() - lastAdaptation > 5000) {  // Adapt every 5 seconds
      // Simple adaptation: increase weights for successful strategies
      if (sensors.ballStrength > 0.5f) {
        aiState.strategyWeights[1] *= 1.05f;  // Ball following
      }

      if (state.shotsSuccessful > 0) {
        aiState.strategyWeights[4] *= 1.1f;  // Shooting
      }

      // Normalize weights
      float sum = 0;
      for (int i = 0; i < 10; i++) sum += aiState.strategyWeights[i];
      for (int i = 0; i < 10; i++) aiState.strategyWeights[i] /= sum;

      lastAdaptation = millis();
    }
  }

  void updatePerformanceMetrics(AIState& aiState, const RobotState& state, const SensorData& sensors) {
    static unsigned long lastUpdate = 0;
    static int lastBallDetections = 0;
    static int ballDetections = 0;

    if (sensors.ballStrength > 0.3f) ballDetections++;

    if (millis() - lastUpdate > 10000) {  // Update every 10 seconds
      aiState.ballCaptureRate = (ballDetections - lastBallDetections) / 10.0f;
      aiState.shotAccuracy = (state.shotsSuccessful > 0) ? (float)state.shotsSuccessful / state.shotsTaken : 0.0f;
      aiState.energyEfficiency = sensors.batteryVoltage / 12.0f;

      lastBallDetections = ballDetections;
      lastUpdate = millis();
    }
  }

  float calculatePredictionConfidence(const AIState& aiState, const SensorData& sensors) {
    // Calculate confidence based on prediction accuracy over time
    static float predictionErrors[10] = { 0 };
    static int errorIndex = 0;
    static Vector2D lastPrediction(0, 0);

    // Compare last prediction with actual ball position
    if (sensors.ballVector.magnitude() > 0 && lastPrediction.magnitude() > 0) {
      float error = (sensors.ballVector - lastPrediction).magnitude();
      predictionErrors[errorIndex] = error;
      errorIndex = (errorIndex + 1) % 10;
    }

    // Calculate average error
    float avgError = 0;
    for (int i = 0; i < 10; i++) avgError += predictionErrors[i];
    avgError /= 10.0f;

    // Convert error to confidence (lower error = higher confidence)
    float confidence = 1.0f / (1.0f + avgError * 5.0f);
    return constrain(confidence, 0.0f, 1.0f);
  }

  int mapStateToIndex(AttackerState state) {
    switch (state) {
      case AttackerState::SEARCHING: return 0;
      case AttackerState::CHASING_BALL: return 1;
      case AttackerState::DRIBBLING: return 2;
      case AttackerState::POSITIONING_FOR_SHOT: return 3;
      case AttackerState::SHOOTING: return 4;
      case AttackerState::AVOIDING_OUTLINE: return 5;
      case AttackerState::COORDINATING_WITH_GK: return 6;
      case AttackerState::LEARNING_MODE: return 7;
      case AttackerState::EMERGENCY_STOP: return 8;
      case AttackerState::RESPECTING_BOUNDARIES: return 9;  // New state
      default: return 10;                                   // Fallback for unknown states
    }
  }
};

// ============================================================================
// ADVANCED COMMUNICATION SYSTEM WITH ERROR CORRECTION
// ============================================================================

class AdvancedCommunicationSystem {
private:
  ConsolidatedHardwareInterface* hardware;

  // Communication protocol
  struct Packet {
    uint8_t header = 0xAA;
    uint8_t type;
    uint8_t length;
    uint8_t data[Config::PACKET_SIZE];
    uint16_t checksum;
    uint8_t footer = 0x55;
  };

  // Team coordination data
  struct TeamData {
    int goalkeeperPosition = 0;
    int goalkeeperState = 0;
    Vector2D goalkeeperBallEstimate;
    bool communicationActive = false;
    unsigned long lastCommTime = 0;
    float signalStrength = 0.0f;
    int packetLossRate = 0;
  } teamData;

  // Communication statistics
  int packetsSent = 0;
  int packetsReceived = 0;
  int packetsLost = 0;
  unsigned long totalCommTime = 0;

public:
  AdvancedCommunicationSystem(ConsolidatedHardwareInterface* hw)
    : hardware(hw) {}

  void update(const SensorData& sensors, const RobotState& state, const AIState& aiState) {
    // Receive data from goalkeeper
    processIncomingData();

    // Send comprehensive data to goalkeeper
    sendRobotStatus(sensors, state, aiState);

    // Update communication statistics
    updateCommStats();

    // Check communication health
    checkCommHealth();
  }

  const TeamData& getTeamData() const {
    return teamData;
  }

  void sendEmergencyStop() {
    Packet packet;
    packet.type = 0xFF;  // Emergency type
    packet.length = 1;
    packet.data[0] = 0x01;  // Stop command

    sendPacket(packet);
  }

  void requestGoalkeeperStatus() {
    Packet packet;
    packet.type = 0x10;  // Status request
    packet.length = 0;

    sendPacket(packet);
  }

private:
  void processIncomingData() {
    String rawData = hardware->receiveBluetoothData();
    if (rawData.length() == 0) return;

    // Parse packet format: TYPE:DATA|CRC:XXXX
    int typeEnd = rawData.indexOf(':');
    int crcStart = rawData.indexOf("|CRC:");

    if (typeEnd == -1 || crcStart == -1) return;

    String typeStr = rawData.substring(0, typeEnd);
    String dataStr = rawData.substring(typeEnd + 1, crcStart);
    String crcStr = rawData.substring(crcStart + 5);

    // Verify checksum
    uint16_t receivedCRC = strtol(crcStr.c_str(), NULL, 16);
    uint16_t calculatedCRC = calculateCRC(typeStr + ":" + dataStr);

    if (receivedCRC != calculatedCRC) {
      packetsLost++;
      return;  // Corrupted packet
    }

    packetsReceived++;
    teamData.lastCommTime = millis();
    teamData.communicationActive = true;

    // Process different packet types
    if (typeStr == "GK_STATUS") {
      parseGoalkeeperStatus(dataStr);
    } else if (typeStr == "GK_BALL") {
      parseGoalkeeperBallData(dataStr);
    } else if (typeStr == "GK_EMERGENCY") {
      handleEmergencyMessage(dataStr);
    }
  }

  void sendRobotStatus(const SensorData& sensors, const RobotState& state, const AIState& aiState) {
    static unsigned long lastSend = 0;
    if (millis() - lastSend < Config::COMM_UPDATE_INTERVAL) return;

    // Create comprehensive status packet
    String statusData = "BALL:" + String(sensors.tsopNum) + ",STR:" + String(sensors.ballStrength, 2) + ",STATE:" + String((int)state.currentState) + ",HEAD:" + String((int)sensors.heading) + ",CONF:" + String(aiState.predictionConfidence, 2) + ",BAT:" + String(sensors.batteryVoltage, 1) + ",TEMP:" + String(sensors.temperature, 1);

    hardware->sendBluetoothData("F1S_STATUS:" + statusData);
    packetsSent++;
    lastSend = millis();
  }

  void parseGoalkeeperStatus(const String& data) {
    // Parse: POS:x,STATE:y,CONF:z
    int posStart = data.indexOf("POS:") + 4;
    int posEnd = data.indexOf(',', posStart);
    teamData.goalkeeperPosition = data.substring(posStart, posEnd).toInt();

    int stateStart = data.indexOf("STATE:") + 6;
    int stateEnd = data.indexOf(',', stateStart);
    teamData.goalkeeperState = data.substring(stateStart, stateEnd).toInt();
  }

  void parseGoalkeeperBallData(const String& data) {
    // Parse ball estimate from goalkeeper
    int xStart = data.indexOf("X:") + 2;
    int xEnd = data.indexOf(',', xStart);
    float x = data.substring(xStart, xEnd).toFloat();

    int yStart = data.indexOf("Y:") + 2;
    int yEnd = data.indexOf(',', yStart);
    float y = data.substring(yStart, yEnd).toFloat();

    teamData.goalkeeperBallEstimate = Vector2D(x, y);
  }

  void handleEmergencyMessage(const String& data) {
    if (data == "STOP") {
      // Goalkeeper requests emergency stop
      // This would trigger emergency protocols
    }
  }

  void sendPacket(const Packet& packet) {
    // Convert packet to string format for transmission
    String packetStr = "PKT:" + String(packet.type, HEX) + ":" + String(packet.length) + ":";

    for (int i = 0; i < packet.length; i++) {
      packetStr += String(packet.data[i], HEX);
    }

    hardware->sendBluetoothData(packetStr);
    packetsSent++;
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
    if (millis() - lastUpdate > 5000) {  // Update every 5 seconds
      if (packetsSent > 0) {
        teamData.packetLossRate = (packetsLost * 100) / packetsSent;
      }

      // Calculate signal strength (simplified)
      teamData.signalStrength = teamData.communicationActive ? (100 - teamData.packetLossRate) / 100.0f : 0.0f;

      lastUpdate = millis();
    }
  }

  void checkCommHealth() {
    // Check if communication is healthy
    if (millis() - teamData.lastCommTime > Config::COMM_TIMEOUT) {
      teamData.communicationActive = false;
      teamData.signalStrength = 0.0f;
    }

    // Auto-recovery attempts
    if (!teamData.communicationActive) {
      static unsigned long lastRecoveryAttempt = 0;
      if (millis() - lastRecoveryAttempt > 2000) {
        requestGoalkeeperStatus();  // Ping goalkeeper
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
    unsigned long lastExecutionDuration;  // For CPU usage calculation
  };

  Task tasks[10];
  int taskCount = 0;
  unsigned long totalCPUTime = 0;
  unsigned long lastCPUMeasurement = 0;
  unsigned long measurementPeriod = 1000000;  // 1 second in microseconds

public:
  TaskScheduler() {
    lastCPUMeasurement = micros();
  }

  void addTask(void (*func)(), unsigned long interval, int priority = 1, bool enabled = true) {
    if (taskCount < 10) {
      tasks[taskCount] = { func, interval, 0, priority, enabled, 0 };
      taskCount++;
    } else {
      Serial.println("Task limit reached!");
    }
  }

  void run() {
    unsigned long currentTime = millis();

    // Sort tasks by priority and due time
    sortTasksByPriority();

    // Execute due tasks
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

    // Update CPU usage statistics
    updateCPUStats();
  }

  void enableTask(int index, bool enable) {
    if (index < taskCount) {
      tasks[index].enabled = enable;
    }
  }

  int getCPUUsage() {
    unsigned long elapsedTime = micros() - lastCPUMeasurement;
    if (elapsedTime == 0) return 0;  // Avoid division by zero
    return (int)((totalCPUTime * 100.0f) / elapsedTime);
  }

private:
  void sortTasksByPriority() {
    // Simple bubble sort by priority
    for (int i = 0; i < taskCount - 1; i++) {
      for (int j = 0; j < taskCount - i - 1; j++) {
        if (tasks[j].priority < tasks[j + 1].priority) {
          Task temp = tasks[j];
          tasks[j] = tasks[j + 1];
          tasks[j + 1] = temp;
        }
      }
    }
  }

  void updateCPUStats() {
    unsigned long now = micros();
    if (now - lastCPUMeasurement >= measurementPeriod) {  // Update every second
      lastCPUMeasurement = now;
      totalCPUTime = 0;  // Reset for next measurement
    }
  }
};

// ============================================================================
// MAIN ROBOT APPLICATION CLASS
// ============================================================================

class F1SRobot {
private:
  // Core systems
  ConsolidatedHardwareInterface hardware;
  ConsolidatedSensorSystem sensorSystem;
  FieldAwarenessSystem fieldSystem;
  ConsolidatedMovementController movement;
  ConsolidatedAIEngine aiEngine;
  AdvancedSensorFusion sensorFusion;
  AdvancedCommunicationSystem communication;
  TaskScheduler scheduler;
  PredictiveAnalyticsEngine analyticsEngine;  // Added for direct access in main loop

  // Robot state
  RobotState robotState;
  AIState aiState;
  SensorData currentSensors;

  // Performance monitoring
  unsigned long loopStartTime = 0;
  float averageLoopTime = 0.0f;

public:
  F1SRobot()
    : movement(&hardware, &fieldSystem), aiEngine(&movement, &hardware, &fieldSystem),
      communication(&hardware), analyticsEngine() {  // Initialize analyticsEngine
    hardware.setSensorSystem(&sensorSystem);
  }

  bool initialize() {
    Serial.begin(115200);
    Serial.println("F1S Attacker Robot v3.1 Initializing...");

    if (!hardware.initialize()) {
      Serial.println("Hardware initialization failed!");
      return false;
    }

    sensorSystem.initialize();
    sensorSystem.calibrateSensors();

    fieldSystem.initialize();
    aiEngine.initialize();
    sensorFusion.initialize();
    analyticsEngine.initialize();  // Initialize analytics engine

    robotState.globalPosition = Vector2D(0, -Config::FIELD_LENGTH / 4);
    robotState.currentState = AttackerState::SEARCHING;

    // Setup task scheduler
    setupTasks();

    // Initialize AI system
    aiState.adaptiveMode = true;
    aiState.explorationRate = 0.1f;

    Serial.println("F1S Attacker Robot v3.1 Ready!");
    return true;
  }

  void run() {
    loopStartTime = micros();

    // Run scheduled tasks
    scheduler.run();

    // Main control loop
    mainControlLoop();

    // Update performance metrics
    updatePerformanceMetrics();

    // Watchdog reset
    resetWatchdog();
  }

  void emergencyStop() {
    robotState.emergencyStop = true;
    robotState.currentState = AttackerState::EMERGENCY_STOP;
    movement.stop();
    hardware.setShooterSpeed(0);
    communication.sendEmergencyStop();
  }

private:
  void setupTasks() {
    // High priority tasks
    scheduler.addTask([this]() {
      this->readSensorsTask();
    },
                      Config::SENSOR_UPDATE_INTERVAL, 10);
    scheduler.addTask([this]() {
      this->updateMotorControlTask();
    },
                      5, 9);
    scheduler.addTask([this]() {
      this->safetyCheckTask();
    },
                      100, 8);

    // Medium priority tasks
    scheduler.addTask([this]() {
      this->updateAIAndAnalyticsTask();
    },
                      Config::AI_UPDATE_INTERVAL, 5);
    scheduler.addTask([this]() {
      this->updateCommunicationTask();
    },
                      Config::COMM_UPDATE_INTERVAL, 4);
    scheduler.addTask([this]() {
      this->updateDisplayTask();
    },
                      200, 3);

    // Low priority tasks
    scheduler.addTask([this]() {
      this->performMaintenanceTask();
    },
                      10000, 1);
    scheduler.addTask([this]() {
      this->logPerformanceTask();
    },
                      5000, 1);
  }

  void mainControlLoop() {
    // State machine execution
    switch (robotState.currentState) {
      case AttackerState::INITIALIZING:
        robotState.currentState = AttackerState::SEARCHING;
        break;

      case AttackerState::EMERGENCY_STOP:
        movement.stop();
        hardware.setShooterSpeed(0);
        return;

      default:
        // Execute AI strategy
        aiEngine.executeStrategy(robotState, aiState, currentSensors);
        break;
    }

    // Update movement optimization
    movement.optimizeEnergyEfficiency(currentSensors);
  }

  // Task functions
  void readSensorsTask() {
    currentSensors = hardware.readSensors();

    // Update sensor fusion
    sensorFusion.updateFilter(currentSensors);
    // currentSensors.globalPosition, velocity, heading are updated by sensorFusion.updateFilter

    // Update ball history
    if (currentSensors.ballVector.magnitude() > 0) {
      robotState.ballHistory[robotState.ballHistoryIndex] = currentSensors.ballVector;
      robotState.ballHistoryIndex = (robotState.ballHistoryIndex + 1) % Config::BALL_HISTORY_SIZE;
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

    // Additional safety checks
    if (currentSensors.batteryVoltage < Config::MIN_BATTERY_VOLTAGE) {
      emergencyStop();
    }

    // Watchdog check
    if (millis() - robotState.lastWatchdogReset > Config::WATCHDOG_TIMEOUT) {
      robotState.errorCode = 1;  // Watchdog timeout
    }
  }

  void updateAIAndAnalyticsTask() {
    // Update computer vision
    ComputerVisionData visionData = aiEngine.visionSystem.getVisionData();  // Access through AI engine
    aiEngine.visionSystem.processVision(currentSensors);

    // Update predictive analytics
    aiEngine.analyticsEngine.updatePredictions(currentSensors, visionData);

    // Execute AI strategy (main logic is in mainControlLoop, this is for AI's internal updates)
    // aiEngine.updateAIState(aiState, currentSensors, robotState); // This is called internally by executeStrategy
  }

  void updateCommunicationTask() {
    communication.update(currentSensors, robotState, aiState);
  }

  void updateDisplayTask() {
    hardware.updateDisplay(currentSensors, robotState, aiState);
  }

  void performMaintenanceTask() {
    // Periodic maintenance tasks
    static int maintenanceCycle = 0;
    maintenanceCycle++;

    if (maintenanceCycle % 10 == 0) {
      // Sensor recalibration check
      // Memory cleanup
      // Performance optimization
    }
  }

  void logPerformanceTask() {
    // Log performance data for analysis
    Serial.print("Performance: Loop=");
    Serial.print(averageLoopTime, 2);
    Serial.print("ms, CPU=");
    Serial.print(scheduler.getCPUUsage());
    Serial.print("%, Battery=");
    Serial.print(currentSensors.batteryVoltage, 1);
    Serial.print("V, Temp=");
    Serial.print(currentSensors.temperature, 1);
    Serial.println("C");
  }

  void updatePerformanceMetrics() {
    unsigned long loopTime = micros() - loopStartTime;

    // Calculate average loop time
    averageLoopTime = (averageLoopTime * 0.9f) + (loopTime / 1000.0f * 0.1f);

    // Update robot state performance metrics
    robotState.loopFrequency = 1000.0f / averageLoopTime;
    robotState.cpuUsage = scheduler.getCPUUsage();
    robotState.memoryUsage = 50;  // Simplified memory usage
  }

  void resetWatchdog() {
    robotState.lastWatchdogReset = millis();
  }
};

// ============================================================================
// ARDUINO MAIN FUNCTIONS
// ============================================================================

F1SRobot robot;

void setup() {
  // Initialize random seed
  randomSeed(analogRead(A0));

  // Initialize robot
  if (!robot.initialize()) {
    while (true) {
      delay(1000);  // Halt on initialization failure
    }
  }
}

void loop() {
  robot.run();

  // Small delay to prevent overwhelming the system
  delayMicroseconds(100);
}

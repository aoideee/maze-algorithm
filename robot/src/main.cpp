// main.cpp — Right-hand rule maze traversal
// Sensors: ultrasonic (PORT_6) = right wall, line follower (PORT_7) = front wall

#include <Arduino.h>
#include <MeMegaPi.h>


// ── Hardware ─────────────────────────────────────────────────────────────────
MeUltrasonicSensor ultraSensor(PORT_6);  // right wall distance
MeLineFollower     lineFollower(7);      // front wall detection
MeEncoderOnBoard   Encoder_1(SLOT1);    // right motor
MeEncoderOnBoard   Encoder_2(SLOT2);    // left motor
MeRGBLed           greenLedPort(PORT_8); // goal LED


// ── Speed & trim ─────────────────────────────────────────────────────────────
const double BASE_SPEED       = 50.0;
const double PIVOT_SPEED      = 65.0;
const double TURN_INNER_SPEED = 35.5;  // inner wheel during arc turns — positive so it rolls forward over tile gaps
const double SWEEP_RIGHT_SPEED = 90.0; // outer wheel for right arc turns — tune independently of BASE_SPEED
const double MOTOR_TRIM       = 1.2;   // corrects left motor hardware imbalance


// ── PID ───────────────────────────────────────────────────────────────────────
const double TARGET_DIST = 9.5;  // desired cm from right wall
const double Kp          = 1.7;
const double Ki          = 0.0;
const double Kd          = 3.0;  // higher Kd/Kp ratio prioritises heading alignment over distance snap
const double DEAD_BAND   = 1.5;  // cm — drive straight when within this range of TARGET_DIST


// ── Wall distance thresholds ─────────────────────────────────────────────────
const double WALL_MAX_DIST      = 12.5; // above this: drive straight, no PID correction
const double OPENING_THRESHOLD  = 20.0; // above this: right opening detected
const double WALL_PRESENT_DIST  = 12.0; // below this: wall is firmly present
const double SWEEP_WALL_CONFIRM = 14.0; // re-detection threshold during right-arc pivot (sensor-driven exit)


// ── Timing (ms) ──────────────────────────────────────────────────────────────
const unsigned long PIVOT_INCREMENT_MS         = 550;  // ~90° pivot-in-place at PIVOT_SPEED
const unsigned long ALIGN_DURATION             = 2000;
const unsigned long POST_TURN_DURATION         = 500;  // clears junction entrance before checking for next opening
const unsigned long SWEEP_PIVOT_TIMEOUT        = 5000; // fallback timeout for arc right turn
const unsigned long SWEEP_PIVOT_FRONT_GUARD_MS = 450;  // suppresses front-wall check at start of right pivot (must be < SWEEP_PIVOT_TIMEOUT)
const unsigned long ISLAND_OPENING_GUARD_MS    = 1500; // suppresses right-opening detection after island exit pivot
const unsigned long REVERSE_MS                 = 180;  // how long to reverse before spinning left
const double        REVERSE_SPEED              = 65.0; // both motors backward


// ── Room detection ───────────────────────────────────────────────────────────
const int ROOM_LEFT_TURN_THRESHOLD = 4; // confirmed inside island after this many consecutive left turns


// ── State machine ─────────────────────────────────────────────────────────────
enum RobotState {
    WALL_FOLLOW,
    SWEEP_RIGHT,     // two-phase right turn: arc into opening
    PIVOT_LEFT,      // sensor-confirmed left turn (incremental)
    POST_TURN_DRIVE, // brief straight drive after a turn
    ALIGNING,        // PID centering after a turn
    GOAL_FOUND       // maze solved: stopped, LED on
};
RobotState currentState = WALL_FOLLOW;

unsigned long maneuverStart = 0;


// ── Navigation flags & counters ───────────────────────────────────────────────
int  leftTurnCount      = 0;     // consecutive left pivots since last right-turn entry
bool inRoom             = false; // true once island is confirmed
bool goalSeeking        = false; // true after island exit; next opening leads to goal
bool justEnteredOpening = false; // set when SWEEP_RIGHT completes; cleared after goal check

unsigned long islandOpeningGuardEnd = 0; // timestamp when island opening guard expires
unsigned long pivotIncrementStart   = 0; // timestamp of current pivot increment start


// ── PID state ────────────────────────────────────────────────────────────────
double        pidIntegral  = 0.0;
double        pidLastError = 0.0;
unsigned long pidLastTime  = 0;

void resetPID() {
    pidIntegral  = 0.0;
    pidLastError = 0.0;
    pidLastTime  = millis();
}


// ── Encoder ISRs ──────────────────────────────────────────────────────────────
void isr_process_encoder1(void) {
    if (digitalRead(Encoder_1.getPortB()) == 0) Encoder_1.pulsePosMinus();
    else                                          Encoder_1.pulsePosPlus();
}
void isr_process_encoder2(void) {
    if (digitalRead(Encoder_2.getPortB()) == 0) Encoder_2.pulsePosMinus();
    else                                          Encoder_2.pulsePosPlus();
}
void updateEncoders() { Encoder_1.loop(); Encoder_2.loop(); }


// ── Motor control ─────────────────────────────────────────────────────────────
// MOTOR_TRIM is applied to the left motor internally to correct hardware imbalance.
void setMotors(double left, double right) {
    Encoder_2.runSpeed(-(left * MOTOR_TRIM));
    Encoder_1.runSpeed(right);
}
void stopMotors() { setMotors(0, 0); }
void spinLeft()   { setMotors(-PIVOT_SPEED, PIVOT_SPEED); }           // pivot in place
void spinRight()  { setMotors(SWEEP_RIGHT_SPEED, TURN_INNER_SPEED); } // arc right — outer=SWEEP_RIGHT_SPEED, inner rolls forward


// ── Sensor helpers ────────────────────────────────────────────────────────────
double getRightDist() {
    double d = ultraSensor.distanceCm();
    return (d <= 0 || d > 400) ? 400.0 : d;
}
bool isFrontBlocked() {
    return (lineFollower.readSensors() != 0);
}


// PID wall-follow: error > 0 = too far (steer right), error < 0 = too close (steer left)
void runPID(double rightDist, bool guardEnabled) {
    if (guardEnabled && rightDist > WALL_MAX_DIST) {
        // No wall nearby — drive straight and reset state so PID re-enters cleanly.
        pidIntegral  = 0.0;
        pidLastError = 0.0;
        pidLastTime  = millis();
        setMotors(BASE_SPEED - 2.0, BASE_SPEED + 2.0); // slight right lean toward where wall should be
        return;
    }

    unsigned long now = millis();
    double dt = constrain((now - pidLastTime) / 1000.0, 0.001, 0.5);
    pidLastTime = now;

    double error = rightDist - TARGET_DIST;

    // Dead-band: within ±DEAD_BAND of target, drive straight.
    // Prevents micro-corrections that cause oscillation and keeps motors equal
    // while the robot is already within an acceptable range.
    if (fabs(error) < DEAD_BAND) {
        pidIntegral  = 0.0;   // clear windup while coasting
        pidLastError = error;  // keep tracking so derivative is smooth on re-entry
        pidLastTime  = now;
        setMotors(BASE_SPEED, BASE_SPEED);
        return;
    }

    pidIntegral       = constrain(pidIntegral + error * dt, -30.0, 30.0);
    double derivative = (error - pidLastError) / dt;
    pidLastError      = error;

    double correction = (Kp * error) + (Ki * pidIntegral) + (Kd * derivative);
    correction = constrain(correction, -BASE_SPEED * 0.5, BASE_SPEED * 0.5);

    double L = constrain(BASE_SPEED + correction, 10.0, BASE_SPEED * 1.5);
    double R = constrain(BASE_SPEED - correction, 10.0, BASE_SPEED * 1.5);
    setMotors(L, R);
}


// ── State transition helpers ──────────────────────────────────────────────────
void startPivotLeft() {
    stopMotors();
    resetPID();
    pivotIncrementStart = millis();
    currentState        = PIVOT_LEFT;
    Serial.println("-> PIVOT_LEFT");
}

void startSweepRight() {
    resetPID();
    leftTurnCount = 0; // right-turn entry resets the left-turn counter
    currentState  = SWEEP_RIGHT;
    maneuverStart = millis();
    Serial.println("--> SWEEP_RIGHT: arc into opening");
}

void startGoalFound() {
    // Stop, light LED, halt forever.
    stopMotors();
    delay(100);
    greenLedPort.setColorAt(0, 0, 255, 0);
    greenLedPort.setColorAt(1, 0, 255, 0);
    greenLedPort.setColorAt(2, 0, 255, 0);
    greenLedPort.setColorAt(3, 0, 255, 0);
    greenLedPort.show();
    currentState = GOAL_FOUND;
    Serial.println("=== GOAL FOUND — Maze Solved! ===");
}


// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(9600);

    pinMode(greenLedPort.pin1(), OUTPUT);
    digitalWrite(greenLedPort.pin1(), LOW); // LED off until GOAL_FOUND

    attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
    attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);

    Encoder_1.setPulse(8); Encoder_1.setRatio(46.67); Encoder_1.setSpeedPid(0.18, 0, 0);
    Encoder_2.setPulse(8); Encoder_2.setRatio(46.67); Encoder_2.setSpeedPid(0.18, 0, 0);

    resetPID();
    delay(1000);
    Serial.println("=== Right-Hand Rule Active ===");
}


// ── Main loop (non-blocking state machine) ────────────────────────────────────
void loop() {
    updateEncoders();

    double rightDist = getRightDist();
    bool   frontWall = isFrontBlocked();

    // Debug — uncomment if needed:
    // Serial.print("R:"); Serial.print(rightDist);
    // Serial.print("cm F:"); Serial.println(frontWall ? "BLOCK" : "clear");

    switch (currentState) {

        // WALL_FOLLOW:
        //   Priority 1: right opening → SWEEP_RIGHT (or island exit pivot)
        //   Priority 2: front wall    → PIVOT_LEFT
        //   Priority 3: normal        → PID wall-follow
        case WALL_FOLLOW: {
            // Goal check: entered opening + hit far wall = maze solved.
            if (goalSeeking && justEnteredOpening && frontWall) {
                startGoalFound();
                break;
            }
            if (!frontWall && !goalSeeking) justEnteredOpening = false;

            if (rightDist > OPENING_THRESHOLD && !frontWall) {

                if (millis() < islandOpeningGuardEnd) {
                    // Island guard active — ignore right reading, keep driving straight.
                    runPID(rightDist, true);
                    break;
                }

                if (inRoom) {
                    // Found the exit gap; pivot left 90° then guard against the gap re-triggering.
                    Serial.println("Island: exit gap -> 90° pivot, opening guard active");
                    inRoom                = false;
                    goalSeeking           = true;
                    leftTurnCount         = 0;
                    islandOpeningGuardEnd = millis() + (2 * REVERSE_MS) + PIVOT_INCREMENT_MS + ISLAND_OPENING_GUARD_MS;
                    startPivotLeft();
                    break;
                }

                Serial.println("Right opening -> SWEEP_RIGHT");
                startSweepRight();

            } else if (frontWall) {
                justEnteredOpening = false;

                if (islandOpeningGuardEnd > 0 && millis() < islandOpeningGuardEnd + 500) {
                    // Front wall during island guard = far wall realignment pivot.
                    Serial.println("Island guard: far wall -> realign pivot");
                    islandOpeningGuardEnd = 0;
                    leftTurnCount         = 0;
                } else {
                    Serial.println("Front wall -> PIVOT_LEFT");
                }
                startPivotLeft();

            } else {
                runPID(rightDist, true);
            }
            break;
        }

        // SWEEP_RIGHT:
        //   Arcs right from the moment the opening is detected.
        //   Drives forward-right (spinRight) until the ultrasonic re-detects the new
        //   right wall (SWEEP_WALL_CONFIRM) or the fallback timeout fires.
        //   Front-wall guard suppresses false aborts for the first SWEEP_PIVOT_FRONT_GUARD_MS.
        case SWEEP_RIGHT: {
            bool pivotGuardActive = (millis() - maneuverStart < SWEEP_PIVOT_FRONT_GUARD_MS);
            if (frontWall && !pivotGuardActive) {
                Serial.println("SWEEP_RIGHT: front wall -> PIVOT_LEFT");
                startPivotLeft();
                break;
            }
            if (rightDist < SWEEP_WALL_CONFIRM) {
                Serial.println("SWEEP_RIGHT: wall confirmed -> POST_TURN_DRIVE");
                resetPID();
                justEnteredOpening = goalSeeking;
                currentState  = POST_TURN_DRIVE;
                maneuverStart = millis();
                break;
            }
            if (millis() - maneuverStart > SWEEP_PIVOT_TIMEOUT) {
                Serial.println("SWEEP_RIGHT: timeout -> POST_TURN_DRIVE");
                stopMotors(); delay(50);
                resetPID();
                currentState  = POST_TURN_DRIVE;
                maneuverStart = millis();
                break;
            }
            spinRight();
            break;
        }

        // PIVOT_LEFT:
        //   Phase 1: reverse away from the front wall.
        //   Phase 2: spin left ~90°.
        //   After each increment: front clear → POST_TURN_DRIVE, else repeat.
        case PIVOT_LEFT: {
            unsigned long elapsed = millis() - pivotIncrementStart;
            if (elapsed < REVERSE_MS) {
                setMotors(-REVERSE_SPEED, -REVERSE_SPEED); // reverse away from front wall
            } else if (elapsed < REVERSE_MS + PIVOT_INCREMENT_MS) {
                spinLeft();
            } else {
                // Brief reverse after pivot to pull wheels off tile edges before checking front sensor.
                setMotors(-REVERSE_SPEED, -REVERSE_SPEED);
                delay(REVERSE_MS);
                stopMotors();
                delay(150); // settle before reading line follower

                if (!isFrontBlocked()) {
                    leftTurnCount++;
                    Serial.print("Left pivot done. Count: "); Serial.println(leftTurnCount);

                    if (leftTurnCount >= ROOM_LEFT_TURN_THRESHOLD && !inRoom) {
                        inRoom = true;
                        Serial.println("*** Island detected (4 left turns) — seeking exit gap ***");
                    }

                    resetPID();
                    currentState  = POST_TURN_DRIVE;
                    maneuverStart = millis();
                    Serial.println("--> POST_TURN_DRIVE");
                } else {
                    Serial.println("Front blocked -> pivot another increment");
                    pivotIncrementStart = millis(); // restart full reverse+spin cycle
                }
            }
            break;
        }

        // POST_TURN_DRIVE:
        //   PID straight after any turn.
        //   Right-opening check is deferred by POST_TURN_DURATION to clear the entrance edge.
        case POST_TURN_DRIVE: {
            if (frontWall) {
                if (goalSeeking && justEnteredOpening) {
                    startGoalFound();
                } else {
                    Serial.println("POST_TURN_DRIVE: front wall -> PIVOT_LEFT");
                    startPivotLeft();
                }
                break;
            }

            if (millis() - maneuverStart < POST_TURN_DURATION) {
                runPID(rightDist, true);
            } else {
                if (rightDist > OPENING_THRESHOLD) {
                    if (inRoom) {
                        Serial.println("POST_TURN_DRIVE: right opening (inRoom) -> WALL_FOLLOW");
                        resetPID();
                        currentState = WALL_FOLLOW;
                    } else if (millis() < islandOpeningGuardEnd) {
                        runPID(rightDist, true);
                    } else {
                        Serial.println("POST_TURN_DRIVE: right opening -> SWEEP_RIGHT");
                        startSweepRight();
                    }
                } else {
                    resetPID();
                    currentState  = ALIGNING;
                    maneuverStart = millis();
                    Serial.println("--> ALIGNING");
                }
            }
            break;
        }

        // ALIGNING:
        //   PID centering for ALIGN_DURATION after a turn.
        //   Aborts early on front wall or right opening.
        case ALIGNING: {
            if (frontWall) {
                if (goalSeeking && justEnteredOpening) {
                    startGoalFound();
                } else {
                    Serial.println("ALIGNING: front wall -> PIVOT_LEFT");
                    startPivotLeft();
                }
                break;
            }
            if (rightDist > OPENING_THRESHOLD) {
                if (inRoom) {
                    Serial.println("ALIGNING: right opening (inRoom) -> WALL_FOLLOW");
                    resetPID();
                    currentState = WALL_FOLLOW;
                } else if (millis() < islandOpeningGuardEnd) {
                    runPID(rightDist, true);
                } else {
                    Serial.println("ALIGNING: right opening -> SWEEP_RIGHT");
                    startSweepRight();
                }
                break;
            }
            if (millis() - maneuverStart < ALIGN_DURATION) {
                runPID(rightDist, false);
            } else {
                resetPID();
                currentState = WALL_FOLLOW;
                Serial.println("--> WALL_FOLLOW (aligned)");
            }
            break;
        }

        // GOAL_FOUND: motors already stopped and LED already on from startGoalFound().
        case GOAL_FOUND:
            break;
    }
}
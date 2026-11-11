#include <lgr.h>

// motors
Motor leftDrive(M3, REVERSED);
Motor rightDrive(M4);
Motor intake(M6);
Motor hang(M2);
Motor lift(M5, REVERSED);

// flipper
const int flipperDown = 500;
const int flipperUp = 2050;
ServoMotor flipper(S6, flipperDown, flipperUp, flipperDown);

// claw
const int clawOpen = 1000;
const int clawHold = 1800;
const int clawClosed = 2000;
ServoMotor claw(S4, clawOpen, clawClosed, clawOpen);


Encoder liftEncoder(S5);
const float liftGearing = 12.0 / 20.0; // lift degrees per encoder degree
const float liftSmoothing = 0.3;       // 1 is no filter; smaller is smoother but slower to follow
float liftAngle = 0;                   // where the lift is: filtered, in lift degrees (kept up to date by liftPID)
const float liftDefault = 0;
const float liftLoad = 215;   // dipped down to take a hex from the flipper
const float liftNeutral = 130; // neutral position
const float liftCarry = 20;  // holding a hex
const float liftScore = 0;
float liftTarget = 0;

const float kP = 2.0;
const float kI = 0.0;
const float kD = 0.0;
const float liftITerm = 30;   // the most power the I term may add

void liftPID() {
    static float sum = 0;
    static float lastError = 0;
    static int readings = 0;  // good readings in a row since the encoder last had none
    const float dt = 0.01;    // the task runs every 10 ms

    // no encoder (the power switch is off, say): stop, rather than drive the
    // lift blind
    if (!liftEncoder.connected()) {
        lift.stop();
        readings = 0;
        return;
    }

    // the encoder has just started reading: once it has settled for a tenth
    // of a second, call where the lift is now 0 and hold it there
    if (readings < 10) {
        lift.stop();
        readings++;
        if (readings == 10) {
            liftEncoder.reset();
            liftAngle = 0;
            liftTarget = 0;
            sum = 0;
            lastError = 0;
        }
        return;
    }

    // the lift's angle, with the encoder's noise filtered out
    float measured = liftEncoder.angle() * liftGearing;
    liftAngle += (measured - liftAngle) * liftSmoothing;

    float error = liftTarget - liftAngle;
    sum += error * dt;
    if (kI > 0) {
        sum = constrain(sum, -liftITerm / kI, liftITerm / kI);
    }
    float rate = (error - lastError) / dt;
    lastError = error;

    lift.move(kP * error + kI * sum + kD * rate);
}
Task liftTask(liftPID);

const int flipperTime = 200;  // ms for the flipper to swing
const int clawTime = 250;     // ms for the claw to open or close
bool macroRunning = false;
bool hexInLift = false;

// move the lift and wait until it gets there (or 2 seconds, if it can't)
void liftTo(float angle) {
    liftTarget = angle;
    int waited = 0;
    while (abs(liftTarget - liftAngle) > 5 && waited < 2000) {
        delay(10);
        waited += 10;
    }
}

void liftMacro() {
    if (!macroRunning) {
        return;
    }

    if (!hexInLift) {
        // load
        liftTo(liftLoad);
        flipper.move(flipperUp);
        delay(flipperTime);
        claw.move(clawClosed);
        delay(clawTime);
        flipper.move(flipperDown);
        delay(flipperTime);
        liftTo(liftCarry);
        delay(500);
        claw.move(clawHold);
        hexInLift = true;
    } else {
        // score
        liftTo(liftScore);
        claw.move(clawOpen);
        delay(clawTime);
        liftTo(liftNeutral);
        hexInLift = false;
    }

    macroRunning = false;
}
Task macroTask(liftMacro);

void setup() {
    Serial.begin(115200);
}

void loop() {

    // lift macro: L2 loads a hex, L2 again scores it
    if (controller.l2Pressed()) {
        macroRunning = true;
    }

    // hang
    if (controller.r3()) {
        hang.move(100);
    } else if (controller.y()) {
        hang.move(-100);
    } else {
        hang.stop();
    }

    // intake
    if (controller.r1()) {
        intake.move(100);
    } else if (controller.r2()) {
        intake.move(-100);
    } else {
        intake.stop();
    }

    // flipper, unless the macro is using it
    if (!macroRunning) {
        if (controller.l1()) {
            flipper.move(flipperUp);
        } else {
            flipper.move(flipperDown);
        }
    }

    // arcade drive: left stick forward and back, right stick turns
    int forward = controller.leftY();
    int turn = controller.rightX();
    leftDrive.move(forward + turn);
    rightDrive.move(forward - turn);

    delay(10);
}

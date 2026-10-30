#include "psr.h"

ControllerPtr ctl;

// motors
Motor leftDrive;
Motor rightDrive;
Motor intake;
Motor hang;
Motor lift;

// servos
ServoMotor flipper;
const int flipperMin = 500;
const int flipperMax = 2050;

ServoMotor claw;
const int clawMin = 500;
const int clawMax = 1900;

// sensors
const int potPort = 15;
int potVal = 0;
int liftTarget = 1000;
int liftSpeed = 0;

void driverControls() {
    // lift
    liftTarget = 1800;


    //lift pid
    potVal = analogRead(potPort);
    liftSpeed = (liftTarget - potVal);
    lift.move(liftSpeed);
    // Serial.print("Lift Speed: ");
    // Serial.print(liftSpeed);
    // Serial.print("\tLift Pot:");
    // Serial.println(potVal);

    // hang
    if (ctl->thumbR()) {
        hang.move(100);
    } else if (ctl->y()) {
        hang.move(-100);
    } else {
        hang.move(0);
    }

    // intake
    if (ctl->r1()) {
        intake.move(100);
    } else if (ctl->r2()) {
        intake.move(-100);
    } else {
        intake.move(0);
    }

    // flipper
    if (ctl->thumbL()) {
        Serial.println("thumb");
        flipper.moveAbsolute(flipperMax);
    } else {
        flipper.moveAbsolute(flipperMin);
    }
  
    // claw
    if (ctl->l1()) {
        flipper.moveAbsolute(clawMin);
    } else {
        flipper.moveAbsolute(clawMax);
    }

    // get joysticks
    int x = ctl->axisRX();
    int y = ctl->axisY();

    // deadzone
    if (abs(x) < 100) {
        x = 0;
    }
    if (abs(y) < 100) {
        y = 0;
    }

    // limits
    if (x > 512) {
        x = 512;
    } else if (x < -512) {
        x = -512;
    }

    if (y > 512) {
        y = 512;
    } else if (y < -512) {
        y = -512;
    }

    // map to +/- 100
    x = map(x, 512, -512, 100, -100);
    y = map(y, 512, -512, 100, -100);

    leftDrive.move(y-x);
    rightDrive.move(y+x);
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);

    psrInit();
    BP32.enableBLEService(true); // set to true for BLE controllers (xbox)

    // motors
    leftDrive.init(M3);
    rightDrive.init(M4, 1); // reversed
    intake.init(M6);
    hang.init(M1);
    lift.init(M5);

    // servos
    flipper.init(S1, flipperMin);
    claw.init(S2, clawMin);

}

void loop() {
    
    delay(1); // the loop requires a delay

    // if the boot button is pressed, send the board into pairing mode
    if (digitalRead(BOOT_BTN) == LOW) {
        enterPairingMode();
    }

    // get bluetooth data from the controller
    bool dataUpdated = BP32.update();
    if (!dataUpdated || !ctl || !ctl->isConnected() || !ctl->hasData()) {
        return; // if no controller is connected and returning data, skip the driver controls
    }

    // dumpGamepad(ctl); // uncomment to print controller inputs to the serial monitor

    driverControls();
}

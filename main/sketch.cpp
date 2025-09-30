// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"

#include <Arduino.h>
#include <Bluepad32.h>
#include <M5Atom.h>
#include "AtomMotion.h"
#include <neotimer.h>
#include <Preferences.h>

//
// README FIRST, README FIRST, README FIRST
//
// Bluepad32 has a built-in interactive console.
// By default, it is enabled (hey, this is a great feature!).
// But it is incompatible with Arduino "Serial" class.
//
// Instead of using, "Serial" you can use Bluepad32 "Console" class instead.
// It is somewhat similar to Serial but not exactly the same.
//
// Should you want to still use "Serial", you have to disable the Bluepad32's console
// from "sdkconfig.defaults" with:
//    CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE=n

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

#define LONG_PRESS 1000
#define WIIMOTE_BTN_1 1
#define WIIMOTE_BTN_2 2
#define WIIMOTE_BTN_A 4
#define WIIMOTE_BTN_B 8

#define MOTOR_FULL_SPEED 127
#define STEERING_SERVO_MID_ANGLE 95

#define LED_PIN1 19
#define LED_PIN2 33

AtomMotion Atom;
Preferences preferences;


bool programButtonPressed = true;
bool motorDirection = false;
bool servoDirection = true;
bool ledState = false;
bool pirouetteTurn = false;

bool waitingButtonCommand = true;
int runningProgram = 0; // persisted in EEPROM (Flash)
boolean programStartedFlag = true;
#define RUNNING_PROGRAM_CYCLE_0 0
#define RUNNING_PROGRAM_CYCLE_1 1
#define RUNNING_PROGRAM_WIIMOTE_TWO_WHEELED 2
#define RUNNING_PROGRAM_JOYCON_STEERING 3
#define RUNNING_PROGRAMS_COUNT 4


Neotimer cycleTimer = Neotimer(2000);

int wiiMotor1 = 0;
int wiiMotor2 = 0;
int steeringServoAngle = STEERING_SERVO_MID_ANGLE;
bool wiiButtonAIsPressed = false;
bool wiiButtonBIsPressed = false;

void parametrizedCyclingProgram(int servoAngle1, int servoAngle2)
{
    if (cycleTimer.repeat())
    {
        servoDirection = !servoDirection;
        Console.println("Servo direction toggled");
        ledState =! ledState;
    }

    if (programButtonPressed)
    {
        motorDirection = !motorDirection;
        programButtonPressed = false; // clear flag
        Console.println("Motor direction toggled");
    }

    if (motorDirection)
    {
        Atom.SetMotorSpeed(1, 127);
        Atom.SetMotorSpeed(2, 127);
    }
    else
    {
        Atom.SetMotorSpeed(1, -127);
        Atom.SetMotorSpeed(2, -127);
    }
    int servoAngle = servoDirection ? servoAngle2 : servoAngle1;
    Atom.SetServoAngle(1, servoAngle);
    Atom.SetServoAngle(2, servoAngle2-servoAngle);  // even servos goes the other direction
    Atom.SetServoAngle(3, servoAngle);
    Atom.SetServoAngle(4, servoAngle2-servoAngle);  // even servos goes the other direction

    if (ledState) {
        digitalWrite(LED_PIN1, HIGH);
    } else {
        digitalWrite(LED_PIN1, LOW);
    }
    digitalWrite(LED_PIN2, HIGH);
}

void program0Loop()
{
    M5.dis.drawpix(0, 0x00ff00); // green
    parametrizedCyclingProgram(0, 180);
}

void program1Loop()
{
    M5.dis.drawpix(0, 0xffff00); // yellow
    parametrizedCyclingProgram(0, 90);
}

double recalcAxis(int32_t rawAxis) {
    return rawAxis / 512.0;
}

void program3WiimoteTwoWheeledLoop()
{
    if (programStartedFlag)
    {
        // stop motors on program started (switched from another program)
        wiiMotor1 = 0;
        wiiMotor2 = 0;
        programStartedFlag = false;
        Console.println("Wiimote program started");
    }

    M5.dis.drawpix(0, 0x0000ff); // blue

    // handle wiimote
    ControllerPtr ctrl = myControllers[0];
    if (ctrl != nullptr) {
        ControllerProperties properties = ctrl->getProperties();
        if (properties.type == CONTROLLER_TYPE_WiiController) {
            // wheel motors
            if (ctrl->dpad() & DPAD_UP) // forward
            {
                wiiMotor1 = MOTOR_FULL_SPEED;
                wiiMotor2 = MOTOR_FULL_SPEED;
                if (ctrl->buttons() & WIIMOTE_BTN_1) // forward left
                {
                    wiiMotor1 = 0;
                    Console.println("Forward left");
                }
                else if (ctrl->buttons() & WIIMOTE_BTN_2) // forward right
                {
                    wiiMotor2 = 0;
                    Console.println("Forward right");
                }
                else
                {
                    Console.println("Forward");
                }
            }
            else if (ctrl->dpad() & DPAD_DOWN) // reverse
            {
                wiiMotor1 = -MOTOR_FULL_SPEED;
                wiiMotor2 = -MOTOR_FULL_SPEED;
                if (ctrl->buttons() & WIIMOTE_BTN_1) // reverse left
                {
                    wiiMotor1 = 0;
                    Console.println("Reverse left");
                }
                else if (ctrl->buttons() & WIIMOTE_BTN_2) // reverse right
                {
                    wiiMotor2 = 0;
                    Console.println("Reverse right");
                }
                else
                {
                    Console.println("Reverse");
                }
            }
            else if (pirouetteTurn && (ctrl->buttons() & WIIMOTE_BTN_1)) // pirouette left
            {
                wiiMotor1 = -MOTOR_FULL_SPEED;
                wiiMotor2 = MOTOR_FULL_SPEED;
                Console.println("Pirouette left");
            }
            else if (pirouetteTurn && (ctrl->buttons() & WIIMOTE_BTN_2)) // pirouette right
            {
                wiiMotor1 = MOTOR_FULL_SPEED;
                wiiMotor2 = -MOTOR_FULL_SPEED;
                Console.println("Pirouette right");
            }
            else if (!pirouetteTurn && (ctrl->buttons() & WIIMOTE_BTN_1)) // forward left instead of pirouette left
            {
                wiiMotor1 = 0;
                wiiMotor2 = MOTOR_FULL_SPEED;
                Console.println("Forward left");
            }
            else if (!pirouetteTurn && (ctrl->buttons() & WIIMOTE_BTN_2)) // forward right instead of pirouette right
            {
                wiiMotor1 = MOTOR_FULL_SPEED;
                wiiMotor2 = 0;
                Console.println("Forward right");
            }
            else
            {
                wiiMotor1 = 0;
                wiiMotor2 = 0;
                Console.println("Stop");
            }
            // manipulation servo
            if ((ctrl->buttons() & WIIMOTE_BTN_A) && !wiiButtonAIsPressed)    // A just pressed
            {
                servoDirection = !servoDirection;
                Console.println("Servo direction toggled");
                wiiButtonAIsPressed = true;
            } else if (!(ctrl->buttons() & WIIMOTE_BTN_A)) {  // A released
                wiiButtonAIsPressed = false;
            }
            // external LED lights
            if ((ctrl->buttons() & WIIMOTE_BTN_B) && !wiiButtonBIsPressed)    // B just pressed
            {
                ledState = !ledState;
                Console.println("External LED toggled");
                wiiButtonBIsPressed = true;
            } else if (!(ctrl->buttons() & WIIMOTE_BTN_B)) {  // B released
                wiiButtonBIsPressed = false;
            }
        } else if (properties.type == CONTROLLER_TYPE_SwitchJoyConLeft || properties.type == CONTROLLER_TYPE_SwitchJoyConRight) {
            double recalcX = -recalcAxis(ctrl->axisX());
            double recalcY = recalcAxis(ctrl->axisY());
            float motor1 = max(min((recalcX + recalcY), 1.0), -1.0);
            float motor2 = max(min((recalcY - recalcX), 1.0), -1.0);
            Console.printf("Joycon %0.2f,%0.2f motor %0.2f,%0.2f\n", recalcX, recalcY, motor1, motor2);
            wiiMotor1 = abs(motor1) > 0.4 ? -motor1*127 : 0; // avoid low PWM values leading to no movement
            wiiMotor2 = abs(motor2) > 0.4 ? -motor2*127 : 0; // avoid low PWM values leading to no movement
        }
    } else {
        wiiMotor1 = 0;
        wiiMotor2 = 0;
        Console.println("Stop (no controller #0)");    
    }
    Atom.SetMotorSpeed(1, wiiMotor1);
    Atom.SetMotorSpeed(2, wiiMotor2);
    int servoAngle1 = servoDirection ? 90 : 0;  // 1 and 2 servos goes smaller angle
    int servoAngle2 = servoDirection ? 180 : 0;  // 3 and 4 servos goes full angle
    Atom.SetServoAngle(1, servoAngle1);
    Atom.SetServoAngle(2, 90-servoAngle1);  // even servos goes the other direction
    Atom.SetServoAngle(3, servoAngle2);
    Atom.SetServoAngle(4, 180-servoAngle2);  // even servos goes the other direction

    if (ledState) {
        digitalWrite(LED_PIN1, HIGH);
        digitalWrite(LED_PIN2, HIGH);
    } else {
        digitalWrite(LED_PIN1, LOW);
        digitalWrite(LED_PIN2, LOW);
    }    

}

/**
 * Nintendo Switch Joycon remote + two motors + servo steering
 */
void program3JoyconSteeringLoop()
{
    if (programStartedFlag)
    {
        // stop motors on program started (switched from another program)
        wiiMotor1 = 0;
        wiiMotor2 = 0;
        steeringServoAngle = STEERING_SERVO_MID_ANGLE;
        programStartedFlag = false;
        Console.println("Joycon Steering program started");
    }

    M5.dis.drawpix(0, 0xff00ff); // violet

    // handle wiimote
    ControllerPtr ctrl = myControllers[0];
    if (ctrl != nullptr) {
        ControllerProperties properties = ctrl->getProperties();
        if (properties.type == CONTROLLER_TYPE_SwitchJoyConLeft || properties.type == CONTROLLER_TYPE_SwitchJoyConRight) {
            double recalcX = recalcAxis(ctrl->axisX()); // to -1..1
            double steeringNarrowed = recalcX * 0.8;
            steeringServoAngle = (int)((steeringNarrowed + 1.0) * STEERING_SERVO_MID_ANGLE); // 72..108            
            Console.printf("Joycon X=%0.2f\tSteering servo angle %d\n", recalcX, steeringServoAngle);            
            if (ctrl->buttons() & 1) { // up
                wiiMotor1 = MOTOR_FULL_SPEED;
                wiiMotor2 = MOTOR_FULL_SPEED;
            } else if (ctrl->buttons() & 8) { // down
                wiiMotor1 = -MOTOR_FULL_SPEED;
                wiiMotor2 = -MOTOR_FULL_SPEED;
            } else {
                wiiMotor1 = 0;
                wiiMotor2 = 0;
            }
            // virtual differential gear - slow down inner wheel by a factor
            double factor = 0.2; // 0.0..1.0, 0.0 = no effect, 1.0 = full stop; actually 0.7 is full stop already
            if (recalcX > 0.2) { // turning right
                wiiMotor1 = wiiMotor1 * (1.0 - recalcX * factor); // slow down right motor
            } else if (recalcX < -0.2) { // turning left
                wiiMotor2 = wiiMotor2 * (1.0 + recalcX * factor); // slow down left motor
            }
        }
    }
    Atom.SetServoAngle(1, steeringServoAngle);
    Atom.SetServoAngle(2, steeringServoAngle);
    Atom.SetServoAngle(3, steeringServoAngle);
    Atom.SetServoAngle(4, steeringServoAngle);
    Atom.SetMotorSpeed(1, wiiMotor1);
    Atom.SetMotorSpeed(2, wiiMotor2);
}


void longPressCommand()
{
    // run next program
    int newRunningProgram = runningProgram + 1;
    runningProgram = newRunningProgram % RUNNING_PROGRAMS_COUNT;
    preferences.putInt("runningProgram", runningProgram);
    programButtonPressed = false; // always clear command button flag in case the previous program has not clear it
    programStartedFlag = true;
}

void shortPressCommand()
{
    programButtonPressed = true;
}

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Console.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Console.printf("Controller model: %s, VID=0x%04x, PID=0x%04x, BLUEPAD_TYPE=%04x\n", ctl->getModelName(), properties.vendor_id,
                           properties.product_id, properties.type);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Console.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Console.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Console.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Console.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...

    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    dumpGamepad(ctl);
}


void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                Console.printf("Unsupported controller\n");
            }
        }
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    pinMode(LED_PIN1, OUTPUT);
    pinMode(LED_PIN2, OUTPUT);
    digitalWrite(LED_PIN1, LOW);
    digitalWrite(LED_PIN2, LOW);

    preferences.begin("lego", false); 
    runningProgram = preferences.getInt("runningProgram", 0);

    M5.begin(false, false, true);  // disable Serial since blupad32 handles Serial on its own via Console
    Atom.Init(); // Motion I2C connectivity initialized here

    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Console.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But it might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "virtual device", is a mouse.
    // By default, it is disabled.
    BP32.enableVirtualDevice(false);

    // Enables the BLE Service in Bluepad32.
    // This service allows clients, like a mobile app, to setup and see the state of Bluepad32.
    // By default, it is disabled.
    BP32.enableBLEService(false);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    // This call fetches all the controllers' data.
    // Call this function in your main loop.
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();

    // handle M5Atom button
    M5.update();
    if (waitingButtonCommand && M5.Btn.pressedFor(LONG_PRESS))
    { // is pressed (not released yet) for LONG_PRESS period
        Console.println("Long press");
        longPressCommand();
        waitingButtonCommand = false; // wait for button release
    }
    else if (waitingButtonCommand && M5.Btn.wasReleased())
    { // release before LONG_PRESS period
        Console.println("Short press");
        shortPressCommand();
    }
    else if (!waitingButtonCommand && M5.Btn.wasReleased())
    {
        Console.println("Released after long press");
        waitingButtonCommand = true; // can accept next command (button press)
    }

    // run selected program loop
    switch (runningProgram)
    {
    case RUNNING_PROGRAM_CYCLE_0:
        program0Loop();
        break;

    case RUNNING_PROGRAM_CYCLE_1:
        program1Loop();
        break;

    case RUNNING_PROGRAM_WIIMOTE_TWO_WHEELED:
        program3WiimoteTwoWheeledLoop();
        break;

    case RUNNING_PROGRAM_JOYCON_STEERING:
        program3JoyconSteeringLoop();
        break;

    default:
        break;
    }

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    //     vTaskDelay(1);
    delay(1);
}

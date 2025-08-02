#include "main.h"
#include "pros/adi.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep

// motor groups
pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue); // left motor group
pros::MotorGroup rightMotors({18, 19, 20}, pros::MotorGearset::blue); // right motor group

// Inertial Sensor on port 10
pros::Imu imu(17);

// tracking wheels
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-16);

// vertical tracking wheel. 2" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.5, // 11.5 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
                              450, // drivetrain rpm is 450
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel 1, set to nullptr as we don't have one
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

pros::Motor intake2(-9, pros::v5::MotorGears::green);
pros::Motor intake(-10, pros::v5::MotorGears::green);

//pneumatics
pros::adi::Pneumatics scraper('H', false);

bool TopScore = true;
    
void setIntake(int intakePower){
    if (TopScore) {
        intake.move(intakePower);
        intake2.move(intakePower);
    }
    else {
        intake.move(intakePower);
        intake2.move(intakePower * -1);
    }
}

bool isScraperExtended = false;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Motor TopSpinner(2, pros::v5::MotorGears::green);

pros::Motor BottomSpinner(-1, pros::v5::MotorGears::green);

//color sensor
pros::Optical colorsensor(3);

bool colorsortOn = false; // flag to control color sorting

// red colorValue > 330 || colorValue < 30)
// blue colorValue > 150 && colorValue < 270

void colorSort(){
    while (colorsortOn) {
        // Get the hue value from the color sensor
        double colorValue = colorsensor.get_hue();

        // Check if the detected color is blue
        if(colorValue > 150 && colorValue < 270){
            BottomSpinner.move(0);
            TopSpinner.move(0);
            pros::delay(1000);
        }
        else {
            BottomSpinner.move(0);
            TopSpinner.move(127);
        }
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    
    pros::Task colorTask([]{
        while (true) {
            colorSort();
            pros::delay(10);
        }
    });


    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */

void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */

void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void test() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);

    chassis.moveToPose(0, 5, 0, 4000);

    chassis.waitUntilDone(); // wait until the movement is done
}

void left() {
    TopScore = true; // set the top score to true
    chassis.setPose(0, 0, 0); // set position to x:0, y:0, heading:0
    TopSpinner.move(127); // set top spinner to 127
    setIntake(127); // set intake to 127
    chassis.moveToPose(-6, 28, 0, 4000);
    chassis.waitUntilDone(); // wait until the movement is done
    chassis.moveToPose(0, 35.75, 45, 4000); 
    chassis.waitUntilDone(); // wait until the movement is done
    BottomSpinner.move(-127); // set bottom spinner to 127
    TopScore = false; // set the top score to false
    TopSpinner.move(0); // set top spinner to 0
    setIntake(90); // set intake to 127
    pros::delay(2000); // wait for 1000ms
    setIntake(0);
    BottomSpinner.move(0);
    chassis.moveToPose(-20, 10, 180, 4000); 
    chassis.waitUntilDone(); // wait until the movement is done
    scraper.extend(); // extend the scraper
    pros::delay(1000); // wait for 1000ms
    TopScore = true;
    setIntake(127); // set intake to 127
    TopSpinner.move(127); // set top spinner to 127
    chassis.moveToPose(-33, -10.3, 180, 4000); 
    chassis.waitUntilDone(); // wait until the movement is done
    pros::delay(1000); // wait for 1000ms
    chassis.moveToPose(-30, 15.75, 0, 4000);
    scraper.retract(); // retract the scraper
    chassis.waitUntilDone(); // wait until the movement is done
    BottomSpinner.move(-127); // set bottom spinner to 127
    TopSpinner.move(0); // set top spinner to 0

}

void right(){
    TopScore = true; // set the top score to true
    chassis.setPose(0, 0, 0); // set position to x:0, y:0, heading:0
    TopSpinner.move(127); // set top spinner to 127
    setIntake(127); // set intake to 127
    chassis.moveToPose(6, 28, 0, 4000);
    chassis.waitUntilDone(); // wait until the movement is done
    chassis.moveToPose(35, -5, 90, 4000); 
    chassis.waitUntilDone(); // wait until the movement is done
    chassis.moveToPose(35, 15.75, 0, 4000);
    chassis.waitUntilDone(); // wait until the movement is done
    BottomSpinner.move(-127); // set bottom spinner to 127
    TopSpinner.move(0); // set top spinner to 0
    setIntake(127);

}

void skills() {
    TopScore = true; // set the top score to true
    chassis.setPose(0, 0, 0); // set position to x:0, y:0, heading:0
    TopSpinner.move(127); // set top spinner to 127
    setIntake(127); // set intake to 127
    chassis.moveToPose(6, 20, 0, 4000);
    chassis.waitUntilDone(); // wait until the movement is done
    chassis.moveToPose(6, 28, 0, 4000);
    chassis.waitUntilDone(); // wait until the movement is done
    chassis.moveToPose(35, -5, 90, 4000); 
    chassis.waitUntilDone(); // wait until the movement is done
    chassis.moveToPose(35, 15.75, 0, 4000);
    chassis.waitUntilDone(); // wait until the movement is done
    BottomSpinner.move(-127); // set bottom spinner to 127
    TopSpinner.move(0); // set top spinner to 0
    setIntake(127);
}

void autonomous() {

    right();

    /*
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has traveled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    // Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // Turn to face a direction of 90º. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
    // also force it to turn clockwise, the long way around
    chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
    */
}

/**
 * Runs in driver control
 */
//motors

void opcontrol() {
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    intake2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    // loop forever
    while (true) {
        colorsortOn = false;
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);

        if (controller.get_digital(DIGITAL_L2)) {
            colorsortOn = false;
            BottomSpinner.move(-127);
            TopSpinner.move(0);
        }
        else {
            colorsortOn = true  ;
        }

        if (controller.get_digital_new_press(DIGITAL_DOWN)) {
            if (isScraperExtended) {
                scraper.retract(); // retract the scraper
                isScraperExtended = false; // update the state
            } else {
                scraper.extend(); // extend the scraper
                isScraperExtended = true; // update the state
            }
        }

        setIntake((controller.get_digital(DIGITAL_R1) - controller.get_digital(DIGITAL_R2)) * 127);

        if (controller.get_digital_new_press(DIGITAL_L1)) {
            if (TopScore) {
                TopScore = false; // set the top score to true
            } else {
                TopScore = true; // set the top score to true
            }
        }
    
        // delay to save resources
        pros::delay(25);
    }
}


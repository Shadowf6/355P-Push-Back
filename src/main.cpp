#include "main.h"

pros::MotorGroup leftMotors({-1, 13, -12}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({8, -9, 10}, pros::MotorGearset::blue);
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 15, lemlib::Omniwheel::NEW_325, 450, 2);

pros::Imu imu(3);
pros::Rotation verticalRotation(-7);
pros::Rotation horizontalRotation(20);
lemlib::TrackingWheel verticalWheel(&verticalRotation, lemlib::Omniwheel::NEW_275, 1);
lemlib::TrackingWheel horizontalWheel(&horizontalRotation, lemlib::Omniwheel::NEW_275, 1);
lemlib::OdomSensors odometry(&verticalWheel, nullptr, &horizontalWheel, nullptr, &imu);

lemlib::ControllerSettings lateralPID(6, 0, 5, 3, 1, 100, 3, 500, 0);
lemlib::ControllerSettings angularPID(5, 0, 20, 3, 1, 100, 3, 500, 0); 

lemlib::Chassis chassis(drivetrain, lateralPID, angularPID, odometry);

pros::Motor intake(15, pros::MotorGearset::blue);
pros::Motor score(-2, pros::MotorGearset::blue);

pros::adi::Pneumatics pivot('A', false);
pros::adi::Pneumatics tongue('B', false);
pros::adi::Pneumatics elbow('C', false);

lv_obj_t *screen;


void initialize() {                                                                                                                                                                                             
    chassis.calibrate();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    
    lv_init();
    screen = lv_screen_active();
    createDisplay(screen);

    pros::Task display([&]() {
        while (true) {
            lv_timer_handler();

            auto pose = chassis.getPose();
            updateCoords(pose.x, pose.y, pose.theta);

            pros::delay(20);
        }
    });
}

void disabled() {}

void competition_initialize() {
    horizontalRotation.reset_position();
    horizontalWheel.reset();
    verticalRotation.reset_position();
    verticalWheel.reset();
    imu.set_heading(0);
    chassis.setPose(0, 0, 0);
}

void autonomous() {
    auto in([&]() {intake.move(127); score.move(-40);});
    auto reset([&]() {intake.move(0); score.move(0);});
    auto goal([&]() {intake.move(127); score.move(127);});
    auto goon([&]() {intake.move(127); score.move(80);});
    auto drive([&](int speed, int ms) {chassis.tank(speed, speed, true); pros::delay(ms); chassis.tank(0, 0, true);});

    if (auton == 1) { // Solo AWP
        /*
        chassis.tank(50, 50, false);
        pros::delay(150);
        chassis.tank(0, 0, false);
        */
 
        // Match Load
        chassis.moveToPoint(0, 30.5, 750, {}, false);
        in();
        tongue.extend();   
        chassis.turnToHeading(90, 500, {}, false);
        chassis.moveToPoint(40, 34, 1400, {.maxSpeed=60}, false);
        
        // Long Goal
        chassis.moveToPoint(-40, 31, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50); 
        goal(); 
        pros::delay(1500);
        tongue.retract();
        in();

        // Right Corner
        drive(80, 250);
        chassis.swingToHeading(240, DriveSide::RIGHT, 500, {}, false);
        goal();
        pros::delay(250);
        chassis.moveToPoint(-29, 11, 1000, {.maxSpeed=80});
        in();
        pros::delay(500);
        tongue.extend();
        chassis.waitUntilDone();
        tongue.retract();

        // Left Corner
        chassis.turnToHeading(175, 500, {}, false);
        chassis.moveToPoint(-25, -33, 1250, {});
        pros::delay(900);
        tongue.extend();
        chassis.waitUntilDone();

        // Mid Goal
        chassis.turnToHeading(130, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(-36.6, -21, 750, {.forwards=false, .maxSpeed=60}, false);
        goal();
        pros::delay(750);
        in();

        // Long Goal
        tongue.retract();
        chassis.moveToPoint(0, -57.25, 1250, {}, false);
        chassis.turnToHeading(90, 500, {}, false);
        chassis.moveToPoint(-20, -60.25, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        goal();                                                                                                                                                                                                                                                                                                                                                                                                           
    } else if (auton == 2) { // Left
        // Corner
        in();
        chassis.moveToPoint(-12, 26, 2500, {.maxSpeed=50}); 
        pros::delay(1200);
        tongue.extend(); 
        chassis.waitUntilDone();

        // Middle Goal
        chassis.turnToHeading(-135, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(3, 37.8, 1000, {.forwards=false, .maxSpeed=60}, false);
        pivot.extend();
        goal(); 
        pros::delay(900);
        pivot.retract();
        reset();

        // Match Load
        in();
        chassis.moveToPoint(-34.5, 2, 2000, {.maxSpeed=80}, false); 
        chassis.turnToHeading(180, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(-34.75, -13, 1500, {.maxSpeed=60}, false);
        
        // Long Goal
        chassis.moveToPoint(-34.75, 30, 1500, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        goal();
    } else if (auton == 3) { // Right
        // Corner
        in();
        chassis.moveToPoint(12, 26, 2500, {.maxSpeed=50}); 
        pros::delay(1200);
        tongue.extend();
        pros::delay(500); 
        chassis.waitUntilDone();

        // Match Load
        chassis.turnToHeading(142, 1000, {.maxSpeed=80}, false);
        chassis.moveToPoint(37.5, 2, 2000, {.maxSpeed=80}, false);
        chassis.turnToHeading(180, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(37.75, -12.5, 1500, {.maxSpeed=60}, false);
        
        // Long Goal
        chassis.moveToPoint(37.75, 30, 1500, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        goal();
        pros::delay(1000);
        intake.move(-127);
        pros::delay(250);
        goal();
    } else if (auton == 4) { // Skills
        // BR ML
        in();
        elbow.extend();
        chassis.moveToPoint(0, 34.25, 1500, {.maxSpeed=80}, false);
        tongue.extend();
        chassis.turnToHeading(90, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(12.5, 37, 1500, {.maxSpeed=60}, false);
        chassis.moveToPoint(0, 37, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.moveToPoint(12.5, 37, 1500, {.maxSpeed=60}, false);

        // TR LG 
        chassis.moveToPoint(-21, 48, 2000, {.forwards=false, .maxSpeed=80}, false);
        reset();
        chassis.turnToHeading(86, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(-71, 48, 2500, {.forwards=false, .maxSpeed=80}, false);
        chassis.moveToPoint(-90, 28, 1500, {.forwards=false, .maxSpeed=80}, false);  
        chassis.turnToHeading(-90, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(-70, 31, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        goal();
        pros::delay(1000);
        intake.move(-127);
        pros::delay(250);
        goal();
        pros::delay(4000);
        in();

        // TR ML
        chassis.moveToPoint(-106.5, 30, 2500, {.maxSpeed=60}, false);
        chassis.moveToPoint(-90, 30, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.moveToPoint(-106.5, 30, 1500, {.maxSpeed=60}, false);

        // TR LG
        chassis.moveToPoint(-70, 31, 1500, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        goal();
        pros::delay(1000);
        intake.move(-127);
        pros::delay(250);
        goal();
        pros::delay(4000);
        in();

        // P
        tongue.retract();
        chassis.setPose(-73, 35, -90);
        chassis.moveToPoint(-88, 35, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(-71, 49, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.turnToHeading(-90, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(-10, 49, 2000, {.forwards=false, .maxSpeed=80}, false);
        chassis.turnToHeading(-41, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(14.5, 32, 1500, {.forwards=false, .minSpeed=60, .earlyExitRange=5});
        chassis.moveToPose(26, 11, 0, 2500, {.forwards=false, .maxSpeed=80});
        chassis.waitUntilDone();
        chassis.turnToHeading(-3, 500, {.maxSpeed=80}, false);
        drive(-100, 600);
    }
}

void opcontrol() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    bool slow = false;

    while (true) {
        chassis.tank(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { 
            // Intake
            pivot.retract();
            intake.move(127);   
            score.move(-40);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { 
            // Outtake
            intake.move(-127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { 
            // Long Goal
            pivot.retract();
            intake.move(127);
            score.move(80);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            // Middle Goal
            pivot.extend();
            intake.move(127);
            score.move(127);
        } else { 
            // Reset
            intake.move(0);
            score.move(0);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            // Elbow
            if (elbow.is_extended()) elbow.retract();
            else elbow.extend();
            pros::delay(250);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            // Tongue
            if (tongue.is_extended()) tongue.retract();
            else tongue.extend();
            pros::delay(250);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            // Slow mode
            if (!slow) {
                leftMotors.set_voltage_limit_all(6000);
                rightMotors.set_voltage_limit_all(6000);
            } else {
                leftMotors.set_voltage_limit_all(12000);
                rightMotors.set_voltage_limit_all(12000);
            }
            slow = !slow;
            pros::delay(500);
        }

        pros::delay(20);
    }
}

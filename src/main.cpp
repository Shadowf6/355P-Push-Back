#include "main.h"

pros::MotorGroup leftMotors({-8, 9, -10}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({16, -17, 18}, pros::MotorGearset::blue);
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 12, lemlib::Omniwheel::NEW_325, 450, 2);

lemlib::ControllerSettings lateralPID(6, 0, 5, 3, 1, 100, 3, 500, 0);
lemlib::ControllerSettings angularPID(5, 0, 20, 3, 1, 100, 3, 500, 0); 

pros::Imu imu(19);
pros::Rotation verticalRotation(15);
pros::Rotation horizontalRotation(-7);
lemlib::TrackingWheel verticalWheel(&verticalRotation, lemlib::Omniwheel::NEW_2, -0.5);
lemlib::TrackingWheel horizontalWheel(&horizontalRotation, lemlib::Omniwheel::NEW_275, 0.5);
lemlib::OdomSensors odometry(&verticalWheel, nullptr, &horizontalWheel, nullptr, &imu);
lemlib::Chassis chassis(drivetrain, lateralPID, angularPID, odometry);

pros::Motor intake(-20, pros::MotorGearset::blue);
pros::Motor score(-1, pros::MotorGearset::blue);

pros::adi::Pneumatics wing('C', false);
pros::adi::Pneumatics tongue('D', false);
pros::adi::Pneumatics pivot('E', false);

pros::Distance left(6);
pros::Distance right(14);
pros::Distance bottom(13);

lv_obj_t *screen;

void initialize() {                                                                                                                                                                                       
    chassis.calibrate();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    score.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    leftMotors.set_voltage_limit_all(1000000);
    rightMotors.set_voltage_limit_all(1000000);

    screen = lv_screen_active();
    createDisplay(screen);
    updateStatus(
        verticalRotation.is_installed() && horizontalRotation.is_installed(), 
        imu.is_installed(), 
        left.is_installed() && right.is_installed() && bottom.is_installed(), 
        intake.is_installed() && score.is_installed()
    );

    pros::Task display([&]() {
        while (true) {
            auto pose = chassis.getPose();
            updateCoords(pose.x, pose.y, pose.theta, 0.0f, 0.0f, 0.0f);

            pros::delay(50);
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
    auto reset([&]() {intake.move(0); score.move(0); score.tare_position();});
    auto in([&]() {intake.move(127); score.move_absolute(0.0, 600);});
    auto out([&](int ms) {intake.move(-100); pros::delay(ms); intake.move(0);});
    auto goal([&](int ms) {intake.move(127); score.move(127); pros::delay(ms); reset();});
    auto mid([&](int ms) {intake.move(127); score.move(127); pros::delay(ms); reset();});
    auto drive([&](int speed, int ms) {chassis.tank(speed, speed, true); pros::delay(ms); chassis.tank(0, 0, true);});

    if (auton == 1 && !skills) { // 7 Right
        // Corner
        in();
        chassis.moveToPoint(12, 26, 1000, {.maxSpeed=80}); 
        pros::delay(750); 
        tongue.extend(); 
        chassis.waitUntilDone();
        
        // Match Load
        chassis.turnToHeading(142, 750, {}, false);
        reset();
        tongue.retract();
        chassis.moveToPoint(36.5, 2, 1000, {.maxSpeed=100}, false);
        tongue.extend();
        chassis.turnToHeading(180, 750, {}, false);
        in();
        chassis.moveToPoint(37.25, -16, 1000, {.maxSpeed=70}, false);
        
        // Goal
        chassis.moveToPoint(37.25, 30, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        out(200);
        goal(2000);
        tongue.retract(); 
        wing.extend(); 
        pros::delay(250); 
        wing.retract();
        chassis.setPose(38, 20, wrap(imu.get_heading()));

        // Wing
        chassis.moveToPoint(38, 1, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(49, 16, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.turnToHeading(180, 750, {}, false);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.moveToPoint(49, 37, 100000, {.forwards=false, .maxSpeed=80}, false);
    } else if (auton == 2 && !skills) { // 7 Left
        // Corner
        in();
        chassis.moveToPoint(-12, 26, 1000, {.maxSpeed=80}); 
        pros::delay(750); 
        tongue.extend(); 
        chassis.waitUntilDone();
        
        // Match Load
        chassis.turnToHeading(-142, 750, {}, false);
        reset();
        chassis.moveToPoint(-36.5, 2, 1000, {.maxSpeed=100}, false);
        chassis.turnToHeading(-180, 750, {}, false);
        in();
        chassis.moveToPoint(-37, -16, 1000, {.maxSpeed=80}, false);
        
        // Goal
        chassis.moveToPoint(-37, 30, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        out(200);
        goal(2000);
        tongue.retract(); 
        wing.extend(); 
        pros::delay(250);
        wing.retract();
        chassis.setPose(-38, 20, wrap(imu.get_heading()));

        // Wing
        chassis.moveToPoint(-38, 1, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(-48, 16, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.turnToHeading(-180, 750, {}, false);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.moveToPoint(-48, 37, 100000, {.forwards=false, .maxSpeed=80}, false);
    } else if (auton == 3 && !skills) { // Left Split
        // Corner
        in();
        chassis.moveToPoint(-12, 26, 1000, {.maxSpeed=80}); 
        pros::delay(750);
        tongue.extend(); 
        chassis.waitUntilDone();

        // Middle Goal
        chassis.turnToHeading(-135, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(1.5, 39, 750, {.forwards=false, .maxSpeed=80}, false);
        pivot.extend();
        goal(250); 
        pivot.retract();
        reset();
        chassis.setPose(1.5, 39, wrap(imu.get_heading()));
        pros::delay(500);

        // Match Load
        chassis.moveToPoint(-38, 4, 2000, {.maxSpeed=100}, false); 
        chassis.turnToHeading(180, 750, {.maxSpeed=80}, false);
        in();
        chassis.moveToPoint(-38, -15, 1000, {.maxSpeed=80}, false);
        
        // Long Goal
        chassis.moveToPoint(-38, 30, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        out(100);
        goal(1200);
        tongue.retract();
        wing.extend(); 
        pros::delay(250); 
        wing.retract();
        chassis.setPose(-37, 20, wrap(imu.get_heading()));

        // Wing
        chassis.moveToPoint(-37, 1, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(-25.5, 16, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.turnToHeading(179, 750, {.maxSpeed=80}, false);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.moveToPoint(-25.5, 37, 100000, {.forwards=false, .maxSpeed=80}, false);
    } else if (auton == 4 && !skills) { // Solo AWP
        // Match Load
        chassis.moveToPoint(0, 35.75, 1000, {.maxSpeed=100}, false);
        tongue.extend();
        chassis.turnToHeading(88, 750, {.maxSpeed=80}, false);
        in();
        chassis.moveToPoint(14, 35.75, 925, {}, false);
        
        // Long Goal
        chassis.moveToPoint(-45, 35.75, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50); 
        intake.move(127);
        score.move(127);
        pros::delay(1250);
        tongue.retract();
        chassis.setPose(-45, 34, wrap(imu.get_heading()));

        // Right Corner
        chassis.moveToPoint(-33, 22, 500, {.maxSpeed=80}, false);
        chassis.turnToHeading(205, 750, {}, false);
        score.brake();
        in();
        chassis.moveToPoint(-46.5, 10, 500, {.maxSpeed=80});
        pros::delay(400);
        tongue.extend();
        chassis.waitUntilDone();

        // Left Corner
        chassis.turnToHeading(180, 500, {}, false);
        tongue.retract();
        chassis.moveToPoint(-46.5, -38, 1000, {.maxSpeed=80});
        pros::delay(850);
        tongue.extend();
        chassis.waitUntilDone();

        // Middle Goal
        chassis.turnToHeading(135, 750, {}, false);
        chassis.moveToPoint(-60, -30, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.turnToHeading(145, 750);
        pivot.extend();
        out(150);
        goal(600);
        pivot.retract();
        tongue.retract();
        in();
        
        // Match Load
        chassis.moveToPoint(-21, -63.5, 1250, {.maxSpeed=100}, false);
        tongue.extend();
        chassis.turnToHeading(90, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(10, -63.5, 1000, {.maxSpeed=70}, false);
        
        // Long Goal
        chassis.moveToPoint(-50, -64.7, 800, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        score.move(127);
    } else if (auton == 5 || skills) { // Skills
        // Bottom Left Match Load
        chassis.moveToPoint(0, 35, 1500, {.maxSpeed=80}, false);
        tongue.extend();
        chassis.turnToHeading(-85, 750, {.maxSpeed=80}, false);
        in();
        chassis.moveToPoint(-13.3, 36.5, 1000, {.maxSpeed=80}, false);
        pros::delay(1000);
        drive(-50, 100);
        pros::delay(250);
        drive(50, 500);
        pros::delay(500);

        // Left Alley
        chassis.moveToPoint(0, 36, 1000, {.forwards=false, .maxSpeed=80}, false);
        tongue.retract();
        chassis.moveToPoint(11.5, 50, 1000, {.minSpeed=80, .earlyExitRange=8});
        chassis.moveToPose(29, 50, 90, 1500); chassis.waitUntilDone();
        chassis.setPose(chassis.getPose().x, 53 - inch(left.get()), wrap(imu.get_heading()));
        chassis.moveToPoint(78, 51, 2000, {.maxSpeed=80}, false);

        // Left Goal 1
        chassis.moveToPoint(100, 36, 1500, {.maxSpeed=80}, false);
        chassis.turnToHeading(90, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(70, 36, 1000, {.forwards=false, .maxSpeed=80}, false);
        out(200);
        chassis.tank(-40, -40);
        goal(3000);
        tongue.extend();
        in();
        chassis.turnToHeading(90, 500, {}, false);
        chassis.setPose(80, 34, wrap(imu.get_heading()));
        
        // Top Left Match Load
        chassis.moveToPoint(115, 34.25, 2000, {.maxSpeed=60}, false);
        pros::delay(1000);
        drive(-50, 100);
        pros::delay(250);
        drive(50, 500);
        pros::delay(500);

        // Left Goal 2
        chassis.moveToPoint(70, 33.5, 1000, {.forwards=false, .maxSpeed=80}, false);
        out(150);
        chassis.tank(-40, -40);
        goal(3000);
        reset();
        tongue.retract();
        chassis.turnToHeading(90, 500, {}, false);
        chassis.setPose(80, 34, 90);
        pros::delay(500);

        // Top Right Match Load
        chassis.moveToPoint(95, 34, 1000, {.maxSpeed=100}, false);
        chassis.turnToHeading(0, 1000, {.maxSpeed=60}, false);
        chassis.moveToPoint(100, -64, 3500, {.forwards=false, .maxSpeed=80}, false);
        tongue.extend();
        chassis.turnToHeading(90, 750, {}, false);
        in();
        chassis.moveToPoint(125, -63.5, 1000, {.maxSpeed=60}, false);
        pros::delay(1000);
        drive(-50, 100);
        pros::delay(250);
        drive(50, 500);
        pros::delay(500);
        
        // Right Alley
        chassis.moveToPoint(110, chassis.getPose().y, 1000, {.forwards=false, .maxSpeed=80}, false);
        reset();
        pros::delay(1000);
        tongue.retract();
        chassis.moveToPoint(95, -84, 1000, {.minSpeed=80, .earlyExitRange=8});
        chassis.waitUntilDone(); tongue.retract();
        chassis.moveToPose(71, -84, -90, 1500); chassis.waitUntilDone();
        chassis.moveToPoint(22, -87, 2000, {.maxSpeed=80}, false);

        // Right Goal 1
        chassis.moveToPoint(0, -72, 1500, {.maxSpeed=80}, false);
        chassis.turnToHeading(-90, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(25, -72, 1000, {.forwards=false, .maxSpeed=80}, false);
        out(200);
        chassis.tank(-40, -40);
        goal(3000);
        tongue.extend();
        reset();
        in();
        chassis.turnToHeading(-90, 500, {}, false);
        chassis.setPose(20, -64, -90);
        
        // Bottom Right Match Load
        chassis.moveToPoint(-15, -64, 2000, {.maxSpeed=60}, false);
        pros::delay(1000);
        drive(-50, 100);
        pros::delay(250);
        drive(50, 500);
        pros::delay(500);

        // Right Goal 2
        chassis.moveToPoint(25, -63.6, 1000, {.forwards=false, .maxSpeed=80}, false);
        out(200);
        chassis.tank(-40, -40);
        goal(3000);
        tongue.retract();
        pros::delay(1000);
        chassis.turnToHeading(-90, 500, {}, false);
        chassis.setPose(0, 0, 0);
        score.move(127);

        // Park
        chassis.moveToPoint(0, 13, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(15, 31, 1000, {.minSpeed=100, .earlyExitRange=4});
        chassis.moveToPose(33, 38, 82.5, 1500); 
        chassis.waitUntilDone();
        tongue.extend();
        pros::delay(500);
        drive(127, 550);
        drive(-100, 200);
        tongue.retract();
    }
}

void opcontrol() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    leftMotors.set_voltage_limit_all(startingSpeed * 1000);
    rightMotors.set_voltage_limit_all(startingSpeed * 1000);
    
    while (true) {
        chassis.tank(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // Intake
            pivot.retract();
            intake.move(127);
            score.move_absolute(0.0, 600);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // Outtake
            intake.move(-100);
            score.move(-127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { // Long Goal
            pivot.retract();
            intake.move(127);
            score.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) { // Middle Goal 
            pivot.extend();
            intake.move(127);
            score.move(127);
        } else { // Reset
            intake.move(0);
            score.move(0);
            score.tare_position();
        }

        // Speed controls
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            startingSpeed = std::max(0, startingSpeed - 1);
            leftMotors.set_voltage_limit_all(startingSpeed * 1000);
            rightMotors.set_voltage_limit_all(startingSpeed * 1000);
            pros::delay(250);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            startingSpeed = std::min(maxSpeed, startingSpeed + 1);
            leftMotors.set_voltage_limit_all(startingSpeed * 1000);
            rightMotors.set_voltage_limit_all(startingSpeed * 1000);
            pros::delay(250);
        }

        // Elbow
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) wing.extend();
        else wing.retract();

        // Tongue
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            if (tongue.is_extended()) tongue.retract();
            else tongue.extend();
            pros::delay(250);
        }
    
        pros::delay(20);
    }
}

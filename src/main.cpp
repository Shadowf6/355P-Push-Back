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

pros::adi::Pneumatics pivot('E', false);
pros::adi::Pneumatics tongue('D', false);
pros::adi::Pneumatics wing('C', false);

pros::Distance left(6);
pros::Distance right(14);
pros::Distance bottom(13);

lv_obj_t *screen;

void initialize() {                                                                                                                                                                                       
    chassis.calibrate();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    score.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    leftMotors.set_voltage_limit_all(12000);
    rightMotors.set_voltage_limit_all(12000);

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
    auto in([&]() {intake.move(127); score.move_absolute(0.0, 600);});
    auto out([&](int ms) {intake.move(-100); pros::delay(ms); intake.move(0);});
    auto reset([&]() {intake.move(0); score.move(0); score.tare_position();});
    auto goal([&](int ms) {intake.move(127); score.move(127); pros::delay(ms); reset();});
    
    auto drive([&](int speed, int ms) {chassis.tank(speed, speed, true); pros::delay(ms); chassis.tank(0, 0, true);});
    auto toWall([&](char side, float dist, float exit, int dir, int speed) {
        if (side == 'l') {
            if (dir == 1) while (inch(left.get()) <= dist - exit) { chassis.tank(speed, speed); pros::delay(30); }
            else if (dir == -1) while (inch(left.get()) >= dist + exit) { chassis.tank(-speed, -speed); pros::delay(30); }
        } else if (side == 'r') {
            if (dir == 1) while (inch(right.get()) <= dist - exit) { chassis.tank(speed, speed); pros::delay(30); }
            else if (dir == -1) while (inch(right.get()) >= dist + exit) { chassis.tank(-speed, -speed); pros::delay(30); }
        } else if (side == 'b') {
            if (dir == 1) while (inch(bottom.get()) >= dist + exit) { chassis.tank(-speed, -speed); pros::delay(30); }
            else if (dir == -1) while (inch(bottom.get()) <= dist - exit) { chassis.tank(speed, speed); pros::delay(30); }
        }
        chassis.tank(0, 0);
    });

    if (auton == 1) { // 7 Rush (Right)
        // Corner
        in();
        chassis.moveToPoint(12, 26, 1000, {.maxSpeed=80}); 
        pros::delay(750); tongue.extend(); chassis.waitUntilDone();
        
        // Match Load
        chassis.turnToHeading(142, 750, {}, false);
        reset();
        chassis.moveToPoint(36.5, 2, 1000, {.maxSpeed=100}, false);
        chassis.turnToHeading(180, 750, {}, false);
        in();
        chassis.moveToPoint(37, -16, 1000, {.maxSpeed=80}, false);
        
        // Goal
        chassis.moveToPoint(37, 30, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        out(100);
        goal(2000);
        tongue.retract(); 
        wing.extend(); pros::delay(250); wing.retract();
        chassis.setPose(38, 20, wrap(imu.get_heading()));

        // Wing
        chassis.moveToPoint(38, 1, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(48, 16, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.turnToHeading(180, 750, {}, false);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.moveToPoint(48, 37, 100000, {.forwards=false, .maxSpeed=80}, false);
    } else if (auton == 2) { // 4 Rush (Right)
        // Corner
        in();
        chassis.moveToPoint(12, 26, 1000, {.maxSpeed=80}, false); 
        
        // Goal
        chassis.turnToHeading(142, 750, {}, false);
        reset();
        chassis.moveToPoint(37, 2, 1000, {.maxSpeed=100}, false);
        chassis.turnToHeading(180, 750, {}, false);
        in();
        chassis.moveToPoint(36.7, 30, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        goal(1250);
        wing.extend(); pros::delay(250); wing.retract();
        chassis.setPose(38, 20, wrap(imu.get_heading()));

        // Wing
        chassis.moveToPoint(38, 1, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(48, 16, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.turnToHeading(180, 750, {}, false);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.moveToPoint(48, 37, 100000, {.forwards=false, .maxSpeed=100}, false);
    } else if (auton == 3) { // 7 Rush (Left)
        // Corner
        in();
        chassis.moveToPoint(-12, 26, 1000, {.maxSpeed=80}); 
        pros::delay(750); tongue.extend(); chassis.waitUntilDone();
        
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
        out(100);
        goal(2000);
        tongue.retract(); 
        wing.extend(); pros::delay(250); wing.retract();
        chassis.setPose(-38, 20, wrap(imu.get_heading()));

        // Wing
        chassis.moveToPoint(-38, 1, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(-48, 16, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.turnToHeading(-180, 750, {}, false);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.moveToPoint(-48, 37, 100000, {.forwards=false, .maxSpeed=80}, false);
    } else if (auton == 4) { // 4 Rush (Left)
        // Corner
        in();
        chassis.moveToPoint(-12, 26, 1000, {.maxSpeed=80}, false); 
        
        // Goal
        chassis.turnToHeading(-142, 750, {}, false);
        reset();
        chassis.moveToPoint(-37, 2, 1000, {.maxSpeed=100}, false);
        chassis.turnToHeading(-180, 750, {}, false);
        in();
        chassis.moveToPoint(-36.7, 30, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        goal(1250);
        wing.extend(); pros::delay(250); wing.retract();
        chassis.setPose(-38, 20, wrap(imu.get_heading()));

        // Wing
        chassis.moveToPoint(-38, 1, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(-48, 16, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.turnToHeading(-180, 750, {}, false);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.moveToPoint(-48, 37, 100000, {.forwards=false, .maxSpeed=100}, false);
    } else if (auton == 5) { // 2+5 (Left)
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
        wing.extend(); pros::delay(250); wing.retract();
        chassis.setPose(-37, 20, wrap(imu.get_heading()));

        // Wing
        chassis.moveToPoint(-37, 1, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(-25.5, 16, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.turnToHeading(179, 750, {.maxSpeed=80}, false);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.moveToPoint(-25.5, 37, 100000, {.forwards=false, .maxSpeed=80}, false);
    } else if (auton == 6) { // Skills
        // Bottom Right Match Load
        chassis.moveToPoint(0, 35.6, 1500, {.maxSpeed=80}, false);
        tongue.extend();
        chassis.turnToHeading(90, 750, {.maxSpeed=80}, false);
        in();
        chassis.moveToPoint(13.3, 36, 1000, {.maxSpeed=80}, false);
        pros::delay(1000);
        drive(-50, 100);
        pros::delay(250);
        drive(50, 500);
        pros::delay(500);

        // Right Alley
        chassis.moveToPoint(0, 36, 1000, {.forwards=false, .maxSpeed=80}, false);
        tongue.retract();
        chassis.turnToHeading(-35, 750, {}, false);
        chassis.moveToPoint(-11.5, 43.5, 1000, {.minSpeed=80, .earlyExitRange=8});
        chassis.moveToPose(-29, 48, -90, 1500); chassis.waitUntilDone();
        chassis.moveToPoint(-78, 47, 2000, {.maxSpeed=80}, false);

        // Right Goal 1
        chassis.moveToPoint(-100, 31, 1500, {.maxSpeed=80}, false);
        chassis.turnToHeading(-90, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(-70, 33.8, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        out(200);
        goal(3000);
        tongue.extend();
        in();
        chassis.turnToHeading(-90, 500, {}, false);
        chassis.setPose(-80, 34, (float)imu.get_heading());
        
        // Top Right Match Load
        chassis.moveToPoint(-115, 32.3, 2000, {.maxSpeed=60}, false);
        pros::delay(1000);
        drive(-50, 100);
        pros::delay(250);
        drive(50, 500);
        pros::delay(500);

        // Right Goal 2
        chassis.moveToPoint(-70, 34.5, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        out(150);
        goal(3000);
        tongue.extend();
        reset();
        chassis.setPose(-80, 34, (float)imu.get_heading());
        pros::delay(500);

        // Top Left Match Load
        chassis.moveToPoint(-95, 34, 1000, {.maxSpeed=80}, false);
        chassis.turnToHeading(-180, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(-93, -67, 3500, {.maxSpeed=80}, false);
        chassis.turnToHeading(-100, 500, {.maxSpeed=80}, false);
        in();
        chassis.moveToPoint(-119, -67, 1000, {.maxSpeed=60}, false);
        pros::delay(1000);
        drive(-50, 100);
        pros::delay(250);
        drive(50, 500);
        pros::delay(500);
        
        // Left Alley
        reset();
        chassis.moveToPoint(-100, -67, 1000, {.forwards=false, .maxSpeed=80}, false);
        tongue.retract();
        chassis.turnToHeading(145, 750, {}, false);
        chassis.moveToPoint(-88.5, -76, 1000, {.minSpeed=80, .earlyExitRange=8});
        chassis.moveToPose(-71, -79.5, 90, 1500); chassis.waitUntilDone();
        chassis.moveToPoint(-22, -78, 2000, {.maxSpeed=80}, false);

        // Left Goal 1
        chassis.moveToPoint(0, -63.5, 1500, {.maxSpeed=80}, false);
        chassis.turnToHeading(90, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(-25, -64.2, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        out(200);
        goal(3000);
        tongue.extend();
        reset();
        in();
        chassis.turnToHeading(90, 500, {}, false);
        chassis.setPose(-20, -64, (float)imu.get_heading());
        
        // Bottom Left Match Load
        chassis.moveToPoint(15, -63, 2000, {.maxSpeed=60}, false);
        pros::delay(1000);
        drive(-50, 100);
        pros::delay(250);
        drive(50, 500);
        pros::delay(500);

        // Left Goal 2
        chassis.moveToPoint(-25, -63.6, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        out(200);
        goal(3000);
        tongue.retract();
        wing.extend();
        pros::delay(1000);
        chassis.setPose(0, 0, 0);

        // Park
        chassis.moveToPoint(0, 13, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(-15, 31, 1000, {.minSpeed=100, .earlyExitRange=4});
        chassis.moveToPose(-33, 36, -84, 1500); chassis.waitUntilDone();
        tongue.extend();
        pros::delay(500);
        drive(127, 550);
        drive(-100, 200);
        tongue.retract();
    } else if (auton == 7) { // Get Carried
        drive(50, 150);
    }
}

void opcontrol() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    
    while (true) {
        chassis.tank(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // Intake
            pivot.retract();
            intake.move(127);
            score.move_absolute(0.0, 600);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // Outtake
            intake.move(-100);
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

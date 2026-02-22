#include "main.h"

pros::MotorGroup leftMotors({-8, 9, -10}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({16, -17, 18}, pros::MotorGearset::blue);
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 12, lemlib::Omniwheel::NEW_325, 450, 2);

lemlib::ControllerSettings lateralPID(6, 0, 5, 3, 1, 100, 3, 500, 0);
lemlib::ControllerSettings angularPID(10, 0, 65, 3, 1, 100, 3, 500, 0); 

pros::Imu imu(2);
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
pros::adi::Pneumatics pivot('E', true);

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
            updateCoords(pose.x, pose.y, pose.theta, left.get(), right.get(), bottom.get());

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
    auto out([&](int ms) {intake.move(-80); pros::delay(ms); intake.move(0);});
    auto goal([&](int ms) {intake.move(127); score.move(127); pros::delay(ms); reset();});
    auto mid([&](int ms) {pivot.retract(); intake.move(90); score.move(127); pros::delay(ms); reset();});
    auto drive([&](int speed, int ms) {chassis.tank(speed, speed, true); pros::delay(ms); chassis.tank(0, 0, true);});

    if (auton == 1) { // 7 Right
        // Corner
        in();
        chassis.moveToPoint(12, 26, 800, {.maxSpeed=80});
        wing.extend();
        pros::delay(250);
        wing.retract();
        pros::delay(500); 
        tongue.extend(); 
        chassis.waitUntilDone();
        
        // Match Load
        chassis.turnToHeading(142, 500, {}, false);
        reset();
        tongue.retract();
        chassis.moveToPoint(39, 2, 1250, {.maxSpeed=100}, false);
        tongue.extend();
        chassis.turnToHeading(180, 500, {}, false);
        in();
        chassis.moveToPoint(40, -17, 800, {.maxSpeed=70}, false);
        
        // Goal
        chassis.moveToPoint(40, 30, 800, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        out(200);
        goal(2000);
        tongue.retract(); 
        chassis.setPose(38, 20, chassis.getPose().theta);

        // Wing
        chassis.moveToPoint(38, 5, 500, {.maxSpeed=80}, false);
        chassis.moveToPoint(48.75, 16, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.turnToHeading(180, 500, {}, false);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.moveToPoint(48.5, 37, 100000, {.forwards=false, .maxSpeed=80}, false);
    } else if (auton == 2) { // 7 Left
        // Corner
        in();
        chassis.moveToPoint(-12, 26, 800, {.maxSpeed=80});
        wing.extend();
        pros::delay(250);
        wing.retract();
        pros::delay(500); 
        tongue.extend(); 
        chassis.waitUntilDone();
        
        // Match Load
        chassis.turnToHeading(-142, 500, {}, false);
        reset();
        tongue.retract();
        chassis.moveToPoint(-38.5, 2, 1250, {.maxSpeed=100}, false);
        tongue.extend();
        chassis.turnToHeading(-180, 500, {}, false);
        in();
        chassis.moveToPoint(-38.5, -17.5, 800, {.maxSpeed=70}, false);
        
        // Goal
        chassis.moveToPoint(-39.25, 30, 800, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        out(200);
        goal(2000);
        tongue.retract(); 
        chassis.setPose(-38, 20, chassis.getPose().theta);

        // Wing
        chassis.moveToPoint(-38, 5, 500, {.maxSpeed=80}, false);

        // +10.75
        chassis.moveToPoint(-27.25, 16, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.turnToHeading(-180, 500, {}, false);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.moveToPoint(-27.25, 37, 100000, {.forwards=false, .maxSpeed=80}, false);
    } else if (auton == 3) { // 4 Right
        // Corner
        in();
        chassis.moveToPoint(12, 26, 1000, {.maxSpeed=80}); 
        wing.extend();
        pros::delay(250);
        wing.retract();
        chassis.waitUntilDone();
        
        // Goal
        chassis.turnToHeading(142, 500, {}, false);
        reset();
        chassis.moveToPoint(36.65, 7, 750, {.maxSpeed=100}, false);
        chassis.turnToHeading(180, 500, {}, false);
        in();
        chassis.moveToPoint(36.65, 30, 650, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        goal(1200);
        chassis.setPose(38, 20, chassis.getPose().theta);

        // Wing
        chassis.moveToPoint(38, 5, 500, {.maxSpeed=80}, false);
        chassis.moveToPoint(48.75, 16, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.turnToHeading(180, 500, {}, false);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.moveToPoint(48.5, 37, 100000, {.forwards=false, .maxSpeed=100}, false);
    } else if (auton == 4) { // 4 Left
        // Corner
        in();
        chassis.moveToPoint(-12, 26, 1000, {.maxSpeed=80}); 
        wing.extend();
        pros::delay(250);
        wing.retract();
        chassis.waitUntilDone();
        
        // Goal
        chassis.turnToHeading(-142, 500, {}, false);
        reset();
        chassis.moveToPoint(-36.75, 7, 750, {.maxSpeed=100}, false);
        chassis.turnToHeading(-180, 500, {}, false);
        in();
        chassis.moveToPoint(-36.75, 30, 650, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        goal(1000);
        chassis.setPose(-38, 20, chassis.getPose().theta);

        // Wing
    } else if (auton == 5) { // Solo AWP (push)
        // Match Load
        chassis.moveToPoint(0, -50, 300, {.forwards=false});
        chassis.moveToPoint(0, 45.5, 1400, {.maxSpeed=100}, false);
        tongue.extend();
        chassis.turnToHeading(80, 500, {}, false);
        in();
        chassis.moveToPoint(14, 45.5, 900, {}, false);
        
        // Long Goal
        chassis.moveToPoint(-45, 45.5, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50); 
        intake.move(127);
        score.move(127);
        pros::delay(1250);
        tongue.retract();
        chassis.setPose(-45, 34, chassis.getPose().theta);

        // Right Corner
        chassis.moveToPoint(-33, 22, 500, {.maxSpeed=80}, false);
        chassis.turnToHeading(220, 750, {}, false);
        score.brake();
        in();
        chassis.moveToPoint(-47, 10, 500, {.maxSpeed=80});
        pros::delay(400);
        tongue.extend();
        chassis.waitUntilDone();

        // Left Corner
        chassis.turnToHeading(180, 500, {}, false);
        tongue.retract();
        chassis.moveToPoint(-45, -37.5, 1000, {.maxSpeed=80});
        pros::delay(850);
        tongue.extend();
        chassis.waitUntilDone();

        // Middle Goal
        chassis.turnToHeading(135, 500, {}, false);
        chassis.moveToPoint(-58.5, -28, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.turnToHeading(145, 750);
        pivot.retract();
        out(50);
        goal(600);
        pivot.extend();
        tongue.retract();
        in();
        
        // Match Load
        chassis.moveToPoint(-21, -63, 1250, {.maxSpeed=100}, false);
        tongue.extend();
        chassis.turnToHeading(90, 400, {}, false);
        chassis.moveToPoint(10, -63, 750, {.maxSpeed=70}, false);
        
        // Long Goal
        chassis.moveToPoint(-50, -65.5, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        score.move(127);
    } else if (auton == 6) { // Solo AWP (no push)
        // Match Load
        chassis.moveToPoint(0, 45.5, 1400, {.maxSpeed=100}, false);
        tongue.extend();
        chassis.turnToHeading(80, 500, {}, false);
        in();
        chassis.moveToPoint(14, 45.5, 900, {}, false);
        
        // Long Goal
        chassis.moveToPoint(-45, 45.5, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50); 
        intake.move(127);
        score.move(127);
        pros::delay(1250);
        tongue.retract();
        chassis.setPose(-45, 34, chassis.getPose().theta);

        // Right Corner
        chassis.moveToPoint(-33, 22, 500, {.maxSpeed=80}, false);
        chassis.turnToHeading(220, 750, {}, false);
        score.brake();
        in();
        chassis.moveToPoint(-47, 10, 500, {.maxSpeed=80});
        pros::delay(400);
        tongue.extend();
        chassis.waitUntilDone();

        // Left Corner
        chassis.turnToHeading(180, 500, {}, false);
        tongue.retract();
        chassis.moveToPoint(-45, -37.5, 1000, {.maxSpeed=80});
        pros::delay(850);
        tongue.extend();
        chassis.waitUntilDone();

        // Middle Goal
        chassis.turnToHeading(135, 500, {}, false);
        chassis.moveToPoint(-58.5, -28, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.turnToHeading(145, 750);
        pivot.retract();
        out(50);
        goal(600);
        pivot.extend();
        tongue.retract();
        in();
        
        // Match Load
        chassis.moveToPoint(-21, -63, 1250, {.maxSpeed=100}, false);
        tongue.extend();
        chassis.turnToHeading(90, 400, {}, false);
        chassis.moveToPoint(10, -63, 750, {.maxSpeed=70}, false);
        
        // Long Goal
        chassis.moveToPoint(-50, -65.5, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        score.move(127);
    } else if (auton == 7) { // Skills
        // BL Corner
        in();
        chassis.moveToPoint(-12, 26, 1500, {.maxSpeed=50}, false);
        pros::delay(500); 

        // Middle Goal
        chassis.turnToHeading(-135, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(1.5, 39.5, 750, {.forwards=false, .maxSpeed=80}, false);
        mid(300);
        pros::delay(500);
        pivot.extend();
        in();

        // Left Goal 1
        chassis.moveToPoint(-39.6, 4, 1500, {.maxSpeed=100}, false);
        chassis.turnToHeading(180, 500, {}, false);
        tongue.extend();
        chassis.moveToPoint(-39.75, 30, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        goal(1000);
        chassis.turnToHeading(175, 500, {}, false);
        chassis.setPose(-38, 20, chassis.getPose().theta);

        // BL Match Load
        in();
        chassis.moveToPoint(-37.25, -15, 1500, {.maxSpeed=60}, false);
        pros::delay(1000); drive(-50, 100); pros::delay(250); drive(50, 200); pros::delay(500);

        // Left Alley
        chassis.moveToPoint(-38, 0, 1000, {.forwards=false, .maxSpeed=80}, false);
        tongue.retract();
        reset();
        chassis.moveToPoint(-53.5, 24, 1500, {.forwards=false, .maxSpeed=80}, false);
        chassis.turnToHeading(180, 500, {}, false);
        chassis.setPose(inch(right.get()), chassis.getPose().y, chassis.getPose().theta);
        chassis.moveToPoint(chassis.getPose().x, 90, 2000, {.forwards=false, .maxSpeed=80}, false);

        // Left Goal 2
        chassis.turnToHeading(-90, 500, {}, false);
        chassis.moveToPoint(14, 90, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.turnToHeading(0, 500, {}, false);
        chassis.moveToPoint(16, 75, 500, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        out(100);
        goal(3000);
        chassis.turnToHeading(2, 500, {}, false);
        chassis.setPose(20, 80, chassis.getPose().theta);

        // TL Match Load
        in();
        tongue.extend();
        chassis.moveToPoint(20.2, 115, 1500, {.maxSpeed=60}, false);
        pros::delay(1000); drive(-50, 100); pros::delay(250); drive(50, 200); pros::delay(500);

        // Left Goal 3
        chassis.moveToPoint(20.5, 75, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        out(100);
        goal(3000);
        tongue.retract();
        chassis.turnToHeading(0, 500, {}, false);
        chassis.setPose(20, 75, chassis.getPose().theta);

        // Corners
        chassis.moveToPoint(27, 85, 750, {.maxSpeed=80}, false); 
        chassis.turnToHeading(120, 750, {}, false);
        in();
        chassis.moveToPoint(48, 70, 800, {.maxSpeed=80}, false);
        chassis.turnToHeading(90, 500, {}, false);
        chassis.moveToPoint(95, 77.5, 800, {.maxSpeed=80}, false);
        
        // Middle Goal
        chassis.turnToHeading(35, 750, {}, false);
        chassis.moveToPoint(82, 65, 1000, {.forwards=false, .maxSpeed=80}, false);
        mid(1000);
        drive(50, 100);
        drive(-50, 250);

        // Right Goal 1
    } else if (auton == 8) { // Get Carried
        drive(50, 150); 
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
            pivot.extend();
            intake.move(127);
            score.move_absolute(0.0, 600);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // Outtake
            intake.move(-80);
            score.move(-127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { // Long Goal
            pivot.extend();
            intake.move(127);
            score.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) { // Middle Goal 
            pivot.retract();
            intake.move(skills ? 90 : 127);
            score.move(127);
        } else { // Reset
            intake.move(0);
            score.move(0);
            score.tare_position();
        }

        // Speed Controls
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

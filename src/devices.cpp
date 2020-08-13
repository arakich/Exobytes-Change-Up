#include "main.h"

okapi::MotorGroup leftDrive({-6,-20});
okapi::MotorGroup rightDrive({7,18});
pros::Motor leftIntake(12,pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor rightIntake(11,pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor topConveyor(15,pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor botConveyor(14,pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::ADIAnalogIn topDetector2 ('E');
pros::ADIAnalogIn topDetector ('H');
pros::ADIAnalogIn botDetector ('G');
pros::ADIAnalogIn ejectDetector ('F');

pros::ADIEncoder left('C','D',true);
pros::ADIEncoder right('A','B',true);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

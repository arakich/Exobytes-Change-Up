#include "main.h"

int intakeState = 2;
bool shooting = false;
bool ejecting = false;

bool topBall;
bool botBall;
bool topBall2;
bool ballInEjector;

void thread_sensors(void *p)
{
  while(true) {
    if(topDetector.get_value() < 2800)
      topBall = true;
    else topBall = false;
    if(topDetector2.get_value() < 2800)
      topBall2 = true;
    else topBall2 = false;
    if(botDetector.get_value() < 2800)
      botBall = true;
    else botBall = false;
    if(ejectDetector.get_value() < 2700)
      ballInEjector = true;
    else ballInEjector = false;

    pros::delay(10);
  }
}

void waitForBallToEject()
{
  while(ballInEjector) pros::delay(10);
  while(!ballInEjector) pros::delay(10);
  while(ballInEjector) pros::delay(10);
}

void waitForTopBalltoLower()
{
  while(topBall || topBall2) {
    topConveyor.move_velocity(-200);
    botConveyor.move_velocity(-200);
    pros::delay(10);
  }
}

void centerTopBall()
{
  if(topBall2 && !topBall)
    topConveyor.move_velocity(-25);
  else if(!topBall2 && topBall)
    topConveyor.move_velocity(25);
  else
    topConveyor.move_velocity(0);
}

void adjustTopBall()
{
  while(!(!topBall && topBall2)) {
    topConveyor.move_velocity(25);
    pros::delay(10);
  }
}

void thread_centerTopBall(void*p)
{
  while(true) {
    if( (topBall2 && topBall) || (!topBall2 && topBall))
      topConveyor.move_velocity(25);
    else
      topConveyor.move_velocity(0);
    pros::delay(10);
  }
}

void thread_conveyor(void* p)
{
  botConveyor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  topConveyor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  pros::Task topBall_task (thread_centerTopBall, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "For opcontrol ONLY");
  topBall_task.suspend();
  while(true) {
    if(shooting && !ejecting) {
      topConveyor.move_velocity(600);
      pros::delay(400);
      botConveyor.move_velocity(300);
      while(shooting)
        pros::delay(10);
    }
    else if(ejecting && !shooting) {
      if(topBall2 && !botBall) {
        waitForTopBalltoLower();
        topConveyor.move_velocity(-600);
        botConveyor.move_velocity(600);
        waitForBallToEject();
      }
      else if(topBall2 && botBall) {
        botConveyor.move_velocity(600);
        topBall_task.resume();
        waitForBallToEject();
        topBall_task.suspend();
      }
      else if(!topBall2) {
        topConveyor.move_velocity(-600);
        botConveyor.move_velocity(600);
      }
    }
    else {
      if(topBall || topBall2)
        centerTopBall();
      else
        topConveyor.move_velocity(200);

      if(botBall && (topBall || topBall2))
        botConveyor.move_velocity(0);
      else
        botConveyor.move_velocity(400);
    }
    pros::delay(10);
  }
}

int inward = 0;
int outward = 1;
int stop = 2;
void thread_intake(void* p)
{
  while(true) {
    switch(intakeState)
    {
      case 0:
        leftIntake.move_voltage(12000);
        rightIntake.move_voltage(12000);
        break;
      case 1:
        leftIntake.move_voltage(-12000);
        rightIntake.move_voltage(-12000);
        break;
      case 2:
        leftIntake.move_voltage(0);
        rightIntake.move_voltage(0);
        break;
    }

    pros::delay(10);
  }
}

void thread_intake_control(void* p)
{
  while(true) {
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      intakeState = inward;
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intakeState = outward;
    }
    else {
      intakeState = stop;
    }
    pros::delay(10);
  }
}

void thread_control(void* p)
{
  pros::Task sub_task (thread_intake_control, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "For opcontrol ONLY");
  while(true)
  {
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      shooting = true;
      ejecting = false;
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      shooting = false;
      ejecting = true;
    }
    else {
     shooting = false;
     ejecting = false;
   }

    pros::delay(10);
  }
}

void thread_drive(void* p)
{
  while(true) {
    leftDrive.moveVoltage(controller.get_analog(ANALOG_LEFT_Y)/127.0*12000);
    rightDrive.moveVoltage(controller.get_analog(ANALOG_RIGHT_Y)/127.0*12000);
    pros::delay(10);
  }
}

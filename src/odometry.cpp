#include "main.h"
//================ Odometry Variables ================

const double WHEEL_DIAMETER = 2.75;
const double ENCODER_WIDTH = 10.25;
const double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER*M_PI;
const int DELAY = 5;

double robotTheta = 0.0;
double robotX = 0.0;
double robotY = 0.0;

lv_obj_t * debugLabel3;
lv_obj_t * debugLabel4;
lv_obj_t * debugLabel5;
lv_obj_t * debugLabel6;

void resetOdometry()
{
  robotTheta = 0;
  robotX = 0;
  robotY = 0;
}


void thread_Odometry(void*param)
{

    robotTheta = 0.0;
    robotX = 0.0;
    robotY = 0.0;

    double dTheta = 0.0;
    double dX = 0.0;
    double dY = 0.0;

    double currentLeft = 0.0;
    double currentRight = 0.0;

    double prevLeft = 0.0;
    double prevRight = 0.0;

    double dLeftVal = 0.0;
    double dRightVal = 0.0;

    lv_obj_t * xLabel = lv_label_create(lv_scr_act(), NULL);
    lv_obj_t * yLabel = lv_label_create(lv_scr_act(), NULL);
    lv_obj_t * thetaLabel = lv_label_create(lv_scr_act(), NULL);
    lv_obj_set_pos(xLabel,255,125);
    lv_obj_set_pos(yLabel,255,145);
    lv_obj_set_pos(thetaLabel,255,165);

    left.reset();
    right.reset();
    pros::delay(50);

    while(true)
    {
        currentLeft = left.get_value()/360.0*WHEEL_CIRCUMFERENCE; //read encoders
        currentRight = right.get_value()/360.0*WHEEL_CIRCUMFERENCE;

        //currentLeft = leftDrive.getPosition()/900*WHEEL_CIRCUMFERENCE; //read encoders
        //currentRight = rightDrive.getPosition()/900*WHEEL_CIRCUMFERENCE;

        dLeftVal = (currentLeft - prevLeft);
        dRightVal = (currentRight - prevRight);

        prevLeft = currentLeft; //update prev values
        prevRight = currentRight;

        dTheta = (dLeftVal - dRightVal) / ENCODER_WIDTH; //calculate change in angle in radians
        robotTheta += dTheta;

        robotTheta = fmod(robotTheta, 2*M_PI);
        if(robotTheta < 0) robotTheta += 2*M_PI;

        //robotTheta = imu.get_heading()*M_PI/180.0;

        dX = (dLeftVal + dRightVal)/2 * sin( (robotTheta) ); //calculate change in x
        dY = (dLeftVal + dRightVal)/2 * cos( (robotTheta) ); //calculate change in y

        robotX += dX; //add to current x and y
        robotY += dY;

        std::string x = std::to_string( robotX );
        char x_array[x.length()+1];

        std::string y = std::to_string( robotY );
        char y_array[y.length()+1];

        std::string theta = std::to_string( robotTheta );
        char theta_array[theta.length() + 1];

        strcpy(x_array,x.c_str());
        strcpy(y_array,y.c_str());
        strcpy(theta_array,theta.c_str());

        lv_label_set_text(xLabel, x_array);
        lv_label_set_text(yLabel, y_array);
        lv_label_set_text(thetaLabel, theta_array);

        pros::delay(10); //reupdate every dT msec
    }
}

double calcDistance(double x1, double y1, double x2, double y2)
{
    return sqrt(pow((y1 - y2), 2) + pow((x1 - x2), 2));
}

double calcDistance(double x2, double y2)
{
  double distance = calcDistance(robotX,robotY,x2,y2);
  //if(fabs(calcAngleError(x2,y2)) > M_PI/2) distance *= -1;
  return distance;
}

double calcAngleError(double theta)
{
  theta = theta*M_PI/180.0;
  double radius = 100;
  double predictedX = radius*sin(robotTheta) + robotX;
  double predictedY = radius*cos(robotTheta) + robotY;
  double targetX = radius*sin(theta) + robotX;
  double targetY = radius*cos(theta) + robotY;
  double chord = calcDistance(predictedX, predictedY, targetX, targetY);

  double angleError = 2*asin( (chord / 2) / (100) );

  predictedX = radius*sin( fmod(angleError + robotTheta, 2*M_PI) ) + robotX;
  predictedY = radius*cos( fmod(angleError + robotTheta, 2*M_PI) ) + robotY;

  if( (predictedX < targetX + 0.1) && (predictedX > targetX - 0.1) && (predictedY < targetY + 0.1) && (predictedY > targetY - 0.1) )
      return angleError;
  else
      return angleError*-1;
}

double calcAngleError(double targetX, double targetY)
{
    double radius = calcDistance(robotX, robotY, targetX, targetY);
    double predictedX = radius*sin(robotTheta) + robotX;
    double predictedY = radius*cos(robotTheta) + robotY;
    double chord = calcDistance(predictedX, predictedY, targetX, targetY);

    double angleError = 2*asin( (chord / 2) / (radius) );

    predictedX = radius*sin( fmod(angleError + robotTheta, 2*M_PI) ) + robotX;
    predictedY = radius*cos( fmod(angleError + robotTheta, 2*M_PI) ) + robotY;

    if( (predictedX < targetX + 0.1) && (predictedX > targetX - 0.1) && (predictedY < targetY + 0.1) && (predictedY > targetY - 0.1) )
        return angleError;
    else
        return angleError*-1;
}

double calcAngleErrorReversed(double targetX, double targetY)
{
    double radius = calcDistance(robotX, robotY, targetX, targetY);
    double predictedX = radius*sin(robotTheta+M_PI) + robotX;
    double predictedY = radius*cos(robotTheta+M_PI) + robotY;
    double chord = calcDistance(predictedX, predictedY, targetX, targetY);

    double angleError = 2*asin( (chord / 2) / (radius) );

    predictedX = radius*sin( fmod(angleError + robotTheta+M_PI, 2*M_PI) ) + robotX;
    predictedY = radius*cos( fmod(angleError + robotTheta+M_PI, 2*M_PI) ) + robotY;

    if( (predictedX < targetX + 0.1) && (predictedX > targetX - 0.1) && (predictedY < targetY + 0.1) && (predictedY > targetY - 0.1) )
        return angleError;
    else
        return angleError*-1;
}

double calcAngleErrorReversed(double theta)
{
  theta = theta*M_PI/180.0;
  double radius = 100;
  double predictedX = radius*sin(robotTheta+M_PI) + robotX;
  double predictedY = radius*cos(robotTheta+M_PI) + robotY;
  double targetX = radius*sin(theta) + robotX;
  double targetY = radius*cos(theta) + robotY;
  double chord = calcDistance(predictedX, predictedY, targetX, targetY);

  double angleError = 2*asin( (chord / 2) / (100) );

  predictedX = radius*sin( fmod(angleError + robotTheta+M_PI, 2*M_PI) ) + robotX;
  predictedY = radius*cos( fmod(angleError + robotTheta+M_PI, 2*M_PI) ) + robotY;

  if( (predictedX < targetX + 0.1) && (predictedX > targetX - 0.1) && (predictedY < targetY + 0.1) && (predictedY > targetY - 0.1) )
      return angleError;
  else
      return angleError*-1;
}

double* calcLineEqn (double x1, double y1, double theta)
{
  static double lineEqn [3];
  theta = theta*M_PI/180;

  double x2 = 10*sin(theta) + x1;
  double y2 = 10*cos(theta) + y1;

  lineEqn[0] = (y2-y1)/(x2-x1);
  lineEqn[1] = y1 - lineEqn[0]*x1;
  lineEqn[2] = theta;

  return lineEqn;
}

double calcErrorToLine( double* lineEqn )
{
  double error = robotY - lineEqn[0]*robotX - lineEqn[1]; //if above line, error >0, if below line, error <0
  double tempTheta = fmod(robotTheta - lineEqn[2], 2*M_PI);
  if( (tempTheta > M_PI && error > 0) || (tempTheta < M_PI && error < 0))
    error *= -1;
  return error;
}

double* convertSlopeIntToStandard(double* lineEqn)
{
  static double standardEqn [3]; //{a,b,c}
  if(lineEqn[0] <= 0)  {
      standardEqn[0] = fabs(lineEqn[0]);
      standardEqn[1] = 1;
      standardEqn[2] = lineEqn[1];
  }
  else {
    standardEqn[0] = fabs(lineEqn[0]);
    standardEqn[1] = -1;
    standardEqn[2] = -lineEqn[1];
  }

  return standardEqn;
}

double* calcPointOfIntersection(double x1, double y1, double theta1)
{
  double* eqn1 = convertSlopeIntToStandard(calcLineEqn(robotX,robotY,robotTheta+.0000001));
  double eqn1_0 = eqn1[0];
  double eqn1_1 = eqn1[1];
  double eqn1_2 = eqn1[2];
  double* eqn2 = convertSlopeIntToStandard(calcLineEqn(x1,y1,theta1));
  double eqn2_0 = eqn2[0];
  double eqn2_1 = eqn2[1];
  double eqn2_2 = eqn2[2];
  static double intersection [2];
  intersection[0] = (eqn1_2*eqn2_1 - eqn2_2*eqn1_1) / (eqn1_0*eqn2_1 - eqn2_0*eqn1_1);
  intersection[1] = (eqn1_2 - eqn1_0*intersection[0])/eqn1_1;
  return intersection;
}

//===========================================Movement Methods====================================================================================================================

void driveVector(double currentSpeed, double angleSpeed, double maxV, bool debugOn)
{
	if(fabs(currentSpeed) > maxV)
		currentSpeed = currentSpeed/fabs(currentSpeed)*maxV;

	double maxCurrentSpeed = fabs(currentSpeed) + fabs(angleSpeed);
	double leftSpeed = currentSpeed + angleSpeed;
	double rightSpeed = currentSpeed - angleSpeed;
	double speedScale;

	if(maxCurrentSpeed > maxV) {
		speedScale = fabs(maxCurrentSpeed/maxV);
		leftSpeed /= speedScale;
		rightSpeed /= speedScale;
	}

	if(debugOn) {
		std::string debug = std::to_string(leftSpeed );
		char debug_array[debug.length()+1];
		strcpy(debug_array,debug.c_str());
		lv_label_set_text(debugLabel3, debug_array);

		std::string debug1 = std::to_string(rightSpeed );
		char debug_array1[debug1.length()+1];
		strcpy(debug_array1,debug1.c_str());
		lv_label_set_text(debugLabel4, debug_array1);

		std::string debug2 = std::to_string(currentSpeed );
		char debug_array2[debug2.length()+1];
		strcpy(debug_array2,debug2.c_str());
		lv_label_set_text(debugLabel5, debug_array2);

		std::string debug3 = std::to_string(angleSpeed );
		char debug_array3[debug3.length()+1];
		strcpy(debug_array3,debug3.c_str());
		lv_label_set_text(debugLabel6, debug_array3);
	}

	leftDrive.moveVoltage(leftSpeed);
	rightDrive.moveVoltage(rightSpeed);
}

void adaptiveDrive(double x, double y, double accel, double maxV, double distkP, double anglekP, double scalePower, int settleTime, int timeout, bool debugOn)
{
	double initX = robotX;
	double initY = robotY;
	double initTheta = robotTheta;
	double distError;
	double angleError;
	double distSpeed;
	double angleSpeed;
	double currentSpeed;
	double projection;
	double distkPScale;
	double scaledDistkP;
	double leftSpeed;
	double rightSpeed;
	double maxCurrentSpeed;
	double speedScale;
	accel *= 12000;
	//minV *= 1000;
	maxV *= 1000;
	distkP *= 1000;
	anglekP *= 1000;

	int settleTimer = 0;
	int timeoutTimer = 0;

	double settleMargin = 0.5; //inches
	double adjustMargin = 6.0;
	double minSpeedMargin = 3.0;

	lv_obj_t * debugLabel1 = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(debugLabel1, "");
	lv_obj_set_pos(debugLabel1,25,125);

	lv_obj_t * debugLabel2 = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(debugLabel2, "");
	lv_obj_set_pos(debugLabel2,25,145);

	debugLabel3 = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(debugLabel3, "");
	lv_obj_set_pos(debugLabel3,25,75);

	debugLabel4 = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(debugLabel4, "");
	lv_obj_set_pos(debugLabel4,25,95);

	debugLabel5 = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(debugLabel5, "");
	lv_obj_set_pos(debugLabel5,255,75);

	debugLabel6 = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(debugLabel6, "");
	lv_obj_set_pos(debugLabel6,255,95);

	while(settleTimer < settleTime && timeoutTimer < timeout)
	{
		distError = calcDistance(x,y);
		angleError = calcAngleError(x,y);
		projection = fabs(distError)*cos(angleError);
		if(projection < 0) projection = 0;
		distkPScale = pow(fabs(projection/distError),scalePower);
		scaledDistkP = distkP * distkPScale;

		angleSpeed = angleError*anglekP;

		if(fabs(distError) < settleMargin || (fabs(distError) < adjustMargin && fabs(angleError) > 85.0*M_PI/180.0) )
			settleTimer+=10;
		timeoutTimer+=10;

		if(fabs(distError) < minSpeedMargin) {
			angleSpeed = 0;
			distSpeed = (distError/fabs(distError))*minSpeedMargin*distkP;
		}
		else if(fabs(distError) < adjustMargin) {
			angleSpeed = 0;
			distSpeed = distError*distkP;
		}
		else
			distSpeed = distError*scaledDistkP;

		currentSpeed = distSpeed;

		driveVector(currentSpeed,angleSpeed,maxV,debugOn);
		pros::delay(10);
		if(debugOn) {
			std::string debug = std::to_string(distError );
			char debug_array[debug.length()+1];
			strcpy(debug_array,debug.c_str());
			lv_label_set_text(debugLabel1, debug_array);

			std::string debug1 = std::to_string(angleError );
			char debug_array1[debug1.length()+1];
			strcpy(debug_array1,debug1.c_str());
			lv_label_set_text(debugLabel2, debug_array1);
		}
	}
	rightDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	leftDrive.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	rightDrive.moveVelocity(0);
  leftDrive.moveVelocity(0);

	if(debugOn) {
		lv_label_set_text(debugLabel1, "finished");
		lv_label_set_text(debugLabel2, "finished");
		pros::delay(1000);
		lv_obj_del(debugLabel1);
		lv_obj_del(debugLabel2);
		lv_label_set_text(debugLabel3, "");
		lv_label_set_text(debugLabel4, "");
		lv_label_set_text(debugLabel5, "");
		lv_label_set_text(debugLabel6, "");
	}
}

void adaptiveDrive(double x, double y)
{
	adaptiveDrive(x,y,0,10,0.65,4,1.0,250,10000,true);
}

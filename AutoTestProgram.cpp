#include <Joystick.h>
#include <SampleRobot.h>
#include <Talon.h>
#include "ctre/Phoenix.h"
#include <Timer.h>
#include <AHRS.h>
#include <math.h>

class Robot: public frc::SampleRobot {

bool tractionDown = false;
bool gyroenabled = true;
double PI = 3.14159;
bool IsMechanum = true;
float Sminimum = .2;
float TurnMax = .5;
float TurnMin = .27;
bool isAutonomoose = true;


public:
	float ABS(float input) {
		if (input<0) {
			input = input * -1;
		}
		return input;
	}

	void smtNUM(std::string text, double num){
		SmartDashboard::SmartDashboard::PutNumber(text, num);
	}
	void smtSTR(std::string text){
		SmartDashboard::SmartDashboard::PutString(text, "hit");
	}
	void smtBOOL(std::string text, bool blean){
		SmartDashboard::SmartDashboard::PutBoolean(text, blean);
	}
	void tankDrive(float RY, float LY){
		m_lf.Set(-LY);
		m_lr.Set(-LY);
		m_rf.Set(RY);
		m_rr.Set(RY);
	}
	void CanMechanum(float RX, float RY, float LX, float robotangle){

		float mag = sqrt((RY*RY)+(RX*RX));
		float joyangle = 0;
		if (gyroenabled) {
			robotangle = (((PI)/180)*robotangle);
			while ((robotangle >= (2*PI)) or (robotangle < 0)){
				if (robotangle >= (2*PI)){
					robotangle = robotangle - (2*PI);
				}

				else if (robotangle < 0){
					robotangle = robotangle + (2*PI);
				}
			}
			float dif = 0;
			if ((RX>=0) && (RY>0)) { // Top Right
				joyangle = atan(RX/RY) + 0;
				dif = joyangle - robotangle;
			}
			else if ((RX>0) && (RY<=0)) { // Bottom Right
				joyangle = (atan((-RY)/RX)) + ((PI)/2);
				dif = joyangle - robotangle;
			}
			else if ((RX<=0) && (RY<0)) { // Bottom Left
				joyangle = atan((-RX)/(-RY)) + (PI);
				dif = joyangle - robotangle;
			}
			else if ((RX<0) && (RY>=0)) { // Top Left
				joyangle = atan(RY/(-RX)) + ((3*PI)/2);
				dif = joyangle - robotangle;
			}
			RY = mag * cos(dif);
			RX = mag * sin(dif);
		}

		mag = sqrt((RY*RY)+(RX*RX));

		float lfVal = RY + LX + RX;
		float lrVal = RY + LX - RX;
		float rfVal = -RY + LX + RX;
		float rrVal = -RY + LX - RX;

		float maxval = lfVal;
		if (lrVal>maxval) {
			maxval = lrVal;
		}
		if (rfVal>maxval) {
			maxval = rfVal;
		}
		if (rrVal>maxval) {
			maxval = rrVal;
		}

		if (ABS(maxval) > 1){
			lfVal = ((lfVal / ABS(maxval))*(mag/sqrt(2)));
			lrVal = ((lrVal / ABS(maxval))*mag/sqrt(2));
			rfVal = ((rfVal / ABS(maxval))*mag/sqrt(2));
			rrVal = ((rrVal / ABS(maxval))*mag/sqrt(2));
		}

		m_lf.Set(lfVal);
		m_lr.Set(lrVal);
		m_rf.Set(rfVal);
		m_rr.Set(rrVal);
	}
	void move(float XDisp, float YDisp, float mag = .95){
		smtNUM("moveInitializeNUM", MotionTracker.GetDisplacementX());
		double initialXDisp = MotionTracker.GetDisplacementX();
		double initialYDisp = MotionTracker.GetDisplacementY();

		while(isAutonomoose){
			double speedX = ((((XDisp - (MotionTracker.GetDisplacementX()-initialXDisp))/(1))/(1/(mag - Sminimum))));
			//^^as the x value gets closer to where it should be, it slows down

			if (speedX<0) {
				speedX -= Sminimum; //ensures the speed is above a minimum value
			}
			else {
				speedX += Sminimum;
			}

			double speedY = ((((YDisp - (MotionTracker.GetDisplacementY()-initialYDisp))/(1))/(1/(mag - Sminimum))));
			//^^as the y value gets closer to where it should be, it slows down
			if (speedY<0) {
				speedY -= Sminimum;
			}
			else {
				speedY += Sminimum;
			}
			smtNUM("speedY", speedY);
			smtNUM("Y relative dis",MotionTracker.GetDisplacementY()-initialYDisp );
			smtNUM("xdis", MotionTracker.GetDisplacementX());
			smtNUM("ydis", MotionTracker.GetDisplacementY());
			if (XDisp < 0){
				if (MotionTracker.GetDisplacementX()-initialXDisp < XDisp){
					speedX = 0;
				}

			}
			else {
				if (MotionTracker.GetDisplacementX()-initialXDisp >= XDisp){
					speedX = 0;
				}

			}
			if (YDisp < 0){
				if (MotionTracker.GetDisplacementY()-initialYDisp < YDisp){
					speedY = 0;
				}
			}
			else {
				if (MotionTracker.GetDisplacementY()-initialYDisp >= YDisp){
					speedY = 0;
				}
			}
			if (speedX == 0 && speedY == 0){
				break;
			}




			CanMechanum(speedX, speedY, 0, MotionTracker.GetAngle());
			if (!rstick.GetRawButton(3)){
				CanMechanum(0,0,0,0);
				break;
			}
		}
		smtSTR("HIT");
	}
	float TurnTo(int theta) {
		while (theta >= 360) {
			theta -= 360;
		}
		while (theta < 0) {
			theta += 360;
		}
		float robotangle = MotionTracker.GetAngle();
		while (robotangle >= 360) {
			robotangle -= 360;
		}
		while (robotangle < 0) {
			robotangle += 360;
		}
		float dif = theta - robotangle; // left is negative

		if (ABS(dif)<(3)) {
			return 0;
		}

		if (dif<=-180) {
			dif += 360;
		}
		if (dif>180) {
			dif -= 360;
		}

		float speed = ((dif/180)*(TurnMax-TurnMin));

		if (speed<0) {
			speed -= TurnMin;
		}
		else if (speed>0) {
			speed += TurnMin;
		}

		return speed;

	}

	void OperatorControl() {
		bool can= true;

		//while(MotionTracker.IsCalibrating()){}
		while (IsOperatorControl() && IsEnabled()) {
			if (rstick.GetRawButton(4)){
				MotionTracker.Reset();
			}
			SmartDashboard::SmartDashboard::PutNumber("disX", MotionTracker.GetDisplacementX());
			smtNUM("disY", MotionTracker.GetDisplacementY());
			if (rstick.GetRawButton(2)){
				IsMechanum = false;
			}
			else if (lstick.GetRawButton(2)) {
				IsMechanum = true;
			}
//			if (IsMechanum){
//				if (rstick.GetRawButton(1) && lstick.GetRawButton(1)){
//					tankDrive(rstick.GetY(),lstick.GetY());
//					butterflySol.Set(frc::DoubleSolenoid::kForward);
//				}
//				else {
//					butterflySol.Set(frc::DoubleSolenoid::kReverse);
//					CanMechanum(rstick.GetX(), -rstick.GetY(), (lstick.GetX()/2), MotionTracker.GetAngle());
//				}
//			}
//			else if (!IsMechanum){
//				tankDrive(rstick.GetY(),lstick.GetY());
//				butterflySol.Set(frc::DoubleSolenoid::kForward);
//			}
			smtBOOL("can", can);
			smtNUM("Z", rstick.GetZ());
			smtNUM("lZ", lstick.GetZ());

			if (rstick.GetRawButton(3)&&can){
				move(0, rstick.GetZ(), lstick.GetZ());
				can= false;
			}
			else if (lstick.GetRawButton(1)&&rstick.GetRawButton(1)){
				tankDrive(rstick.GetY(), lstick.GetY());
			}
			else {
				CanMechanum(rstick.GetX(), rstick.GetY(), (lstick.GetX()/2), MotionTracker.GetAngle());
			}
			if (rstick.GetRawButton(2)){
				can = true;
			}

		}

	}

	void Autnomous(){
		isAutonomoose = true;







		isAutonomoose = false;
	}

private:
	frc::Joystick rstick { 0 };
	frc::Joystick lstick { 1 };
	WPI_TalonSRX m_rf { 1 };
	WPI_TalonSRX m_rr { 12 };
	WPI_TalonSRX m_lf { 0 };
	WPI_TalonSRX m_lr { 13 };
	AHRS MotionTracker {SPI::Port::kMXP}; //new motion tracker
	DoubleSolenoid butterflySol {0, 1};
	static constexpr double kUpdatePeriod = 0.005;
};

START_ROBOT_CLASS(Robot)

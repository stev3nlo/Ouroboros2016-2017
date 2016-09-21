package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Steven on 9/21/2016.
 */
public abstract class MyOpMode extends LinearOpMode {

	//drive train motors
	DcMotor R1;
	DcMotor R2;
	DcMotor L1;
	DcMotor L2;

	//Adafruit IMU Object
	SensorAdafruitIMU gyro;

	//Modern Robotics Color Sensor Objects
	SensorMRColor colorF;	//front color sensor
	SensorMRColor colorB;	//back color sensor

	//constructor
	public MyOpMode() {
		super();
	}

	//hardwaremaps and sets initial values
	public void initialize() {
		R1 = hardwareMap.dcMotor.get("R1");
		R2 = hardwareMap.dcMotor.get("R2");
		L1 = hardwareMap.dcMotor.get("L1");
		L2 = hardwareMap.dcMotor.get("L2");

		//initialize sensors
		gyro = new SensorAdafruitIMU();
		colorF = new SensorMRColor();
		colorB = new SensorMRColor();

		reset();
	}

	//method used for any base movement
	public void move(double speedR, double speedL) {
		R1.setPower(speedR);
		R2.setPower(speedR);
		L1.setPower(speedL);
		L2.setPower(speedL);
	}

	public void stopMoving() {
		R1.setPower(0);
		R2.setPower(0);
		L1.setPower(0);
		L2.setPower(0);
	}

	//methods used to move forward
	public void moveForwards(double speed) {
		move(speed, speed);
	}

	//method used to move backwards
	public void moveBackwards(double speed) {
		moveForwards(-speed);
	}

	//method used to turn right
	public void turnRight(double speed) {
		move(-speed, speed);
	}

	//method used to turn left
	public void turnLeft(double speed) {
		turnRight(-speed);
	}

	//method that turns right using the AdafruitIMU sensor
	public void gyroTurnRight(double speed, double targetAngle) {
		double startAngle = gyro.getYaw();		//angle of the robot before the turn
		double newAngle = gyro.getYaw();		//angle of the robot during the turn
		turnRight(speed);		//starts the turn
		while (Math.abs(newAngle - startAngle) < targetAngle) {		//uses the abs value so this method can be used to turn the other way
			newAngle = gyro.getYaw();		//updates the new angle constantly
		}
		stopMoving();
	}

	//method that turns left using the sensor
	public void gyroTurnLeft(double speed, double targetAngle) {
		gyroTurnRight(-speed, targetAngle);
	}

	//***not done***
	public void moveToWhiteLine(double speed) {
		int red = colorF.getRed();
		int green = colorF.getGreen();
		int blue = colorF.getBlue();
		int alpha = colorF.getAlpha();

		boolean isWhite = false;
		moveForwards(speed);
		while (!isWhite) {

			if ((red < 0) && (green < 0) && (blue < 0) && (alpha < 0)) {	//need to test values for white
				isWhite = true;
			}
		}
		stopMoving();
	}

	public void reset() {
		R1.setPower(0);
		R2.setPower(0);
		L1.setPower(0);
		L2.setPower(0);
	}
}

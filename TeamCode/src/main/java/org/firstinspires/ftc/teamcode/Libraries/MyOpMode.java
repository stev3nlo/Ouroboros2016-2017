package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.TreeMap;

import static java.lang.Integer.parseInt;

/**
 * @author 		Steven Lo
 * @version 	1.0			(I don't know what this really is)
 * @since 		1.0 		(IDK about this either)
 * @date 		9/21/2016.
 */
public abstract class MyOpMode extends LinearOpMode {

	public static final int encoderTicksPerRotation = 1140;
	public static final int goalRPM = 150;
	public static final double motorL1SpeedMultiplier = 1.0;
	public static final double motorL2SpeedMultiplier = 1.0;
	public static final double motorR1SpeedMultiplier = 1.0;
	public static final double motorR2SpeedMultiplier = 1.0;
	protected static final int timeToDropBalls = 3;
	protected double timeSinceLastStabilization = 0.0;
	protected TreeMap<Double,Long> RPMs = new TreeMap<Double,Long>();
	protected double curRPM;
	protected double initTime;
	protected int numCyclesOfSlowingSpinner = 0;
	protected double timeAtLastSpinnerSlowdown;
	protected double initAngle = 180.0;

	protected double driftCounterDivisorLeft = 1.0;
	protected double driftCounterDivisorRight = 1.0;

	public boolean areRollersDropping = false;
	public boolean areRollersRaising = false;
	public double speedOfMovingRollers = 0.1;
	public double startTimeOfDroppingRollers = 0.0;
	public double startTimeOfRaisingRollers = 0.0;
	public double rollerMovementTimeUp = 1.32;
	public double rollerMovementTimeDown = 1.3;


	//drive train motors
	/**
	 * DcMotor variable for the first motor on the right
	 */
	protected DcMotor motorR1;
	/**
	 * DcMotor variable for the second motor on the right
	 */
	protected DcMotor motorR2;
	/**
	 * DcMotor variable for the first motor on the left
	 */
	protected DcMotor motorL1;
	/**
	 * DcMotor variable for the second motor on the left
	 */
	protected DcMotor motorL2;
	/**
	 * DcMotor variable for the motor on the manipulator
	 */
	protected DcMotor motorManip;

	protected DcMotor motorSpinner;

	protected Servo servoDropper;	// servo for manipulator
	protected Servo servoBeaconPusher;
	protected Servo servoWheelBeaconPusher;
	protected CRServo servoRollerF;
	protected Servo servoRollerB;
	//protected Servo servoRangeF;
	//protected Servo servoRangeB;



	//sensors
	/**
	 * Adafruit IMU Object
	 *
	 */
	protected SensorAdafruitIMU gyro;

	/**
	 * Middle Modern Robotics Color Sensor object
	 */
	//protected SensorMRColor colorC;
	/**
	 * Rear Modern Robotics Color Sensor object
	 */
	//protected SensorMRColor colorR;
	/**
	 * Modern Robotics Color Sensor for beacon
	 */
	protected SensorMRColor colorB;
	/**
	 * Modern Robotics Range Sensor that uses ultraSonic and Optical Distance
	 */
	protected SensorMRRange rangeF;
	protected SensorMRRange rangeB;
	//protected MROpticalDistanceSensor ods;

	//Speed values for motors
	//protected double curPowerOfMotorR1 = 0.0;
	//protected double curPowerOfMotorL1 = 0.0;
	//protected double curPowerOfMotorManip = 0.0;
	protected double curPowerOfMotorSpinner = 0.61;

	public long spinnerEncoderOffset = 0;

	long timeAtEndOfLastCycle;

	public double timeAtLastStabilization;
	double timeBallsFinishDropping;

	boolean firstCycleOfSpinner;


	double curTime;

	//constructor
	public MyOpMode()
	{
		super();
	}

	//hardware maps and sets initial values

	/**
	 * Hardware maps and initializes motors and sensors.
	 * <p>
	 *     Sets up all the motors and sensors and sets their initial values
	 * </p>
	 */
	public void initialize()
	{
		//hardware maps the drive motors
		motorR1 = hardwareMap.dcMotor.get("motorR1");
		motorR2 = hardwareMap.dcMotor.get("motorR2");
		motorL1 = hardwareMap.dcMotor.get("motorL1");
		motorL2 = hardwareMap.dcMotor.get("motorL2");
		motorManip = hardwareMap.dcMotor.get("motorManip");
		motorSpinner = hardwareMap.dcMotor.get("motorSpinner");
		servoDropper = hardwareMap.servo.get("servoDropper");
		servoRollerF = hardwareMap.crservo.get("servoRollerF");
		servoRollerB = hardwareMap.servo.get("servoRollerB");
		closeServoDropper();
		servoBeaconPusher = hardwareMap.servo.get("servoBeaconPusher");
		moveBeaconPusherIn();
		moveBackRollerDown(); //for some reason needs to be down, movebackrollerup made it in down position
		curPowerOfMotorSpinner = 0.61;
		motorSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		reset();
	}

	public double getVoltage(String motorControllerName)
	{
		return hardwareMap.voltageSensor.get(motorControllerName).getVoltage();
	}

	//initialize sensors
	public void initializeSensors() throws InterruptedException
	{
		//telemetry.addData("Gyro", "Initializing");
		//telemetry.update();

		//pause(1.0);
		//colorC = new SensorMRColor(hardwareMap.colorSensor.get("colorC"));
		//colorR = new SensorMRColor(hardwareMap.colorSensor.get("colorR"));
		colorB = new SensorMRColor(hardwareMap.colorSensor.get("colorB"));
		telemetry.addData("ColorB", "Initialized");
		telemetry.update();
		//pause(1.0);
		//rangeF = new SensorMRRange(hardwareMap.get(I2cDevice.class,"rangeF"));
		//telemetry.addData("RangeF", "Initialized");
		//telemetry.update();
		//pause(1.0);
		//rangeB = new SensorMRRange(hardwareMap.get(I2cDevice.class,"rangeB"));
	//	telemetry.addData("RangeB", "Initialized");
		//telemetry.update();
		//pause(1.0);
		//colorC.sensorSetup(0x2e);
		//colorR.sensorSetup(0x2a);
		colorB.sensorSetup(0x2c);
		telemetry.addData("ColorB", "I2C address");
		telemetry.update();
		//pause(1.0);
		//rangeF.sensorSetup(0x4a);
		//telemetry.addData("RangeF", "I2C address");
		//telemetry.update();
		//pause(1.0);
		//rangeB.sensorSetup(0x4c);
		//telemetry.addData("RangeB", "I2C address");
		//telemetry.update();
		//pause(1.0);
		telemetry.addData("sensors", "initialized");
		telemetry.update();
		//pause(1.0);
		gyro = new SensorAdafruitIMU(hardwareMap.get(BNO055IMU.class, "gyro"));
		telemetry.addData("Gyro", "Initialized");
		telemetry.update();
	}

	public void pullInWheelBeaconPusher()
	{
		servoWheelBeaconPusher.setPosition(0.5);
	}

	public void pushOutWheelBeaconPusher()
	{
		servoWheelBeaconPusher.setPosition(0.15);
	}

//	public void initializeBlueServos()
//	{
//		servoRangeF.setPosition(.35); //NEEDS TO be TESTED, will be angled to wall
//		servoRangeB.setPosition(0.5); // NEEDS TO BE TESTED, will be PARALLEL to wall
//	}
//
//	public void initializeRedServos()
//	{
//		servoRangeB.setPosition(.65); //NEEDS TO be TESTED, will be angled to wall
//		servoRangeF.setPosition(0.5); // NEEDS TO BE TESTED, will be PARALLEL to wall
//	}

	/**
	 * Moves all the drive motors with given speed for the left and right sides
	 * separately. This is the base method used for all movement with the base,
	 * i.e. moving and backwards and turning left and right.
	 *
	 * @param speedL
	 * @param speedR
	 */
	public void move(double speedL, double speedR)
	{
		motorL1.setPower(speedL/driftCounterDivisorLeft);
		motorL2.setPower(speedL/driftCounterDivisorLeft);
		motorR1.setPower(speedR/driftCounterDivisorRight);
		motorR2.setPower(speedR/driftCounterDivisorRight);
	}

	public void moveManip(double speed)
	{
		motorManip.setPower(speed);
	}

	public void runSpinner(double speed)
	{
		motorSpinner.setPower(speed);
	}

	public void setServoDropperPosition(double position)
	{
		servoDropper.setPosition(position);
	}

	public void openServoDropper()
	{
		if(opModeIsActive())
			setServoDropperPosition(0.45);
	}

	public void closeServoDropper()
	{
		setServoDropperPosition(0.75);
	}

	public long getSpinnerEncoderVal()
	{
		return motorSpinner.getCurrentPosition();
	}

	public long getMotorL1EncoderVal()
	{
		return motorL1.getCurrentPosition();
	}

	public long getMotorL2EncoderVal()
	{
		return motorL2.getCurrentPosition();
	}

	public long getMotorR1EncoderVal()
	{
		return motorR1.getCurrentPosition();
	}

	public long getMotorR2EncoderVal()
	{
		return motorR2.getCurrentPosition();
	}

	/**
	 * Stops all of the motors.
	 * <p>
	 *     This method is used to stop the drive motors.
	 * </p>
	 */
	public void stopMotors() {
		motorR1.setPower(0.0);
		motorR2.setPower(0.0);
		motorL1.setPower(0.0);
		motorL2.setPower(0.0);
	}

	/**
	 * Sets the speed of the motors controlling the wheels on the left and right side of
	 * the robot to the same value so it moves forward
	 * <p>
	 *     Calls the move method and gives a speed
	 * </p>
	 * @param speed
	 */
	public void moveForwards(double speed) {
		if(speed<0)
			speed *= -1;
		move(-speed, speed);
	}

	public void moveForwards(double speedL, double speedR)
	{
		if(speedL<0)
			speedL *= -1;
		if(speedR<0)
			speedR *= -1;
		move(-speedL, speedR);
	}

	/**
	 * Sets the speed of the motors controlling the wheels on the left and right side of
	 * the robot to the same value so it moves backwards
	 * <p>
	 *     Calls the moveForward method and gives an opposite speed
	 * </p>
	 * @param speed
	 */
	public void moveBackwards(double speed)
	{
		if(speed<0)
			speed *= -1;
		move(speed, -speed);
	}

	public void moveBackwards(double speedL, double speedR)
	{
		if(speedL<0)
			speedL *= -1;
		if(speedR<0)
			speedR *= -1;
		move(speedL, -speedR);
	}

	/**
	 * Sets the speed of the motors controlling the wheels on the left and right side of
	 * the robot to opposite values so it turns to the right
	 * <p>
	 * 	   Calls the move method and gives a speed for the right side and a negative speed
	 *     for the left side so it turns to the right
	 * </p>
	 * @param speed
	 */
	public void turnRight(double speed) {
		if(speed < 0)
			speed *= -1;
		move(-speed, -speed);
	}

	/**
	 * Sets the speed of the motors controlling the wheels on the left and right side of
	 * the robot to opposite values so it turns left
	 * <p>
	 * 	   Calls the turnRight method and gives the opposite speed to turn the other way to the left
	 * </p>
	 * @param speed
	 */
	public void turnLeft(double speed) {
		if(speed < 0)
			speed *= -1;
		move(speed,speed);
	}

	public void arcTurnRight(double speed)
	{
		if(speed > 0)
			moveForwards(0.03,speed);
		else if(speed < 0)
			moveBackwards(0.03,speed);
	}
	public void arcTurnRightDrift(double speed)
	{
		if(speed > 0)
			moveForwards(speed*.5,speed);
		else if(speed < 0)
			moveBackwards(speed*.5,speed);
	}

	public void arcTurnLeft(double speed) {
		if(speed > 0)
			moveForwards(speed, 0.03);
		else if(speed < 0)
			moveBackwards(speed,0.03);
	}

	public void gyroArcTurnRight(double speed, double targetAngle) throws InterruptedException
	{
		double startAngle = gyro.getYaw();		//angle of the robot before the turn
		double newAngle = gyro.getYaw();		//angle of the robot during the turn
		arcTurnRight(speed);		//starts the turn
		while (opModeIsActive() && getAngleDiff(newAngle, startAngle) < targetAngle) {		//uses the abs value so this method can be used to turn the other way
			telemetry.addData("startAngle",startAngle);
			telemetry.addData("newAngle",newAngle);
			/*
			if(getAngleDiff(newAngle,startAngle)>targetAngle/2)
			{
				rangeF.getUltraSonicDistance();
				rangeB.getUltraSonicDistance();
			}*/
			//rangeF.getUltraSonicDistance();
			//rangeB.getUltraSonicDistance();
			telemetry.update();
			newAngle = gyro.getYaw();		//updates the new angle constantly
			idle();
		}
		move(0.0,0.0);
		//if(speed>0)
		//	turnLeft(0.03);
		//else
		//	turnRight(0.03);
		try{pause(0.2);}catch(Exception e){}
	}

	public void gyroArcTurnLeft(double speed, double targetAngle) throws InterruptedException
	{
		double startAngle = gyro.getYaw();		//angle of the robot before the turn
		double newAngle = gyro.getYaw();		//angle of the robot during the turn
		arcTurnLeft(speed);		//starts the turn
		while (opModeIsActive() && getAngleDiff(newAngle, startAngle) < targetAngle) {		//uses the abs value so this method can be used to turn the other way
			telemetry.addData("startAngle",startAngle);
			telemetry.addData("newAngle",newAngle);
			telemetry.update();
			newAngle = gyro.getYaw();		//updates the new angle constantly
			idle();
		}
		stopMotors();
	}

	/**
	 * This method will be used to turn to the right when we need to adjust to shoot after
	 * getting the beacon.
	 * <p>
	 *     This uses the gyro object and constantly gets the yaw value and turns until it reaches
	 *     the target yaw value. This will allow us to turn with enough accuracy to score the
	 *     balls in the vortex.
	 * </p>
	 * @param speed
	 * @param targetAngle
	 */
	public void gyroTurnRight(double speed, double targetAngle) throws InterruptedException {
		double startAngle = gyro.getYaw();		//angle of the robot before the turn
		double newAngle = gyro.getYaw();		//angle of the robot during the turn
		turnRight(speed);		//starts the turn
		initCurtime();
		double startTime = getCurTime();
		while (getCurTime() - startTime < 1.0 && opModeIsActive() && getAngleDiff(newAngle, startAngle) < targetAngle) {		//uses the abs value so this method can be used to turn the other way
			//telemetry.addData("startAngle",startAngle);
			//telemetry.addData("newAngle",newAngle);
			//telemetry.update();
			newAngle = gyro.getYaw();		//updates the new angle constantly
			turnRight(speed * ((1 - ((getAngleDiff(startAngle,newAngle) / targetAngle) / 1.2))));
			initCurtime();
			idle();
		}
		stopMotors();
	}

	/**
	 * This method will be used to turn to the left for when we are on the other side of the field
	 * <p>
	 *     This method uses the gyroTurnRight method and gives it the opposite speed to make it
	 *     turn to the left instead of the right.
	 * </p>
	 * @param speed
	 * @param targetAngle
	 */
	public void gyroTurnLeft(double speed, double targetAngle) throws InterruptedException {
		double startAngle = gyro.getYaw();		//angle of the robot before the turn
		double newAngle = gyro.getYaw();		//angle of the robot during the turn
		turnLeft(speed);		//starts the turn
		initCurtime();
		double startTime = getCurTime();
		while (getCurTime() - startTime < 1.0 && opModeIsActive() && getAngleDiff(newAngle, startAngle) < targetAngle) {		//uses the abs value so this method can be used to turn the other way
			//telemetry.addData("startAngle",startAngle);
			//telemetry.addData("newAngle",newAngle);
			//telemetry.update();
			newAngle = gyro.getYaw();		//updates the new angle constantly
			turnLeft(speed * ((1 - ((getAngleDiff(startAngle, newAngle) / targetAngle) / 1.2))));
			initCurtime();
			idle();
		}
		stopMotors();
	}

	public double getAngleDiff(double angle1, double angle2)
	{
		if(Math.abs(angle1 - angle2) < 180.0)
			return Math.abs(angle1-angle2);
		else if(angle1 > angle2)
		{
			angle1 -= 360;
			return Math.abs(angle2-angle1);
		}
		else
		{
			angle2 -= 360;
			return Math.abs(angle1-angle2);
		}
	}
	public void gyroTurnRightCorrection(double speed, double targetAngle) throws InterruptedException {
		double startAngle = gyro.getYaw();
		double newAngle = gyro.getYaw();
		turnRight(speed);
		double angleDiff = getAngleDiff(startAngle, newAngle);
		while(opModeIsActive() && angleDiff < targetAngle) {
			newAngle = gyro.getYaw();
			angleDiff = getAngleDiff(startAngle, newAngle);
			idle();
		}
		turnLeft(speed * .5);
		while(opModeIsActive() && angleDiff > targetAngle) {
			newAngle = gyro.getYaw();
			angleDiff = getAngleDiff(startAngle, newAngle);
			idle();
		}
		stopMotors();
	}

	public void gyroTurnLeftCorrection(double speed, double targetAngle) throws InterruptedException {
		gyroTurnRightCorrection(-speed, -targetAngle);
	}

	/**
	 * This method moves the robot forward until the first color sensor contacts the white line
	 * <p>
	 *     This method uses the color sensor. It makes the robot continue to move forward until the
	 *     color sensor gets values that indicate it's over the white line, at which point it will
	 *     stop.
	 * </p>
	 * @param speed
	 */
//	public void moveToWhiteLine(double speed) throws InterruptedException {
//		telemetry.addData("current alpha", colorC.getAlpha());
//		telemetry.update();
//		moveForwards(speed);
//		while (colorC.groundColor().equals("Gray")) {
//			telemetry.addData("speed", speed);
//			telemetry.addData("current alpha", colorC.getAlpha());
//			telemetry.addData("Ground Color", colorC.groundColor());
//			telemetry.update();
//			idle();
//		}
//		telemetry.addData("at white line", "");
//		telemetry.addData("ground Color", colorC.groundColor());
//		telemetry.update();
//		stopMotors();
//	}

	/**
	 * This method will align the robot with the white line with the second color sensor.
	 * <p>
	 *     This method will continuously turn right until the second color sensor gets values that
	 *     indicate that it's over the white line, meaning both of the color sensors will now be
	 *     over the white line, aligning the robot up to the beacon.
	 * </p>
	 * @param speed
	 */
//	public void turnRightToWhiteLine(double speed) throws InterruptedException {
//		turnRight(speed);
//		while (!colorR.groundColor().equals("White")) {
//			idle();
//		}
//		stopMotors();
//	}


	/**
	 * This method gives the opposite speed to the turnRightToWhiteLine method to do the same thing
	 * but on the other side of the field (turning left)
	 * <p>
	 *     The sensor will be used the same way as the turnRightToWhiteLine method but instead of
	 *     turning to the right, it will give a negative speed to make it turn left instead of
	 *     right.
	 * </p>
	 * @param speed
	 */
//	public void turnLeftToWhiteLine(double speed) throws InterruptedException {
//		turnRightToWhiteLine(-speed);
//	}

//	public void moveBackToBeacon(String side)
//	{
//		while (!(colorB.getColor().equals(side)))
//		{
//			moveBackwards(1.0);
//		}
//		stopMotors();
//	}

//	public void moveForwardToBeacon(String side)
//	{
//		while (!(colorB.getColor().equals(side)))
//		{
//			moveForwards(1.0);
//		}
//		stopMotors();
//	}





	//Moving to and from beacon methods
	/*
	public void moveForwardToBeacon(double speed) throws InterruptedException {
		moveForwards(speed);
		while (!rangeF.inFrontOfBeacon()) {
			idle();
		}
		stopMotors();
	}
	*/

	public void moveAwayFromBeacon(double speed, int distance) throws InterruptedException {
		moveBackwards(speed);
//		while (!(rangeF.getDistanceCM() > distance)) {
//			idle();
//		}
		stopMotors();
	}



	//Essential shooter methods
	/**
	 * This method will reset all the values to the default (stopped, initial positions)
	 */
	public void initShooter()
	{
		firstCycleOfSpinner = true;
	}

	public void initCurtime()
	{
		if(opModeIsActive())
			curTime = ((double)System.nanoTime())/1000000000.0;
	}

	public double getCurTime()
	{
		return curTime;
	}

	public void shoot() {
		runSpinner(1.0);
		initCurtime();
		double startTime = getCurTime();
		openServoDropper();
		while(startTime - startTime > timeToDropBalls)
		{
			idle();
		}
		closeServoDropper();
		runSpinner(0.0);
	}

	public void moveToRangeFromWall(double speed, int range, boolean isBlue) {
		double rawUSD;
		if (opModeIsActive()) {
//			if (isBlue) {
//				rawUSD = rangeF.getDistanceCM();
//				moveForwards(speed);
//				while ((rawUSD > range) && opModeIsActive()) {
//					rawUSD = rangeF.getDistanceCM();
//				}
//				stopMotors();
//			} else {
//				rawUSD = rangeB.getDistanceCM();
//				moveBackwards(speed);
//				while ((rawUSD > range) && opModeIsActive()) {
//					rawUSD = rangeB.getDistanceCM();;
//				}
//			}
		}
	}

	public void arcTurnRightToWall(double speed)
	{
		double USDF = rangeF.getUltraSonicDistance();
		double USDB = rangeB.getUltraSonicDistance();
		if(speed > 0)
		{
			while(USDF<USDB && opModeIsActive())
			{
				telemetry.addData("arcTurning","to wall");
				telemetry.addData("USDF",USDF);
				telemetry.addData("USDB",USDB);
				telemetry.update();
				USDF = rangeF.getUltraSonicDistance();
				USDB = rangeB.getUltraSonicDistance();
				arcTurnRightDrift(speed);
				idle();
			}
		}
		else
		{
			while(USDF>USDB && opModeIsActive())
			{
				telemetry.addData("arcTurning","to wall");
				telemetry.addData("USDF",USDF);
				telemetry.addData("USDB",USDB);
				telemetry.update();
				USDF = rangeF.getUltraSonicDistance();
				USDB = rangeB.getUltraSonicDistance();
				arcTurnRightDrift(speed);
				idle();
			}
		}
	}

	public void arcTurnLeftToWall(double speed)
	{
		double USDF = rangeF.getUltraSonicDistance();
		double USDB = rangeB.getUltraSonicDistance();
		if(speed > 0)
		{
			while(USDF>USDB && opModeIsActive())
			{
				USDF = rangeF.getUltraSonicDistance();
				USDB = rangeB.getUltraSonicDistance();
				telemetry.addData("arcTurning","to wall");
				telemetry.addData("USDF", USDF);
				telemetry.addData("USDB",USDB);
				telemetry.update();
				arcTurnLeft(speed);
				idle();
			}
		}
		else
		{
			while(USDF<USDB && opModeIsActive())
			{
				telemetry.addData("arcTurning","to wall");
				telemetry.addData("USDF",USDF);
				telemetry.addData("USDB",USDB);
				telemetry.update();
				USDF = rangeF.getUltraSonicDistance();
				USDB = rangeB.getUltraSonicDistance();
				arcTurnLeft(speed);
				idle();

			}
		}
	}

	public double lookUpTurningAngleForRangeDiff(double diff)
	{
		int intDiff = (int)diff;
		switch(intDiff)
		{
			case 0: return 0.0;
			case 1: return 0.75;
			case 2: return 3.3;
			case 3: return 6.1;
			case 4: return 9.8;
			case 5: return 10.1;
			case 6: return 13.7;
			case 7: return 15.4;
			case 8: return 18.25;
			//unsure cases
			case 9: return 21.0;
			case 10: return 24.0;
			case 11: return 26.6;
			case 12: return 29.7;
			case 13: return 32.8;
			case 14: return 36.0;
			case 15: return 38.6;
			case 16: return 42.2;
		}
		return 42.2;
	}
	public void turnParallelToWallWithGyroSimple(double speed, double count) throws InterruptedException{
		double USDF = -1;
		double USDB = -1;
		boolean broken = false;
		double startYaw = gyro.getYaw();

		if (opModeIsActive()) {
			USDF = rangeF.getUltraSonicDistance();
			USDB = rangeB.getUltraSonicDistance();
			String caseName = getCaseNameFromInfo(USDF,USDB,10,4.0);
			double startTime = getCurTime();
			while((USDB == -1 || USDF == -1) && opModeIsActive())
			{
				telemetry.addData("USDF", USDF);
				telemetry.addData("USDB", USDB);
				telemetry.addData("count",count);
				telemetry.update();
				initCurtime();
				if(getCurTime() - startTime > 0.75) {
					broken = true;
					break;
				}
				idle();
			}
			if (!broken && USDF > USDB && !caseName.equals("22")) {
				gyroTurnRight(speed, lookUpTurningAngleForRangeDiff(USDF - USDB));
				turnLeft(0.03);
			} else if (!broken && USDB > USDF && !caseName.equals("00")){
				gyroTurnLeft(speed,lookUpTurningAngleForRangeDiff(USDB-USDF));
				turnRight(0.03);
			}
		}
		try{pause(0.25);}catch(InterruptedException e){}
	}
	public void turnParallelToWallWithGyro(double speed, double count) throws InterruptedException{
		double USDF = -1;
		double USDB = -1;
		boolean broken = false;
		double startYaw = gyro.getYaw();
		if (opModeIsActive()) {
			USDF = rangeF.getUltraSonicDistance();
			USDB = rangeB.getUltraSonicDistance();
			/*
			if(count == 0 && USDF > USDB)
				count = 3;
			else if(USDB > USDF && count == 0)
				count = 2;*/
			double startTime = getCurTime();
			while(opModeIsActive())
			{
				telemetry.addData("USDF", USDF);
				telemetry.addData("USDB", USDB);
				telemetry.addData("count",count);
				telemetry.update();
				initCurtime();
				if(getCurTime() - startTime > 0.25) {
					if(USDF==-1 || USDB == -1)
						broken = true;
					break;
				}
				idle();
			}
			double distFrom180 = getAngleDiff(initAngle, startYaw);
			double turningAngle = lookUpTurningAngleForRangeDiff(USDF - USDB);
			/*
			if(turningAngle > distFrom180 * 1.5) //use distfrom180
			{
				if (startYaw > initAngle) {
					gyroTurnLeft(speed, distFrom180);
					turnRight(0.03);
				} else if (USDB > USDF) {
					gyroTurnRight(speed, distFrom180);
					turnLeft(0.03);
				}
			}
			else {*/
			telemetry.addData("USDF",USDF);
			telemetry.addData("USDB",USDB);
			telemetry.update();
			pause(2.0);
				if (!broken && USDF > USDB) {
					gyroTurnRight(speed, lookUpTurningAngleForRangeDiff(USDF - USDB));
					turnLeft(0.03);
				} else if (!broken && USDB > USDF) {
					gyroTurnLeft(speed, lookUpTurningAngleForRangeDiff(USDB - USDF));
					turnRight(0.03);
				}
			//}
		}
		try{pause(0.25);}catch(InterruptedException e){}
		if(count < 0 && !broken && (Math.abs(USDF - USDB) >= 1.0))
		{
			turnParallelToWallWithGyro(speed, count + 1);
		}
	}

	public void turnParallelToWall(double speed) {
		double USDF = -1;
		double USDB = -1;
		boolean broken = false;
		double startYaw = gyro.getYaw();
		if (opModeIsActive()) {
			USDF = rangeF.getUltraSonicDistance();
			USDB = rangeB.getUltraSonicDistance();
			double startTime = getCurTime();
			while((USDB == -1 || USDF == -1) && opModeIsActive())
			{
				telemetry.addData("USDF", USDF);
				telemetry.addData("USDB", USDB);
				telemetry.update();
				initCurtime();
				if(getCurTime() - startTime > 1.5) {
					broken = true;
					break;
				}
				idle();
			}
			if (USDF > USDB) {
				turnRight(speed);
				while ((USDF > USDB) && opModeIsActive()) {
					if(broken)
						break;
					telemetry.addData("turning","to wall");
					telemetry.addData("USDF",USDF);
					telemetry.addData("USDB",USDB);
					telemetry.update();
					USDF = rangeF.getUltraSonicDistance();
					USDB = rangeB.getUltraSonicDistance();
					idle();
				}
				turnLeft(0.05);
			} else {
				if (USDF < USDB) {
					turnLeft(speed);
					while ((USDF < USDB) && opModeIsActive()) {
						if(broken)
							break;
						telemetry.addData("turning","to wall");
						telemetry.addData("USDF",USDF);
						telemetry.addData("USDB",USDB);
						telemetry.update();
						USDF = rangeF.getUltraSonicDistance();
						USDB = rangeB.getUltraSonicDistance();
						idle();
					}
					turnRight(0.05);
				}
			}
		}
		try{pause(0.25);}catch(InterruptedException e){}
		double endYaw = gyro.getYaw();
		if(!broken && (Math.abs(USDF - USDB) >= 1.0 || getAngleDiff(startYaw,endYaw) >= 5.0))
		{
			turnParallelToWall(speed - 0.01);
		}
	}

	public String getCaseNameFromInfo(double USDF, double USDB, int targetDist, double thresholdW)
	{
		String name = "";
		if(USDF>=-1.1 && USDF <= -0.9 || USDB>=-1.1 && USDB <= -0.9)
			return "11";
		if(USDF>targetDist+thresholdW)
		{
			name += "0";
		}
		else if(USDF<targetDist-thresholdW)
		{
			name += "2";
		}
		else
		{
			name += "1";
		}

		if(USDB>targetDist+thresholdW)
		{
			name += "0";
		}
		else if(USDB<targetDist-thresholdW)
		{
			name += "2";
		}
		else
		{
			name += "1";
		}
		return name;
	}

	public void driveToNextBeacon(double speed, boolean isBlue, int encoderDist, double leftMult, double rightMult)
	{
		String color;
		if (colorB.beaconColor().equals("Blue"))
		{
			color = "Blue";
		}
		else if (colorB.beaconColor().equals("Red"))
		{
			color = "Red";
		}
		else
		{
			color = "Neither";
		}
		String targetColor;
		if(isBlue)
			targetColor = "Blue";
		else
			targetColor = "Red";
		int currEnc = getAvgEnc();
		int avgEnc = currEnc;
		int distMoved = Math.abs(avgEnc - currEnc);
		double curSpeedL = -leftMult * speed;
		double curSpeedR = rightMult * speed;
		while(opModeIsActive() && (distMoved < encoderDist || !color.equals(targetColor)))
		{
			if(distMoved < encoderDist)
			{
				curSpeedL = -leftMult * speed * ((1 - (((double) distMoved / (double) encoderDist) / 2)));
				curSpeedR = rightMult * speed * ((1 - (((double) distMoved / (double) encoderDist) / 2)));
			}
			move(curSpeedL,curSpeedR);
			if (colorB.beaconColor().equals("Blue"))
			{
				color = "Blue";
			}
			else if (colorB.beaconColor().equals("Red"))
			{
				color = "Red";
			}
			else
			{
				color = "Neither";
			}
			avgEnc = getAvgEnc();
			distMoved = Math.abs(avgEnc - currEnc);
			telemetry.addData("distMoved",distMoved);
			telemetry.update();
			idle();
		}
		moveForwards(0.0);
	}

	public boolean driveAlongWallToBeaconOrForUnits(double speed, boolean isBlue, int encoderDist, double leftMult, double rightMult)
	{
		boolean foundBeacon = false;
		String color;
		if (colorB.beaconColor().equals("Blue"))
		{
			color = "Blue";
		}
		else if (colorB.beaconColor().equals("Red"))
		{
			color = "Red";
		}
		else
		{
			color = "Neither";
		}
		int currEnc = getAvgEnc();
		int avgEnc = currEnc;
		int distMoved = Math.abs(avgEnc - currEnc);
		if(isBlue)
		{
			while (!color.equals("Blue") && distMoved < encoderDist && opModeIsActive()) {
				avgEnc = getAvgEnc();
				distMoved = Math.abs(avgEnc - currEnc);
				if (speed > 0)
					moveForwards(speed*leftMult,speed*rightMult);
				else
					moveBackwards(-speed*leftMult,speed*rightMult);
				if (colorB.beaconColor().equals("Blue"))
				{
					foundBeacon = true;
					color = "Blue";
				}
				else if (colorB.beaconColor().equals("Red"))
				{
					color = "Red";
				}
				else
				{
					color = "Neither";
				}
				telemetry.addData("color", color);
				telemetry.update();
				idle();
			}
		}
		else
		{
			while(!color.equals("Red") && distMoved < encoderDist && opModeIsActive())
			{
				avgEnc = getAvgEnc();
				distMoved = Math.abs(avgEnc - currEnc);
				if (speed > 0)
					moveForwards(speed*leftMult,speed*rightMult);
				else
					moveBackwards(-speed*leftMult,speed*rightMult);
				if (colorB.beaconColor().equals("Blue")) {
					color = "Blue";
				} else if (colorB.beaconColor().equals("Red")) {
					foundBeacon = true;
					color = "Red";
				} else {
					color = "Neither";
				}
				telemetry.addData("color", color);
				telemetry.update();
				idle();
			}
		}
		move(0.0, 0.0);
		return foundBeacon;
	}

	public void driveAlongWallToBeacon(double speed, boolean isBlue, double leftMult, double rightMult)
	{
		String color = "Neither";
		if (colorB.beaconColor().equals("Blue"))
		{
			color = "Blue";
		}
		else if (colorB.beaconColor().equals("Red"))
		{
			color = "Red";
		}
		else
		{
			color = "Neither";
		}
		if(isBlue)
		{
			while(!color.equals("Blue") && opModeIsActive())
			{
				if (speed > 0)
					moveForwards(speed*leftMult,speed*rightMult);
				else
					moveBackwards(-speed*leftMult,speed*rightMult);
				if (colorB.beaconColor().equals("Blue"))
				{
					color = "Blue";
				}
				else if (colorB.beaconColor().equals("Red"))
				{
					color = "Red";
				}
				else
				{
					color = "Neither";
				}
				telemetry.addData("color", color);
				telemetry.update();
				idle();
			}
		}
		else
		{
			while(!color.equals("Red") && opModeIsActive())
			{
				if (speed > 0)
					moveForwards(speed*leftMult,speed*rightMult);
				else
					moveBackwards(-speed*leftMult,speed*rightMult);
				if (colorB.beaconColor().equals("Blue")) {
					color = "Blue";
				} else if (colorB.beaconColor().equals("Red")) {
					color = "Red";
				} else {
					color = "Neither";
				}
				telemetry.addData("color", color);
				telemetry.update();
				idle();
			}
		}
		if(speed > 0)
			moveBackwards(0.03);
		else
			moveForwards(0.03);
	}

	public void moveStraight(double speed, boolean isForward)
	{
		if(isForward)
		{
			moveForwards(speed, speed);
		}
		else
		{
			moveBackwards(speed,speed);
		}
	}

	public void moveStraightDriftLeft(double speed, boolean isForward)
	{
		double driftMultiplierLeftSide = 1.4;
		double driftMultiplierRightSide = 0.6;
		if(isForward)
		{
			moveForwards(speed *driftMultiplierLeftSide, speed * driftMultiplierRightSide);
		}
		else
		{
			moveBackwards(speed * driftMultiplierLeftSide, speed * driftMultiplierRightSide);
		}
	}

	public void moveStraightDriftRight(double speed, boolean isForward)
	{
		if(speed < 0)
			speed *= -1;
		double driftMultiplierLeftSide = 1.25;
		if(isForward)
		{
			moveForwards(speed * driftMultiplierLeftSide, speed * 0.75);
		}
		else
		{
			moveBackwards(speed * driftMultiplierLeftSide, speed * 0.6);
		}
	}

	public boolean checkIfShouldBeStabilizing(String color, boolean isBlue, boolean isForEncoderDist, int encoderDist, double startEnc, double currEnc)
	{
		if(opModeIsActive())
		{
			if(isBlue && color.equals("Blue") || !isBlue && color.equals("Red"))
			{
				return false;
			}
			else if(isForEncoderDist && Math.abs(currEnc - startEnc) > encoderDist)
			{
				return false;
			}
		}
		return true;
	}

	public void stabilizeAlongWall(double speed, double thresholdW, int targetDist, boolean isBlue, boolean isForEncoderDist, int encoderDist) throws InterruptedException
	{
		if(!isForEncoderDist)
			encoderDist = Integer.MAX_VALUE;
		String color = "Neither";
		double USDF;
		double USDB;
		double startEnc = getAvgEnc();
		double currEnc = startEnc;
		boolean isForwards = speed > 0;
		while(checkIfShouldBeStabilizing(color,isBlue,isForEncoderDist,encoderDist,startEnc,currEnc))
		{
			USDF = rangeF.getUltraSonicDistance();
			USDB = rangeB.getUltraSonicDistance();
			String caseName = getCaseNameFromInfo(USDF, USDB, targetDist, thresholdW);
			telemetry.addData("stabilizeAlongWallWithRangeToBeacon", "");
			telemetry.addData("caseName", caseName);
			telemetry.addData("color", color);
			telemetry.addData("USDF", USDF);
			telemetry.addData("USDB", USDB);

			currEnc = getAvgEnc();
			telemetry.addData("avg Enc", startEnc);
			telemetry.addData("curr Enc", currEnc);
			startEnc = Math.abs(startEnc - currEnc);

			double distFrom180 = getAngleDiff(initAngle, gyro.getYaw());
			double turningAngle;
			if (USDF > USDB)
				turningAngle = lookUpTurningAngleForRangeDiff(USDF - USDB);
			else
				turningAngle = lookUpTurningAngleForRangeDiff(USDB - USDF);
			switch (caseName) {
				case "00":
					if (USDB - USDF >= 6.0) {
						telemetry.addData("Not drifting", "");
						moveStraight(speed, true);
					} else {
						telemetry.addData("Drifting", "");
						moveStraightDriftLeft(speed, isForwards);
					}
					break;
				case "01":
					arcTurnRightToWall(speed * 1.4);
					break;
				case "02":
					turnParallelToWallWithGyro(speed * 1.15, 0);
					break;
				case "10":
					arcTurnRightToWall(-speed * 1.4);
					break;
				case "11":
					moveStraight(speed, isForwards);
					break;
				case "12":
					arcTurnLeftToWall(speed * 1.4);
					break;
				case "20":
					turnParallelToWallWithGyro(speed * 1.15, 0);
					break;
				case "21":
					arcTurnLeftToWall(-speed * 1.4);
					break;
				case "22":
					moveStraightDriftRight(speed, isForwards);
					break;
			}
			if (colorB.beaconColor().equals("Blue")) {
				color = "Blue";
			} else if (colorB.beaconColor().equals("Red")) {
				color = "Red";
			} else {
				color = "Neither";
			}
			telemetry.update();
			idle();
		}
	}



	//Methods to control button pushing
	public void pushButton() {
		if(opModeIsActive()) {
			moveBeaconPusherOut();
			try {
				pause(2.5);
			} catch (Exception e) {
			}
			moveBeaconPusherIn();
		}
	}

	public void pushButtonWithRollers() {
		if(opModeIsActive()) {
			moveBeaconPusherOutRollers();
			try {
				pause(2);
			} catch (Exception e) {
			}
			moveBeaconPusherIn();
		}
	}

	public void pushButtonWithRollersQuick() {
		if(opModeIsActive()) {
			moveBeaconPusherOutRollers();
			try {
				pause(0.5);
			} catch (Exception e) {
			}
			moveBeaconPusherIn();
			try {
				pause(0.5);
			} catch (Exception e) {
			}
		}
	}

	public void pushButtonWithDistance()
	{
		int USDF = rangeF.getUltraSonicDistance();
		int USDB = rangeB.getUltraSonicDistance();
		double dist = 0.0;

		if (Math.abs(USDF - USDB) < 2 && USDF!=-1 && USDB != -1)
		{
			dist = valToPushOutBeaconPusherFromDist(USDF);
			moveBeaconPusherOutDist(dist);
		}
//		else if(USDF != -1 && Math.abs(USDF - USDB) < 4)
//		{
//			dist = valToPushOutBeaconPusherFromDist(Integer.getInteger((USDF+2) + ""));
//			moveBeaconPusherOutDist(dist);
//		}
		else
		{
			moveBeaconPusherOutDist(dist);
		}

	}

	public void pushButtonWithDistanceOutput()
	{
		double USDF = rangeF.getUltraSonicDistance();
		double USDB = rangeB.getUltraSonicDistance();
		double dist = 0.0;

		if (Math.abs(USDF - USDB) < 2 && USDF!=-1 && USDB != -1)
		{
			dist = valToPushOutBeaconPusherFromDist(Integer.getInteger(USDF + ""));
			telemetry.addData("dist", dist);
			telemetry.update();
		}
//		else if(USDF != -1 && Math.abs(USDF - USDB) < 4)
//		{
//			dist = valToPushOutBeaconPusherFromDist(Integer.getInteger((USDF+2) + ""));
//			telemetry.addData("dist", dist);
//			telemetry.update();
//		}Z
		else
		{
			telemetry.addData("dist", dist);
			telemetry.update();
		}

	}

	public double valToPushOutBeaconPusherFromDist(int dist)
	{
		switch(dist)
		{
			case 1: return 0.65;
			case 2: return 0.65;
			case 3: return 0.65;
			case 4: return 0.65;
			case 5: return 0.65;
			case 6: return 0.65;
			case 7: return 0.65;
			case 8: return 0.55;
			case 9: return 0.49;
			case 10: return 0.37;//changed
			case 11: return 0.3;//changed
			case 12: return 0.21;//changed
			case 13: return 0.13;//changed
			case 14: return 0.07;
			case 15: return 0;
			case 16: return 0;
		}
		return 0.0;
	}

	public void pushButtonDistance(double dist) {
		moveBeaconPusherOut();
		try{pause(2.5);}catch(Exception e){}
		moveBeaconPusherIn();
	}

	public void pushButtonWithSpeed() {
		initCurtime();
		double startTime = getCurTime();
		double curTimeInLoop = getCurTime();
		while(curTimeInLoop - startTime < 3.0 && opModeIsActive())
		{
			initCurtime();
			curTimeInLoop = getCurTime();
			double curPos = Math.sqrt(3.0) - Math.sqrt((curTimeInLoop-startTime));
			curPos /= Math.sqrt(3.0);
			servoBeaconPusher.setPosition(curPos);
			idle();
		}
	}

	public void pushButtonWithSpeedUntilFlashing() {
		initCurtime();
		double startTime = getCurTime();
		double curTimeInLoop = getCurTime();
		boolean colorIsGray = false;
		while(!colorIsGray && curTimeInLoop - startTime < 3.0 && opModeIsActive())
		{
			initCurtime();
			curTimeInLoop = getCurTime();
			double curPos = Math.sqrt(3.0) - Math.sqrt((curTimeInLoop-startTime));
			curPos /= Math.sqrt(3.0);
			servoBeaconPusher.setPosition(curPos);
			telemetry.addData("colorB",""+colorB);
			telemetry.update();
			if(colorB.beaconColor().equals("Neither"))
				colorIsGray = true;
			idle();
		}
	}

	public void moveFrontRollerUp()
	{
		servoRollerF.setPower(-1.0);
	}

	public void moveBackRollerUp()
	{
		servoRollerB.setPosition(1.0);
	}

	public void moveFrontRollerDown()
	{
		servoRollerF.setPower(1.0);
	}

	public void moveBackRollerDown()
	{
		servoRollerB.setPosition(0.0);
	}

	public void stopRollers()
	{
		servoRollerF.setPower(0.0);
	}

	public void moveRollersUp()
	{
		servoRollerF.setPower(-1.0);
		servoRollerB.setPosition(0.1);
	}

	public void moveRollersDown()
	{
		servoRollerF.setPower(1.0);
		servoRollerB.setPosition(1.0);
	}

	public void holdRollersUp()
	{
		servoRollerF.setPower(-0.05);
	}

	public void moveBeaconPusherOut()
	{
		double v = 0.0;
		servoBeaconPusher.setPosition(v);
	}

	public void moveBeaconPusherOutRollers()
	{
		double v = 0.35;
		servoBeaconPusher.setPosition(v);
	}

	public void moveBeaconPusherOutDist(double dist)
	{
		servoBeaconPusher.setPosition(dist);
	}

	public void moveBeaconPusherIn()
		{
		//TEST VALUES
		double v = 1.0;
		servoBeaconPusher.setPosition(v);
	}


	public void reset() {
		motorR1.setPower(0);
		//motorR2.setPower(0);
		motorL1.setPower(0);
		//motorL2.setPower(0);
	}

	public void updateMotorSpeeds()
	{
		//motorSpinner.setPower(curPowerOfMotorSpinner);
	}

	public void updateServoPositions()
	{

	}

//	public void moveBackToWhiteLineODS(double speed)
//	{
//		while(ods.lightDetected()<.5) //random number!!! needs to be replaced!!!
//		{
//			while(rangeF.getUltraSonicDistance() > rangeB.getUltraSonicDistance())
//			{
//				move(-0.5,-0.8);
//			}
//			while(rangeF.getUltraSonicDistance() < rangeB.getUltraSonicDistance())
//			{
//				move(-0.8,-0.5);
//			}
//			while (rangeF.getUltraSonicDistance() == rangeB.getUltraSonicDistance())
//			{
//				move(-0.5, -0.5);
//			}
//		}
//		stopMotors();
//
//	}

	public void pause(double t) throws InterruptedException {
		initCurtime();

		//hacked
		double startTime = getCurTime();
		while (opModeIsActive() && getCurTime() < startTime + t)
		{
			initCurtime();
			idle();
		}
	}

	public void update()
	{

	}

	public Map.Entry getFirstSetFromHashMap(HashMap<Double,Long> single)
	{
		Iterator it = single.entrySet().iterator();
		while (it.hasNext() && opModeIsActive()) {
			Map.Entry pair = (Map.Entry)it.next();
			idle();
			return pair;
		}
		return null;
	}

	public HashMap<Double,Long> getFirstEncoderTimeSetAfterTime(double t)
	{
		HashMap<Double,Long> vals = new HashMap<Double,Long>();
		Iterator it = RPMs.entrySet().iterator();
		ArrayList<Double> toRemove = new ArrayList<Double>();
		while (it.hasNext() && opModeIsActive()) {
			Map.Entry pair = (Map.Entry)it.next();
			if((double)pair.getKey()>t)
			{
				vals.put((double)pair.getKey(),(long)pair.getValue());
				break;
			}
			else
			{
				toRemove.add((double)pair.getKey());
			}
			it.remove(); // avoids a ConcurrentModificationException
			idle();
		}
		while(toRemove.size()>0 && opModeIsActive()) {
			RPMs.remove(toRemove.remove(0));
			idle();
		}
		return vals;
	}

	public double oldTime;
	public long oldEncoderVal;
	public double timeSinceLastRPMUpdate;
	public void runRPMStabilization()
	{
		RPMs.put(getCurTime(), getSpinnerEncoderVal());

		telemetry.addData("curRPM", curRPM);
		telemetry.addData("goalRPM",goalRPM);
		if(getCurTime()-initTime>3.0 && getCurTime() - timeAtLastStabilization > 0.3) {
			//if(getCurTime() > initTime + 0.25) {
			HashMap<Double, Long> firstEncoderTimeSetAfterTime = getFirstEncoderTimeSetAfterTime(getCurTime() - 0.75);
			Map.Entry pair = getFirstSetFromHashMap(firstEncoderTimeSetAfterTime);
			oldTime = (double) pair.getKey();
			oldEncoderVal = (long) pair.getValue();
			curRPM = getSpinnerEncoderVal() - oldEncoderVal;
			curRPM /= 1120; //Number of rotations since last run
			timeSinceLastRPMUpdate = getCurTime() - oldTime;
			curRPM /= getCurTime() - oldTime; //Number of rotations per second
			curRPM *= 60.0;
			//telemetry.addData("motorRPM", curRPM);
			if(curRPM>goalRPM)
				curPowerOfMotorSpinner -= Math.sqrt(Math.abs(((curRPM - goalRPM)*(curRPM-goalRPM)*(curRPM-goalRPM)))) / (10000);
			else
				curPowerOfMotorSpinner += Math.sqrt(Math.abs(((curRPM - goalRPM)*(curRPM-goalRPM)*(curRPM-goalRPM)))) / (10000);
			if (curPowerOfMotorSpinner > 1.0)
				curPowerOfMotorSpinner = 1.0;
			else if (curPowerOfMotorSpinner < 0.0)
				curPowerOfMotorSpinner = 0.0;
			timeAtLastStabilization = getCurTime();
		}
		telemetry.addData("curTime","" + getCurTime());
		telemetry.addData("curEncoderVal", "" + getSpinnerEncoderVal());
		telemetry.addData("oldTime",oldTime);
		telemetry.addData("oldEncoderVal", oldEncoderVal);
		telemetry.addData("timeSinceLastRPMUpdate", timeSinceLastRPMUpdate);
	}

	public void runRPMStabilizationAuto()
	{
		RPMs.put(getCurTime(), getSpinnerEncoderVal());
		telemetry.addData("curRPM", curRPM);
		telemetry.addData("goalRPM",goalRPM);
		if(getCurTime()-initTime>3.0 && getCurTime() - timeAtLastStabilization > 0.3) {
			//if(getCurTime() > initTime + 0.25) {
			HashMap<Double, Long> firstEncoderTimeSetAfterTime = getFirstEncoderTimeSetAfterTime(getCurTime() - 0.2);
			Map.Entry pair = getFirstSetFromHashMap(firstEncoderTimeSetAfterTime);
			oldTime = (double) pair.getKey();
			oldEncoderVal = (long) pair.getValue();
			curRPM = getSpinnerEncoderVal() - oldEncoderVal;
			curRPM /= 1120; //Number of rotations since last run
			timeSinceLastRPMUpdate = getCurTime() - oldTime;
			if(timeSinceLastRPMUpdate != 0) {
				curRPM /= getCurTime() - oldTime; //Number of rotations per second
				curRPM *= 60.0;
				//telemetry.addData("motorRPM", curRPM);
				if (curRPM > goalRPM)
					curPowerOfMotorSpinner -= Math.sqrt(Math.abs(((curRPM - goalRPM) * (curRPM - goalRPM) * (curRPM - goalRPM)))) / (10000);
				else
					curPowerOfMotorSpinner += Math.sqrt(Math.abs(((curRPM - goalRPM) * (curRPM - goalRPM) * (curRPM - goalRPM)))) / (10000);
				if (curPowerOfMotorSpinner > 1.0)
					curPowerOfMotorSpinner = 1.0;
				else if (curPowerOfMotorSpinner < 0.0)
					curPowerOfMotorSpinner = 0.0;
				timeAtLastStabilization = getCurTime();
			}
		}
		telemetry.addData("curTime","" + getCurTime());
		telemetry.addData("curEncoderVal", "" + getSpinnerEncoderVal());
		telemetry.addData("oldTime", oldTime);
		telemetry.addData("oldEncoderVal",oldEncoderVal);
		telemetry.addData("timeSinceLastRPMUpdate", timeSinceLastRPMUpdate);
	}

	public void moveWithEncoders(double speed, int goal, double leftMult, double rightMult) {
		if(opModeIsActive()) {
			int currEnc = getAvgEnc();
			int avgEnc = currEnc;
			if(speed > 0)
				moveForwards(speed * leftMult,speed * rightMult);
			else
				moveBackwards(speed * leftMult, speed * rightMult);
			int distMoved = Math.abs(avgEnc - currEnc);
			while (distMoved < goal && opModeIsActive()) {
				avgEnc = getAvgEnc();
				telemetry.addData("avg Enc", avgEnc);
				telemetry.addData("curr Enc", currEnc);
				move((-speed * leftMult * ((1 - (((double) distMoved / (double) goal) / 2)))), (speed * rightMult * ((1 - (((double) distMoved / (double) goal) / 2)))));
				distMoved = Math.abs(avgEnc - currEnc);
				telemetry.update();
				idle();
			}
			if(speed>0)
				moveBackwards(0.03);
			else
				moveForwards(0.03);
			try{pause(0.2);}catch(Exception e){}
			stopMotors();
		}
	}

	public void moveWithEncodersCoastToWallOrToBeaconWithMinDist(double speed, int goal, int minDist, boolean isBlue, double leftMult, double rightMult) {
		if(opModeIsActive()) {
			int currEnc = getAvgEnc();
			int avgEnc = currEnc;
			double startYaw = gyro.getYaw();
			if(speed > 0)
				moveForwards(speed * leftMult,speed * rightMult);
			else
				moveBackwards(speed * leftMult, speed * rightMult);
			int distMoved = Math.abs(avgEnc - currEnc);
			double minMult = Math.min(leftMult,rightMult);
			while(distMoved < minDist && opModeIsActive())
			{
				avgEnc = getAvgEnc();
				telemetry.addData("avg Enc", avgEnc);
				telemetry.addData("curr Enc", currEnc);
				double curYaw = gyro.getYaw();
				if(getAngleDiff(curYaw,startYaw) < 15) {
					move((-speed * leftMult), (speed * rightMult));
				}
				else
				{
					move(-speed * minMult, speed * minMult);
				}
					//move((-speed * leftMult * ((1 - (((double) distMoved / (double) goal) / 3)))), (speed * rightMult * ((1 - (((double) distMoved / (double) goal) / 3)))));
				distMoved = Math.abs(avgEnc - currEnc);
				telemetry.update();
				idle();
			}
			String color = "Neither";
			if (colorB.beaconColor().equals("Blue"))
			{
				color = "Blue";
			}
			else if (colorB.beaconColor().equals("Red"))
			{
				color = "Red";
			}
			else
			{
				color = "Neither";
			}
			boolean seesBeacon = false;
			while (distMoved < goal && opModeIsActive()) {
				avgEnc = getAvgEnc();
				double curYaw = gyro.getYaw();
				telemetry.addData("avg Enc", avgEnc);
				telemetry.addData("curr Enc", currEnc);
				color = colorB.beaconColor();
				telemetry.addData("AngleDiff",getAngleDiff(curYaw,startYaw));
				if(getAngleDiff(curYaw, startYaw) < 15) {
					moveBackwards(speed*leftMult,speed*rightMult);
					//move((-speed * leftMult * ((1 - (((double) (distMoved-minDist) / (double) (goal-minDist)) / 3)))), (speed * rightMult * ((1 - (((double) distMoved-minDist / (double) goal-minDist) / 3)))));
				}
				else
				{
					moveBackwards(speed*minMult,speed*minMult);
					//move((-speed * minMult * ((1 - (((double) (distMoved-minDist) / (double) (goal-minDist)) / 3)))), (speed * minMult * ((1 - (((double) distMoved-minDist / (double) goal-minDist) / 3)))));
				}
				//if(color.equals("Blue") && isBlue)
				//	seesBeacon = true;
				//else if(color.equals("Red") && !isBlue)
				//	seesBeacon = true;
				distMoved = Math.abs(avgEnc - currEnc);
				telemetry.update();
				idle();
			}
			move(0.0,0.0);
			try{pause(0.2);}catch(Exception e){}
			stopMotors();
		}
	}

	public void moveWithEncodersCoastWithMaxTime(double speed, int goal, double maxTime, double leftMult, double rightMult) {
		if(opModeIsActive()) {
			int currEnc = getAvgEnc();
			int avgEnc = currEnc;
			if(speed > 0)
				moveForwards(speed * leftMult,speed * rightMult);
			else
				moveBackwards(speed * leftMult, speed * rightMult);
			int distMoved = Math.abs(avgEnc - currEnc);
			initCurtime();
			double startTime = getCurTime();
			while (getCurTime()-startTime<maxTime && distMoved < goal && opModeIsActive()) {
				avgEnc = getAvgEnc();
				telemetry.addData("avg Enc", avgEnc);
				telemetry.addData("curr Enc", currEnc);
				getCurTime();
				distMoved = Math.abs(avgEnc - currEnc);
				telemetry.update();
				idle();
			}
			move(0.0,0.0);
			try{pause(0.2);}catch(Exception e){}
			stopMotors();
		}
	}

//	public void moveWithEncodersCoastWithMaxTimeWithDriftAfterContact(double speed, int goal,
//																	  double maxTime,
//																	  double leftMult,
//																	  double rightMult,
//																	  int angChangeGoal) {
//		double startAng = gyro.getYaw();
//		boolean touchedWall = false;
//		if (opModeIsActive()) {
//			int currEnc = getAvgEnc();
//			int avgEnc = currEnc;
//			double angDiff = getAngleDiff(startAng,gyro.getYaw());
//			if (angDiff > angChangeGoal) {
//				touchedWall = true;
//			}
//			if (touchedWall) {
//				if(speed > 0)
//					moveForwards(speed * leftMult,speed * rightMult);
//				else
//					moveBackwards(speed * leftMult, speed * rightMult);
//			} else {
//				if(speed > 0)
//					moveForwards(speed,speed);
//				else
//					moveBackwards(speed, speed);
//			}
//
//			int distMoved = Math.abs(avgEnc - currEnc);
//			initCurtime();
//			double startTime = getCurTime();
//			while (getCurTime()-startTime<maxTime && distMoved < goal && opModeIsActive()) {
//				avgEnc = getAvgEnc();
//				telemetry.addData("avg Enc", avgEnc);
//				telemetry.addData("curr Enc", currEnc);
//				getCurTime();
//				distMoved = Math.abs(avgEnc - currEnc);
//				telemetry.update();
//				idle();
//			}
//			move(0.0,0.0);
//			try{pause(0.2);}catch(Exception e){}
//			stopMotors();
//		}
//	}

	public void moveWithEncodersCoastWithMaxTimeWithIncreasingDriftOrToBeacon(double speed, int goal, double maxTime, double leftMult, double rightMult, boolean isBlue) {

		double startTime = getCurTime();
		double currEncoder = getAvgEnc();
		double startEncoder = currEncoder;
		double power = speed;
		while(opModeIsActive() && currEncoder - startEncoder < goal && getCurTime() - startTime < maxTime) {
			currEncoder = getAvgEnc();
			double curMult = leftMult - (((((double)(currEncoder*currEncoder))/((double)(goal*goal)))*(((leftMult-rightMult))/(goal*goal))));
			if(speed > 0)
				moveForwards(speed,speed*curMult);
			else
				moveBackwards(speed, speed*curMult);
			initCurtime();
			idle();
		}
		if(speed > 0)
			moveBackwards(0.03);
		else
			moveForwards(0.03);
		try{pause(0.2);}catch(Exception e){}
		stopMotors();

	}

	public void moveWithEncodersCoastWithMaxTimeWithIncreasingDrift(double speed, int goal, double maxTime, boolean isBlue, double leftMult, double rightMult) {

		double startTime = getCurTime();
		double currEncoder = getAvgEnc();
		double startEncoder = currEncoder;
		double power = speed;
		String color = "Neither";
		if (colorB.beaconColor().equals("Blue"))
		{
			color = "Blue";
		}
		else if (colorB.beaconColor().equals("Red"))
		{
			color = "Red";
		}
		else
		{
			color = "Neither";
		}
		while(opModeIsActive() && currEncoder - startEncoder < goal && getCurTime() - startTime < maxTime) {
			currEncoder = getAvgEnc();
			double curMult = leftMult - (((((double)currEncoder)/((double)(goal)))*(((leftMult-rightMult))/(goal))));
			if(speed > 0)
				moveForwards(speed,speed*curMult);
			else
				moveBackwards(speed, speed*curMult);
			if(color.equals("Blue")&&isBlue)
			{
				goal = (int) (currEncoder + 200);
			}
			else if(color.equals("Red")&&!isBlue)
			{
				goal = (int) (currEncoder + 200);
			}
			initCurtime();
			idle();
		}
		try{pause(0.2);}catch(Exception e){}
		stopMotors();

	}

	public void moveWithEncodersCoastWithMaxTimeWithDriftAfterContact(double speed, int goal, double maxTime, double leftMult, double rightMult, int angleChangeGoal) {

		double startTime = getCurTime();
		double currEncoder = getAvgEnc();
		double startEncoder = currEncoder;
		double power = speed;
		double startAngle = gyro.getYaw();
		double currentAngle = startAngle;
		String color = "Neither";
		if (colorB.beaconColor().equals("Blue"))
		{
			color = "Blue";
		}
		else if (colorB.beaconColor().equals("Red"))
		{
			color = "Red";
		}
		else
		{
			color = "Neither";
		}
		while(opModeIsActive() && currEncoder - startEncoder < goal && getCurTime() - startTime < maxTime &&
				Math.abs(currentAngle - startAngle) < angleChangeGoal) {
			currentAngle = gyro.getYaw();
			currEncoder = getAvgEnc();
			if(speed > 0)
				moveForwards(speed,speed);
			else
				moveBackwards(speed, speed);
			idle();
		}

		while(opModeIsActive() && currEncoder - startEncoder < goal && getCurTime() - startTime < maxTime) {
			currEncoder = getAvgEnc();
			if(speed > 0)
				moveForwards(speed * leftMult,speed * rightMult);
			else
				moveBackwards(speed * leftMult, speed * rightMult);
			idle();
		}
		move(0,0);
		try{pause(0.2);}catch(Exception e){}
		stopMotors();

	}

	public void moveWithEncodersCoast(double speed, int goal, double leftMult, double rightMult) {
		if(opModeIsActive()) {
			int currEnc = getAvgEnc();
			int avgEnc = currEnc;
			if(speed > 0)
				moveForwards(speed * leftMult,speed * rightMult);
			else
				moveBackwards(speed * leftMult, speed * rightMult);
			int distMoved = Math.abs(avgEnc - currEnc);
			while (distMoved < goal && opModeIsActive()) {
				avgEnc = getAvgEnc();
				telemetry.addData("avg Enc", avgEnc);
				telemetry.addData("curr Enc", currEnc);
				move((-speed * leftMult * ((1 - (((double) distMoved / (double) goal) / 3)))), (speed * rightMult * ((1 - (((double) distMoved / (double) goal) / 3)))));
				distMoved = Math.abs(avgEnc - currEnc);
				telemetry.update();
				idle();
			}
			move(0.0,0.0);
			try{pause(0.2);}catch(Exception e){}
			stopMotors();
		}
	}

	public void moveWithEncodersCoastAndDropRollers(double speed, int goal, double leftMult, double rightMult) {
		if(opModeIsActive()) {
			int currEnc = getAvgEnc();
			int avgEnc = currEnc;
			if(speed > 0)
				moveForwards(speed * leftMult,speed * rightMult);
			else
				moveBackwards(speed * leftMult, speed * rightMult);
			int distMoved = Math.abs(avgEnc - currEnc);
			initCurtime();
			double startTimeForRollersDropping = getCurTime();
			while (getCurTime()-startTimeForRollersDropping < rollerMovementTimeDown && distMoved < goal && opModeIsActive()) {
				avgEnc = getAvgEnc();
				telemetry.addData("avg Enc", avgEnc);
				telemetry.addData("curr Enc", currEnc);
				distMoved = Math.abs(avgEnc - currEnc);
				telemetry.update();
				if(distMoved < goal)
					move((-speed * leftMult * ((1 - (((double) distMoved / (double) goal) / 3)))), (speed * rightMult * ((1 - (((double) distMoved / (double) goal) / 3)))));
				else
					move(0.0,0.0);

				if(getCurTime()-startTimeForRollersDropping < rollerMovementTimeDown)
					moveRollersDown();
				else
					stopRollers();
				idle();
			}
			move(0.0,0.0);
			try{pause(0.2);}catch(Exception e){}
			stopMotors();
		}
	}

	public void moveWithEncodersCoastExtra(double speed, int goal, double leftMult, double rightMult) {
		if(opModeIsActive()) {
			int currEnc = getAvgEnc();
			int avgEnc = currEnc;
			if(speed > 0)
				moveForwards(speed * leftMult,speed * rightMult);
			else
				moveBackwards(speed * leftMult, speed * rightMult);
			int distMoved = Math.abs(avgEnc - currEnc);
			double curSpeed = speed;
			while (distMoved < goal && opModeIsActive()) {
				avgEnc = getAvgEnc();
				telemetry.addData("avg Enc", avgEnc);
				telemetry.addData("curr Enc", currEnc);

				if(distMoved < goal/3)
					curSpeed = speed * (((double)distMoved) / (((double)goal)/3));
				else if(distMoved > ((2*goal)/3)) {
					double factor = -1 * (distMoved - goal);
					factor /= goal / 3;
					curSpeed = speed * factor;
				}
				else
					curSpeed = speed;
				if(Math.abs(curSpeed) > 1.0)
					if (speed < 0)
						curSpeed = -1.0;
					else
						curSpeed = 1.0;
				else if(Math.abs(curSpeed) < 0.13) {
					if (speed < 0)
						curSpeed = -0.13;
					else
						curSpeed = 0.13;
				}
				telemetry.addData("curSpeed",curSpeed);
				//move((-speed * leftMult * ((1 - (((double) distMoved / (double) goal) / 3)))), (speed * rightMult * ((1 - (((double) distMoved / (double) goal) / 3)))));
				move(leftMult*-curSpeed,rightMult*curSpeed);
				distMoved = Math.abs(avgEnc - currEnc);
				telemetry.update();
				idle();
			}
			move(0.0,0.0);
			try{pause(0.2);}catch(Exception e){}
			stopMotors();
		}
	}

	public void moveToWallWithRange(double speed, double goalUSDB, double goalUSDF, double goalDistance, double rMult, double lMult)
	{
		double USDB;
		double USDF;
		USDB = rangeB.getUltraSonicDistance();
		USDF = rangeF.getUltraSonicDistance();
		boolean goalFMet = false;
		boolean goalBMet = false;
		boolean goalDistMoved = false;
		int startEnc = getAvgEnc();
		while(!goalBMet && !goalFMet && !goalDistMoved && opModeIsActive())
		{
			USDB = rangeB.getUltraSonicDistance();
			USDF = rangeF.getUltraSonicDistance();
			moveBackwards(speed*lMult,speed*rMult);
			if (USDF<=goalUSDF)
			{
				goalFMet = true;
			}
			else if (USDB<=goalUSDB)
			{
				goalBMet = true;
			}
			else if (getAvgEnc()-startEnc>= goalDistance)
			{
				goalDistMoved = true;
			}
			idle();
		}
		move(0.0,0.0);
	}

	public void moveWithEncodersCoast(double speed, int goal) {
		if(opModeIsActive()) {
			int currEnc = getAvgEnc();
			int avgEnc = currEnc;
			if(speed > 0)
				moveForwards(speed,speed);
			else
				moveBackwards(speed,speed);
			int distMoved = Math.abs(avgEnc - currEnc);
			while (distMoved < goal && opModeIsActive()) {
				avgEnc = getAvgEnc();
				telemetry.addData("avg Enc", avgEnc);
				telemetry.addData("curr Enc", currEnc);
				move((-speed * ((1 - (((double) distMoved / (double) goal) / 2)))), (speed * ((1 - (((double) distMoved / (double) goal) / 2)))));
				distMoved = Math.abs(avgEnc - currEnc);
				telemetry.update();
				idle();
			}
		}
	}

	public void moveWithEncodersCoastExtra(double speed, int goal) {
		if(opModeIsActive()) {
			int currEnc = getAvgEnc();
			int avgEnc = currEnc;
			if(speed > 0)
				moveForwards(speed,speed);
			else
				moveBackwards(speed,speed);
			int distMoved = Math.abs(avgEnc - currEnc);
			while (distMoved < goal && opModeIsActive()) {
				avgEnc = getAvgEnc();
				telemetry.addData("avg Enc", avgEnc);
				telemetry.addData("curr Enc", currEnc);
				if(distMoved < goal/3)

				move((-speed * ((1 - (((double) distMoved / (double) goal) / 2)))), (speed * ((1 - (((double) distMoved / (double) goal) / 2)))));
				distMoved = Math.abs(avgEnc - currEnc);
				telemetry.update();
				idle();
			}
		}
	}

	public int getAvgEnc() {
		int encL1 = motorL1.getCurrentPosition();
		//telemetry.addData("encL1",encL1);
		int encL2 = motorL2.getCurrentPosition();
		//telemetry.addData("encL2",encL2);
		int encR1 = motorR1.getCurrentPosition();
		//telemetry.addData("encR1",encR1);
		int encR2 = motorR2.getCurrentPosition();
		//telemetry.addData("encR2",encR2);
		telemetry.addData("encL1",encL1);
		telemetry.addData("encL2",encL2);
		telemetry.addData("encR1",encR1);
		telemetry.addData("encR2",encR2);
		int avg = Math.abs(encL1) + Math.abs(encL2) + Math.abs(encR1) + Math.abs(encR2);
		avg /= 4;
		return avg;
	}

	@Override
	public void runOpMode() throws InterruptedException
	{
		initialize(); //Sets up motors, servos, and gyros
		motorL1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		motorL2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		motorR1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		motorR2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		motorSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		motorSpinner.setMaxSpeed(4500);
	}
}
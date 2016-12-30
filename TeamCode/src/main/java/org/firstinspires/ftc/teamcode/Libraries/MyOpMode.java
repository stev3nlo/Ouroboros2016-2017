package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
	protected double curPowerOfMotorSpinner = 0.9;

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
		closeServoDropper();
		servoBeaconPusher = hardwareMap.servo.get("servoBeaconPusher");
		moveBeaconPusherIn();
		servoWheelBeaconPusher = hardwareMap.servo.get("servoWheelBeaconPusher");
		pullInWheelBeaconPusher();



		curPowerOfMotorSpinner = 0.9;
		motorSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		//motorSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		//motorSpinner.setMaxSpeed((int)(((double)(encoderTicksPerRotation*goalRPM))/60.0));

		reset();
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
		rangeF = new SensorMRRange(hardwareMap.get(I2cDevice.class,"rangeF"));
		telemetry.addData("RangeF", "Initialized");
		telemetry.update();
		//pause(1.0);
		rangeB = new SensorMRRange(hardwareMap.get(I2cDevice.class,"rangeB"));
		telemetry.addData("RangeB", "Initialized");
		telemetry.update();
		//pause(1.0);
		//colorC.sensorSetup(0x2e);
		//colorR.sensorSetup(0x2a);
		colorB.sensorSetup(0x2c);
		telemetry.addData("ColorB", "I2C address");
		telemetry.update();
		//pause(1.0);
		rangeF.sensorSetup(0x4a);
		telemetry.addData("RangeF", "I2C address");
		telemetry.update();
		//pause(1.0);
		rangeB.sensorSetup(0x4c);
		telemetry.addData("RangeB", "I2C address");
		telemetry.update();
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
		motorL1.setPower(speedL);
		motorL2.setPower(speedL);
		motorR1.setPower(speedR);
		motorR2.setPower(speedR);
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
		setServoDropperPosition(.85);
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
			moveForwards(0.05,speed);
		else if(speed < 0)
			moveBackwards(0.05,speed);
	}

	public void arcTurnLeft(double speed) {
		if(speed > 0)
			moveForwards(speed,0.05);
		else if(speed < 0)
			moveBackwards(speed,0.05);
	}

	public void gyroArcTurnRight(double speed, double targetAngle) throws InterruptedException
	{
		double startAngle = gyro.getYaw();		//angle of the robot before the turn
		double newAngle = gyro.getYaw();		//angle of the robot during the turn
		arcTurnRight(speed);		//starts the turn
		while (opModeIsActive() && getAngleDiff(newAngle, startAngle) < targetAngle) {		//uses the abs value so this method can be used to turn the other way
			telemetry.addData("startAngle",startAngle);
			telemetry.addData("newAngle",newAngle);
			telemetry.update();
			newAngle = gyro.getYaw();		//updates the new angle constantly
			idle();
		}
		if(speed>0)
			moveBackwards(0.05);
		else
			moveForwards(0.05);
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
	 * This method will be used to turn to the left for when we are on the other side of the field
	 * <p>
	 *     This method uses the gyroTurnRight method and gives it the opposite speed to make it
	 *     turn to the left instead of the right.
	 * </p>
	 * @param speed
	 * @param targetAngle
	 */
	public void gyroTurnLeft(double speed, double targetAngle) throws InterruptedException {
		gyroTurnRight(-speed, -targetAngle);
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
				USDF = rangeF.getUltraSonicDistance();
				USDB = rangeB.getUltraSonicDistance();
				arcTurnRight(speed);
				idle();
			}
		}
		else
		{
			while(USDF>USDB && opModeIsActive())
			{
				USDF = rangeF.getUltraSonicDistance();
				USDB = rangeB.getUltraSonicDistance();
				arcTurnRight(speed);
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
				arcTurnLeft(speed);
				idle();
			}
		}
		else
		{
			while(USDF<USDB && opModeIsActive())
			{
				USDF = rangeF.getUltraSonicDistance();
				USDB = rangeB.getUltraSonicDistance();
				arcTurnLeft(speed);
				idle();

			}
		}
	}

	public void turnParallelToWall(double speed) {
		double USDF;
		double USDB;
		if (opModeIsActive()) {
			USDF = rangeF.getUltraSonicDistance();
			USDB = rangeB.getUltraSonicDistance();
			if (USDF > USDB) {
				turnRight(speed);
				while ((USDF > USDB) && opModeIsActive()) {
					telemetry.addData("turning","to wall");
					telemetry.addData("USDF",USDF);
					telemetry.addData("USDB",USDB);
					telemetry.update();
					USDF = rangeF.getUltraSonicDistance();
					USDB = rangeB.getUltraSonicDistance();
					idle();
				}
				stopMotors();
			} else {
				if (USDF < USDB) {
					turnLeft(speed);
					while ((USDF < USDB) && opModeIsActive()) {
						telemetry.addData("turning","to wall");
						telemetry.addData("USDF",USDF);
						telemetry.addData("USDB",USDB);
						telemetry.update();
						USDF = rangeF.getUltraSonicDistance();
						USDB = rangeB.getUltraSonicDistance();
						idle();
					}
					stopMotors();
				}
			}
		}
	}

	public String getCaseNameFromInfo(double USDF, double USDB, int targetDist, double thresholdW)
	{
		String name = "";
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

	public void stabilizeAlongWallWithRangeToBeacon(double speed, double thresholdA, double thresholdW, int targetDist, boolean isBlue)
	{
		double USDF;
		double USDB;
		String color = "Neither";
		if(opModeIsActive())
		{
			if(isBlue)
			{
				if(speed > 0)
				{
					while(!color.equals("Blue") && opModeIsActive())
					{
						if (colorB.beaconColor().equals("Blue")) {
							color = "Blue";
						} else if (colorB.beaconColor().equals("Red")) {
							color = "Red";
						} else {
							color = "Neither";
						}
						USDF = rangeF.getUltraSonicDistance();
						USDB = rangeB.getUltraSonicDistance();
						String caseName = getCaseNameFromInfo(USDF, USDB, targetDist, thresholdW);
						telemetry.addData("caseName",caseName);
						telemetry.update();
						switch (caseName) {
							case "00":
								moveForwards(speed, speed * 0.7);
								break;
							case "01":
								arcTurnRightToWall(-speed);
								break;
							case "02":
								turnParallelToWall(speed);
								break;
							case "10":
								arcTurnRightToWall(speed);
								break;
							case "11":
								moveForwards(speed*1.2,speed);
								break;
							case "12":
								arcTurnLeftToWall(speed);
								break;
							case "20":
								turnParallelToWall(speed);
								break;
							case "21":
								arcTurnLeftToWall(-speed);
								break;
							case "22":
								moveForwards(speed * 0.7, speed);
								break;
						}
						idle();
					}
				}
				else
				{
					while(!color.equals("Blue") && opModeIsActive()) {
						if (colorB.beaconColor().equals("Blue")) {
							color = "Blue";
						} else if (colorB.beaconColor().equals("Red")) {
							color = "Red";
						} else {
							color = "Neither";
						}
						USDF = rangeF.getUltraSonicDistance();
						USDB = rangeB.getUltraSonicDistance();
						String caseName = getCaseNameFromInfo(USDF, USDB, targetDist, thresholdW);
						telemetry.addData("caseName",caseName);
						telemetry.update();
						switch (caseName) {
							case "00":
								moveBackwards(speed, speed * 0.5);
								break;
							case "01":
								arcTurnRightToWall(-speed);
								break;
							case "02":
								turnParallelToWall(speed);
								break;
							case "10":
								arcTurnRightToWall(speed);
								break;
							case "11":
								moveBackwards(speed);
								break;
							case "12":
								arcTurnLeftToWall(speed);
								break;
							case "20":
								turnParallelToWall(speed);
								break;
							case "21":
								arcTurnLeftToWall(-speed);
								break;
							case "22":
								moveBackwards(speed * 0.7, speed);
								break;
						}
						idle();
					}
				}
			}
			else
			{
				if(speed > 0)
				{
					while(!color.equals("Red") && opModeIsActive())
					{
						if (colorB.beaconColor().equals("Blue")) {
							color = "Blue";
						} else if (colorB.beaconColor().equals("Red")) {
							color = "Red";
						} else {
							color = "Neither";
						}
						USDF = rangeF.getUltraSonicDistance();
						USDB = rangeB.getUltraSonicDistance();
						String caseName = getCaseNameFromInfo(USDF, USDB, targetDist, thresholdW);
						telemetry.addData("caseName",caseName);
						telemetry.update();
						switch (caseName) {
							case "00":
								moveForwards(speed, speed * 0.5);
								break;
							case "01":
								arcTurnRightToWall(-speed*1.7);
								break;
							case "02":
								turnParallelToWall(speed);
								break;
							case "10":
								arcTurnRightToWall(speed*1.7);
								break;
							case "11":
								moveForwards(speed);
								break;
							case "12":
								arcTurnLeftToWall(speed*1.7);
								break;

							case "20":
								turnParallelToWall(speed);
								break;
							case "21":
								arcTurnLeftToWall(-speed*1.7);
								break;
							case "22":
								moveForwards(speed * 0.5, speed);
								break;
						}
						idle();
					}
				}
				else
				{
					while(!color.equals("Red") && opModeIsActive()) {
						if (colorB.beaconColor().equals("Blue")) {
							color = "Blue";
						} else if (colorB.beaconColor().equals("Red")) {
							color = "Red";
						} else {
							color = "Neither";
						}
						USDF = rangeF.getUltraSonicDistance();
						USDB = rangeB.getUltraSonicDistance();
						String caseName = getCaseNameFromInfo(USDF, USDB, targetDist, thresholdW);
						telemetry.addData("caseName",caseName);
						telemetry.update();
						switch (caseName) {
							case "00":
								moveBackwards(speed, speed * 0.5);
								break;
							case "01":
								arcTurnRightToWall(-speed);
								break;
							case "02":
								turnParallelToWall(speed);
								break;
							case "10":
								arcTurnRightToWall(speed);
								break;
							case "11":
								moveBackwards(speed);
								break;
							case "12":
								arcTurnLeftToWall(speed);
								break;
							case "20":
								turnParallelToWall(speed);
								break;
							case "21":
								arcTurnLeftToWall(-speed);
								break;
							case "22":
								moveBackwards(speed * 0.5, speed);
								break;
						}
						idle();
					}
				}
			}
		}
		moveBackwards(0.05,0.05);
		telemetry.addData("saw the color","true");
	}

	public void stabilizeAlongWallWithRangeForEncoderDist(double speed, double thresholdA, double thresholdW, int targetDist, boolean isBlue, int encoderDist)
	{
		double USDF;
		double USDB;
		String color = "Neither";
		if(opModeIsActive())
		{
			if(isBlue)
			{
				if(speed > 0)
				{
					int currEnc = getAvgEnc();
					int avgEnc = currEnc;
					int distMoved = Math.abs(avgEnc - currEnc);
					while (distMoved < encoderDist && opModeIsActive()) {
						avgEnc = getAvgEnc();
						telemetry.addData("avg Enc", avgEnc);
						telemetry.addData("curr Enc", currEnc);
						distMoved = Math.abs(avgEnc - currEnc);
						USDF = rangeF.getUltraSonicDistance();
						USDB = rangeB.getUltraSonicDistance();
						String caseName = getCaseNameFromInfo(USDF, USDB, targetDist, thresholdW);
						switch (caseName) {
							case "00":
								moveForwards(speed, speed * 0.7);
								break;
							case "01":
								arcTurnRightToWall(-speed);
								break;
							case "02":
								turnParallelToWall(speed);
								break;
							case "10":
								arcTurnRightToWall(speed);
								break;
							case "11":
								moveForwards(speed);
								break;
							case "12":
								arcTurnLeftToWall(speed);
								break;
							case "20":
								turnParallelToWall(speed);
								break;
							case "21":
								arcTurnLeftToWall(-speed);
								break;
							case "22":
								moveForwards(speed * 0.7, speed);
								break;
						}
						idle();
					}
				}
				else
				{
					int currEnc = getAvgEnc();
					int avgEnc = currEnc;
					int distMoved = Math.abs(avgEnc - currEnc);
					while (distMoved < encoderDist && opModeIsActive()) {
						avgEnc = getAvgEnc();
						telemetry.addData("avg Enc", avgEnc);
						telemetry.addData("curr Enc", currEnc);
						distMoved = Math.abs(avgEnc - currEnc);
						USDF = rangeF.getUltraSonicDistance();
						USDB = rangeB.getUltraSonicDistance();
						String caseName = getCaseNameFromInfo(USDF, USDB, targetDist, thresholdW);
						telemetry.addData("caseName",caseName);
						telemetry.update();
						switch (caseName) {
							case "00":
								moveBackwards(speed, speed * 0.7);
								break;
							case "01":
								arcTurnRightToWall(-speed);
								break;
							case "02":
								turnParallelToWall(speed);
								break;
							case "10":
								arcTurnRightToWall(speed);
								break;
							case "11":
								moveBackwards(speed);
								break;
							case "12":
								arcTurnLeftToWall(speed);
								break;
							case "20":
								turnParallelToWall(speed);
								break;
							case "21":
								arcTurnLeftToWall(-speed);
								break;
							case "22":
								moveBackwards(speed * 0.7, speed);
								break;
						}
						idle();
					}
				}
			}
			else
			{
				if(speed > 0)
				{
					int currEnc = getAvgEnc();
					int avgEnc = currEnc;
					int distMoved = Math.abs(avgEnc - currEnc);
					while (distMoved < encoderDist && opModeIsActive()) {
						avgEnc = getAvgEnc();
						telemetry.addData("avg Enc", avgEnc);
						telemetry.addData("curr Enc", currEnc);
						distMoved = Math.abs(avgEnc - currEnc);
						USDF = rangeF.getUltraSonicDistance();
						USDB = rangeB.getUltraSonicDistance();
						String caseName = getCaseNameFromInfo(USDF, USDB, targetDist, thresholdW);
						telemetry.addData("caseName",caseName);
						telemetry.update();
						switch (caseName) {
							case "00":
								moveForwards(speed, speed * 0.5);
								break;
							case "01":
								arcTurnRightToWall(-speed*1.7);
								break;
							case "02":
								turnParallelToWall(speed);
								break;
							case "10":
								arcTurnRightToWall(speed*1.7);
								break;
							case "11":
								moveForwards(speed);
								break;
							case "12":
								arcTurnLeftToWall(speed*1.7);
								break;

							case "20":
								turnParallelToWall(speed);
								break;
							case "21":
								arcTurnLeftToWall(-speed*1.7);
								break;
							case "22":
								moveForwards(speed * 0.5, speed);
								break;
						}
						idle();
					}
				}
				else
				{
					int currEnc = getAvgEnc();
					int avgEnc = currEnc;
					int distMoved = Math.abs(avgEnc - currEnc);
					while (distMoved < encoderDist && opModeIsActive()) {
						avgEnc = getAvgEnc();
						telemetry.addData("avg Enc", avgEnc);
						telemetry.addData("curr Enc", currEnc);
						distMoved = Math.abs(avgEnc - currEnc);
						USDF = rangeF.getUltraSonicDistance();
						USDB = rangeB.getUltraSonicDistance();
						String caseName = getCaseNameFromInfo(USDF, USDB, targetDist, thresholdW);
						telemetry.addData("caseName",caseName);
						telemetry.update();
						switch (caseName) {
							case "00":
								moveBackwards(speed, speed * 0.5);
								break;
							case "01":
								arcTurnRightToWall(-speed);
								break;
							case "02":
								turnParallelToWall(speed);
								break;
							case "10":
								arcTurnRightToWall(speed);
								break;
							case "11":
								moveBackwards(speed);
								break;
							case "12":
								arcTurnLeftToWall(speed);
								break;
							case "20":
								turnParallelToWall(speed);
								break;
							case "21":
								arcTurnLeftToWall(-speed);
								break;
							case "22":
								moveBackwards(speed * 0.5, speed);
								break;
						}
						idle();
					}
				}
			}
		}
	}

	/*
	public void moveAlongWallToBeacon(double speed, double thresholdA, double thresholdW, int targetDist, boolean isBlue) {
		double USDF;
		double USDB;
		String color = "Neither";
		if (opModeIsActive()) {
			if (isBlue) {
				//moveForwards(speed);
				if(speed > 0)
				{
					boolean isArcTurningLeft = false;
					boolean isArcTurningRight = false;
					double speedL = 0.0;
					double speedR = 0.0;
					while ((!color.equals("Blue")) && opModeIsActive())
					{
						telemetry.addData("move Along wall to beacon blue","");
						telemetry.addData("color sensor Color", colorB.beaconColor());
						if (colorB.beaconColor().equals("Blue")) {
							color = "Blue";
						} else if (colorB.beaconColor().equals("Red")) {
							color = "Red";
						} else {
							color = "Neither";
						}
						USDF = rangeF.getUltraSonicDistance();
						USDB = rangeB.getUltraSonicDistance();
						telemetry.addData("range F", USDF);
						telemetry.addData("range B", USDB);
						double multiplier = -1.0;
						if(isArcTurningLeft)
						{
							if(USDF >= USDB) {
								isArcTurningLeft = false;
								speedL = 0.0;
								speedR = 0.0;
							}
							else
							{
								speedL = -speed;
								speedR = 0.0;
							}
						}
						else if(isArcTurningRight)
						{
							if(USDF <= USDB) {
								isArcTurningRight = false;
								speedL = 0.0;
								speedR = 0.0;
							}
							else {
								speedL = 0.0;
								speedR = speed;
							}
						}
						else if (USDB >= targetDist + thresholdW) //turn left, slow left side
						{
							multiplier = 1.0 - (((double) USDB - targetDist) / (targetDist / 3));
							if (multiplier < 0.3)
								multiplier = 0.3;
							speedL = speed * multiplier;
							speedR = -speed * 0.76;
						}
						else if (USDF >= targetDist + thresholdW) //turn right, slow right side
						{
							isArcTurningRight = true;
							speedL = -speed;
							speedR = 0.0;
						} else if (USDB <= targetDist - thresholdW) {
							multiplier = 1.0 - (((double) targetDist - USDB) / (targetDist / 3));
							if (multiplier < 0.3)
								multiplier = 0.3;
							speedL = speed;
							speedR = -speed * 0.76 * multiplier;
						} else if (USDF <= targetDist - thresholdW) //turn left, slow left side
						{
							isArcTurningLeft = true;
							speedL = 0.0;
							speedR = speed;
						} else if (USDF - USDB >= thresholdA) //turn right, slow right side
						{
							multiplier = 1.0 - (((double) USDF - USDB) / thresholdA * 2);
							if (multiplier < 0.3)
								multiplier = 0.3;
							speedL = speed;
							speedR = -speed * 0.76 * multiplier;
						} else if (USDB - USDF >= thresholdA) {
							multiplier = 1.0 - (((double) USDB - USDF) / thresholdA * 2);
							if (multiplier < 0.3)
								multiplier = 0.3;
							speedL = speed * multiplier;
							speedR = -speed * 0.76;
						} else {
							speedL = speed;
							speedR = -speed * 0.76;
						}
						move(speedL,speedR);
						telemetry.addData("speedL",speedL);
						telemetry.addData("speedR",speedR);
						if(isArcTurningLeft)
							telemetry.addData("isArcTurningLeft","true");
						else
							telemetry.addData("isArcTurningLeft","false");

						if(isArcTurningRight)
							telemetry.addData("isArcTurningRight","true");
						else
							telemetry.addData("isArcTurningRight","false");
						telemetry.addData("multiplier", multiplier);
						telemetry.update();
					}
					stopMotors();
				}
				else
				{
					boolean isArcTurningLeft = false;
					boolean isArcTurningRight = false;
					while ((!color.equals("Blue")) && opModeIsActive()) {
						telemetry.addData("move Along wall to beacon blue", "");
						telemetry.addData("color sensor Color", colorB.beaconColor());
						if (colorB.beaconColor().equals("Blue")) {
							color = "Blue";
						} else if (colorB.beaconColor().equals("Red")) {
							color = "Red";
						} else {
							color = "Neither";
						}

						USDF = rangeF.getUltraSonicDistance();
						USDB = rangeB.getUltraSonicDistance();
						telemetry.addData("range F", USDF);
						telemetry.addData("range B", USDB);
						telemetry.update();
						double multiplier = -1.0;
						if(isArcTurningLeft)
						{
							if(USDF <= USDB)
								isArcTurningLeft = false;
							else
								move(0, speed * 0.76);
						}
						else if(isArcTurningRight)
						{
							if(USDF <= USDB)
								isArcTurningRight = false;
							else
								move(-speed,0);
						}
						else if (USDF >= targetDist + thresholdW) //turn right, slow right side
						{
							multiplier = 1.0 - (((double) USDF - targetDist) / (targetDist / 3));
							if (multiplier < 0.3)
								multiplier = 0.3;
							move(speed, -speed * .76 * multiplier);
						} else if (USDB >= targetDist + thresholdW) //turn left, slow left side
						{
							isArcTurningLeft = true;
							move(0, speed * 0.76);
						} else if (USDF <= targetDist - thresholdW) //turn left, slow left side
						{
							multiplier = 1.0 - (((double) targetDist - USDF) / (targetDist / 3));
							if (multiplier < 0.3)
								multiplier = 0.3;
							move(speed * multiplier, -speed * 0.76);
						} else if (USDB <= targetDist - thresholdW) {
							isArcTurningRight = true;
							move(speed * 0.76,0);
						} else if (USDF - USDB >= thresholdA) //turn right, slow right side
						{
							multiplier = 1.0 - (((double) USDF - USDB) / thresholdA * 2);
							if (multiplier < 0.3)
								multiplier = 0.3;
							move(speed * multiplier, -speed * .76);
						} else if (USDB - USDF >= thresholdA) {
							multiplier = 1.0 - (((double) USDB - USDF) / thresholdA * 2);
							if (multiplier < 0.3)
								multiplier = 0.3;
							move(speed, -speed * 0.76 * multiplier);
						} else {
							move(speed, -speed * 0.76);
						}
						telemetry.addData("multiplier", multiplier);
						telemetry.update();
					}
					stopMotors();
				}
			} else {
				//moveForwards(speed);

				while ((!color.equals("Red")) && opModeIsActive()) {
					telemetry.addData("move Along wall to beacon red","");
					telemetry.addData("color sensor Color", colorB.beaconColor());
					if (colorB.beaconColor().equals("Blue")) {
						color = "Blue";
					} else if (colorB.beaconColor().equals("Red")) {
						color = "Red";
					} else {
						color = "Neither";
					}


					USDF = rangeF.getUltraSonicDistance();
					USDB = rangeB.getUltraSonicDistance();
					telemetry.addData("range F", USDF);
					telemetry.addData("range B", USDB);
					telemetry.update();

					double multiplier = -1.0;

					if(USDF >= targetDist + thresholdW) //turn right, slow right side
					{
						multiplier = 1.0 - (((double)USDF-targetDist)/(targetDist/3));
						if(multiplier<0.4)
							multiplier = 0.4;
						move(speed, -speed * .76 * multiplier);
					}
					else if (USDB >= targetDist + thresholdW) //turn left, slow left side
					{
						multiplier = 1.0 - (((double)USDB-targetDist)/(targetDist/3));
						if(multiplier < 0.4)
							multiplier = 0.4;
						move(speed*multiplier, -speed * 0.76 );
					}
					else if(USDF <= targetDist - thresholdW) //turn left, slow left side
					{
						multiplier = 1.0 - (((double)targetDist-USDF)/(targetDist/3));
						if(multiplier < 0.4)
							multiplier = 0.4;
						move(speed*multiplier, -speed * 0.76 );
					}
					else if(USDB <= targetDist - thresholdW)
					{
						multiplier = 1.0 - (((double)targetDist-USDB)/(targetDist/3));
						if(multiplier < 0.4)
							multiplier = 0.4;
						move(speed, -speed * 0.76 * multiplier);
					}
					else if(USDF - USDB >= thresholdA) //turn right, slow right side
					{
						multiplier = 1.0 - (((double)USDF-USDB)/thresholdA*5);
						if(multiplier < 0.4)
							multiplier = 0.4;
						move(speed, -speed * .76 * multiplier);
					}
					else if(USDB - USDF >= thresholdA)
					{
						multiplier = 1.0 - (((double)USDB-USDF)/thresholdA*5);
						if(multiplier < 0.4)
							multiplier = 0.4;
						move(speed*multiplier, -speed * 0.76 );
					}
					else
					{
						move(speed, -speed * 0.76);
					}
					telemetry.addData("multiplier",multiplier);
					telemetry.update();
				}
				stopMotors();
			}
		}
	}

	public void moveAlongWallForUnits(double speed, double thresholdA, double thresholdW, int targetDist, boolean isBlue, int goal) {
		double USDF;
		double USDB;
		String color = "Neither";
		if (opModeIsActive())
		{
			if (isBlue)
			{
				if(speed > 0)
				{
					int currEnc = getAvgEnc();
					int avgEnc = currEnc;
					move(speed, -speed * 0.76);
					int distMoved = Math.abs(avgEnc - currEnc);
					boolean isArcTurningLeft = false;
					boolean isArcTurningRight = false;
					while (distMoved < goal && opModeIsActive())
					{
						avgEnc = getAvgEnc();
						telemetry.addData("avg Enc", avgEnc);
						telemetry.addData("curr Enc", currEnc);
						distMoved = Math.abs(avgEnc - currEnc);

						USDF = rangeF.getUltraSonicDistance();
						USDB = rangeB.getUltraSonicDistance();
						telemetry.addData("range F", USDF);
						telemetry.addData("range B", USDB);
						double multiplier = -1.0;
						if(isArcTurningLeft)
						{
							if(USDF <= USDB)
								isArcTurningLeft = false;
							else
								move(0, speed * 0.76);
						}
						else if(isArcTurningRight)
						{
							if(USDF <= USDB)
								isArcTurningRight = false;
							else
								move(-speed,0);
						}
						else if (USDF >= targetDist + thresholdW) //turn right, slow right side
						{
							multiplier = 1.0 - (((double) USDF - targetDist) / (targetDist / 3));
							if (multiplier < 0.3)
								multiplier = 0.3;
							move(speed, -speed * .76 * multiplier);
						} else if (USDB >= targetDist + thresholdW) //turn left, slow left side
						{
							isArcTurningLeft = true;
							move(0, speed * 0.76);
						} else if (USDF <= targetDist - thresholdW) //turn left, slow left side
						{
							multiplier = 1.0 - (((double) targetDist - USDF) / (targetDist / 3));
							if (multiplier < 0.3)
								multiplier = 0.3;
							move(speed * multiplier, -speed * 0.76);
						} else if (USDB <= targetDist - thresholdW) {
							isArcTurningRight = true;
							move(speed * 0.76,0);
						} else if (USDF - USDB >= thresholdA) //turn right, slow right side
						{
							multiplier = 1.0 - (((double) USDF - USDB) / thresholdA * 2);
							if (multiplier < 0.3)
								multiplier = 0.3;
							move(speed * multiplier, -speed * .76);
						} else if (USDB - USDF >= thresholdA) {
							multiplier = 1.0 - (((double) USDB - USDF) / thresholdA * 2);
							if (multiplier < 0.3)
								multiplier = 0.3;
							move(speed, -speed * 0.76 * multiplier);
						} else {
							move(speed, -speed * 0.76);
						}
						telemetry.addData("multiplier", multiplier);
						telemetry.update();
					}
				}
				else
				{
					int currEnc = getAvgEnc();
					int avgEnc = currEnc;
					move(speed, -speed * 0.76);
					int distMoved = Math.abs(avgEnc - currEnc);
					boolean isArcTurningLeft = false;
					boolean isArcTurningRight = false;
					while (distMoved < goal && opModeIsActive())
					{
						avgEnc = getAvgEnc();
						telemetry.addData("avg Enc", avgEnc);
						telemetry.addData("curr Enc", currEnc);
						distMoved = Math.abs(avgEnc - currEnc);

						USDF = rangeF.getUltraSonicDistance();
						USDB = rangeB.getUltraSonicDistance();
						telemetry.addData("range F", USDF);
						telemetry.addData("range B", USDB);
						double multiplier = -1.0;
						if(isArcTurningLeft)
						{
							if(USDF <= USDB)
								isArcTurningLeft = false;
							else
								move(0, speed * 0.76);
						}
						else if(isArcTurningRight)
						{
							if(USDF <= USDB)
								isArcTurningRight = false;
							else
								move(-speed,0);
						}
						else if (USDF >= targetDist + thresholdW) //turn right, slow right side
						{
							multiplier = 1.0 - (((double) USDF - targetDist) / (targetDist / 3));
							if (multiplier < 0.3)
								multiplier = 0.3;
							move(speed, -speed * .76 * multiplier);
						} else if (USDB >= targetDist + thresholdW) //turn left, slow left side
						{
							isArcTurningLeft = true;
							move(0, speed * 0.76);
						} else if (USDF <= targetDist - thresholdW) //turn left, slow left side
						{
							multiplier = 1.0 - (((double) targetDist - USDF) / (targetDist / 3));
							if (multiplier < 0.3)
								multiplier = 0.3;
							move(speed * multiplier, -speed * 0.76);
						} else if (USDB <= targetDist - thresholdW) {
							isArcTurningRight = true;
							move(speed * 0.76,0);
						} else if (USDF - USDB >= thresholdA) //turn right, slow right side
						{
							multiplier = 1.0 - (((double) USDF - USDB) / thresholdA * 2);
							if (multiplier < 0.3)
								multiplier = 0.3;
							move(speed * multiplier, -speed * .76);
						} else if (USDB - USDF >= thresholdA) {
							multiplier = 1.0 - (((double) USDB - USDF) / thresholdA * 2);
							if (multiplier < 0.3)
								multiplier = 0.3;
							move(speed, -speed * 0.76 * multiplier);
						} else {
							move(speed, -speed * 0.76);
						}
						telemetry.addData("multiplier", multiplier);
						telemetry.update();
					}
				}
				stopMotors();
			}
			else
			{
				if(speed > 0)
				{
					//moveForwards(speed);
					while ((!color.equals("Red")) && opModeIsActive())
					{
						telemetry.addData("move Along wall to beacon red", "");
						telemetry.addData("color sensor Color", colorB.beaconColor());
						if (colorB.beaconColor().equals("Blue")) {
							color = "Blue";
						} else if (colorB.beaconColor().equals("Red")) {
							color = "Red";
						} else {
							color = "Neither";
						}


						USDF = rangeF.getUltraSonicDistance();
						USDB = rangeB.getUltraSonicDistance();
						telemetry.addData("range F", USDF);
						telemetry.addData("range B", USDB);
						telemetry.update();

						if(USDF >= targetDist + thresholdW)
						{
							double multiplier = 0.5 + (((double)USDF-targetDist)/(targetDist/2));
							move(speed*multiplier, -speed * .76);
						}
						else if (USDB >= targetDist + thresholdW)
						{
							double multiplier = 0.5 + (((double)USDB-targetDist)/(targetDist/2));
							move(speed, -speed * 0.76 * multiplier);
						}
						else if(USDF - USDB >= thresholdA) //turn right, slow right side
						{
							double multiplier = 0.7 + (((double)USDF-USDB)/thresholdA*5);
							move(speed, -speed * .76 * multiplier);
						}
						else if(USDB - USDF >= thresholdA)
						{
							double multiplier = 0.7 + (((double)USDB-USDF)/thresholdA*5);
							move(speed*multiplier, -speed * 0.76 );
						}
						else
						{
							move(speed, -speed * 0.76);
						}
					}
				}
			}
		}
		stopMotors();
	}
	*/

//	public void setServosParallel()
//	{
//		servoRangeB.setPosition(0.5);
//		servoRangeF.setPosition(0.5);
//	}

	//Methods to control button pushing
	public void pushButton() {
		moveBeaconPusherOut();
		try{pause(3.0);}catch(Exception e){}
		moveBeaconPusherIn();
	}


	public void moveBeaconPusherOut()
		{
		//TEST VALUES
		double v = 0.0;
		servoBeaconPusher.setPosition(v);
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
		while (it.hasNext()) {
			Map.Entry pair = (Map.Entry)it.next();
			return pair;
		}
		return null;
	}

	public HashMap<Double,Long> getFirstEncoderTimeSetAfterTime(double t)
	{
		HashMap<Double,Long> vals = new HashMap<Double,Long>();
		Iterator it = RPMs.entrySet().iterator();
		ArrayList<Double> toRemove = new ArrayList<Double>();
		while (it.hasNext()) {
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
		}
		while(toRemove.size()>0)
			RPMs.remove(toRemove.remove(0));
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

	public void moveWithEncoders(double speed, int goal) {
		if(opModeIsActive()) {
			int currEnc = getAvgEnc();
			int avgEnc = currEnc;
			if(speed > 0)
				moveForwards(speed*1.2,speed);
			else
				moveBackwards(speed*1.2,speed);
			int distMoved = Math.abs(avgEnc - currEnc);
			while (distMoved < goal && opModeIsActive()) {
				avgEnc = getAvgEnc();
				telemetry.addData("avg Enc", avgEnc);
				telemetry.addData("curr Enc", currEnc);
				//move((-speed * ((1 - (((double) distMoved / (double) goal) / 2)))), (speed * ((1 - (((double) distMoved / (double) goal) / 2)))*.7));
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

	}
}
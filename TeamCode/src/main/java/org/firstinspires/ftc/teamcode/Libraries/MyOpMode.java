package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * @author 		Steven Lo
 * @version 	1.0			(I don't know what this really is)
 * @since 		1.0 		(IDK about this either)
 * @date 		9/21/2016.
 */
public abstract class MyOpMode extends LinearOpMode {

	//drive train motors
	/**
	 * DcMotor variable for the first motor on the right
	 */
	DcMotor R1;
	/**
	 * DcMotor variable for the second motor on the right
	 */
	DcMotor R2;
	/**
	 * DcMotor variable for the first motor on the left
	 */
	DcMotor L1;
	/**
	 * DcMotor variable for the second motor on the left
	 */
	DcMotor L2;

	DcMotor motorSpinner;


	//sensors
	/**
	 * Adafruit IMU Object
	 */
	SensorAdafruitIMU gyro;

	/**
	 * First Modern Robotics Color Sensor object
	 */
	SensorMRColor color1;
	/**
	 * Second Modern Robotics Color Sensor object
	 */
	SensorMRColor color2;

	//color values for the white line
	public static int redValue = 0;
	public static int greenValue = 0;
	public static int blueValue = 0;
	public static int alphaValue = 0;

	//constructor
	public MyOpMode() {
		super();
	}

	//hardware maps and sets initial values

	/**
	 * Hardware maps and initializes motors and sensors.
	 * <p>
	 *     Sets up all the motors and sensors and sets their initial values
	 * </p>
	 */
	public void initialize() {
		//hardware maps the drive motors
		R1 = hardwareMap.dcMotor.get("R1");
		R2 = hardwareMap.dcMotor.get("R2");
		L1 = hardwareMap.dcMotor.get("L1");
		L2 = hardwareMap.dcMotor.get("L2");

		//initialize sensors
		gyro = new SensorAdafruitIMU();
		color1 = new SensorMRColor();
		color2 = new SensorMRColor();

		reset();
	}

	//method used for any base movement

	/**
	 * Moves all the drive motors with given speed for the left and right sides
	 * separately. This is the base method used for all movement with the base,
	 * i.e. moving and backwards and turning left and right.
	 *
	 * @param speedR
	 * @param speedL
	 */
	public void move(double speedR, double speedL) {
		R1.setPower(speedR);
		R2.setPower(speedR);
		L1.setPower(speedL);
		L2.setPower(speedL);
	}

	/**
	 * Stops all of the motors.
	 * <p>
	 *     This method is used to stop the drive motors.
	 * </p>
	 */
	public void stopMotors() {
		R1.setPower(0);
		R2.setPower(0);
		L1.setPower(0);
		L2.setPower(0);
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
		move(speed, speed);
	}

	/**
	 * Sets the speed of the motors controlling the wheels on the left and right side of
	 * the robot to the same value so it moves backwards
	 * <p>
	 *     Calls the moveForward method and gives an opposite speed
	 * </p>
	 * @param speed
	 */
	public void moveBackwards(double speed) {
		moveForwards(-speed);
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
		move(-speed, speed);
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
		turnRight(-speed);
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
	public void gyroTurnRight(double speed, double targetAngle) {
		double startAngle = gyro.getYaw();		//angle of the robot before the turn
		double newAngle = gyro.getYaw();		//angle of the robot during the turn
		turnRight(speed);		//starts the turn
		while (Math.abs(newAngle - startAngle) < targetAngle) {		//uses the abs value so this method can be used to turn the other way
			newAngle = gyro.getYaw();		//updates the new angle constantly
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
	public void gyroTurnLeft(double speed, double targetAngle) {
		gyroTurnRight(-speed, targetAngle);
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
	public void moveToWhiteLine(double speed) {
		int red = color1.getRed();
		int green = color1.getGreen();
		int blue = color1.getBlue();
		int alpha = color1.getAlpha();

		boolean isWhite = false;
		moveForwards(speed);
		while (!isWhite) {
			if ((red <= redValue) && (green <= greenValue) && (blue <= blueValue) && (alpha <= alphaValue)) {	//need to test values for white
				isWhite = true;
			}
			red = color1.getRed();
			green = color1.getGreen();
			blue = color1.getBlue();
			alpha = color1.getAlpha();
		}
		stopMotors();
	}

	/**
	 * This method will align the robot with the white line with the second color sensor.
	 * <p>
	 *     This method will continuously turn right until the second color sensor gets values that
	 *     indicate that it's over the white line, meaning both of the color sensors will now be
	 *     over the white line, aligning the robot up to the beacon.
	 * </p>
	 * @param speed
	 */
	public void turnRightToWhiteLine(double speed) {
		int red = color2.getRed();
		int green = color2.getGreen();
		int blue = color2.getBlue();
		int alpha = color2.getAlpha();

		boolean isWhite = false;
		turnRight(speed);
		while (!isWhite) {
			if ((red <= redValue) && (green <= greenValue) && (blue <= blueValue) && (alpha <= alphaValue)) {
				isWhite = true;
			}
			red = color2.getRed();
			green = color2.getGreen();
			blue = color2.getBlue();
			alpha = color2.getAlpha();
		}
		stopMotors();
	}

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
	public void turnLeftToWhiteLine(double speed) {
		turnRightToWhiteLine(-speed);
	}

	/**
	 * This method will reset all the values to the default (stopped, initial positions)
	 */
	public void reset() {
		R1.setPower(0);
		R2.setPower(0);
		L1.setPower(0);
		L2.setPower(0);
	}
	@Override
	public void runOpMode()
	{
		initialize(); //Sets up motors, servos, and gyros
		/**if(isRedSide)
			basketInitRed();
		else
			basketInitBlue();
		 */
	}
}

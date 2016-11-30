package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


import java.util.Locale;



/**
 * FTC4545 implementation for the {@link SensorAdafruitIMU}.
 *
 * Has basic getOrientation, getYaw, getRoll, and getPitch methods, as well as getAccelVectors, getXAccel, getYAccel, and getZAccel
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
public class SensorAdafruitIMU
{
    // The IMU sensor object
    BNO055IMU imu;
	BNO055IMU.Parameters parameters;

    // State used for updating telemetry
    Orientation angles;
    Acceleration accels;

    public SensorAdafruitIMU()
    {
        //imu = hardwareMap.get(BNO055IMU.class, "imu");
    }

	public SensorAdafruitIMU(BNO055IMU gyro) {
		imu = gyro;
        initializeParams();
	}

	public void initializeParams() {
		parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
		parameters.loggingEnabled = true;
		parameters.loggingTag = "Gyro";
		parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

		imu.initialize(parameters);
    }

    public double[] getOrientation()
    {
        double[] yawrollpitch = new double[3];
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        yawrollpitch[0] = angles.firstAngle;
        yawrollpitch[1] = angles.secondAngle;
        yawrollpitch[2] = angles.thirdAngle;
        return yawrollpitch;
    }

    public double getYaw()
    {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);

        return -1*angles.firstAngle;
    }

    public double getRoll()
    {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return angles.secondAngle;
    }

    public double getPitch()
    {

        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return angles.thirdAngle;
    }

    public double[] getAcceleration()
    {

        double[] accelRay = new double[3];
        accels = imu.getLinearAcceleration();
        accelRay[0] = accels.xAccel;
        accelRay[1] = accels.yAccel;
        accelRay[2] = accels.zAccel;
        return accelRay;
    }

    public double getXAccel()
    {
        initializeParams();accels = imu.getLinearAcceleration();
        return accels.xAccel;
    }

    public double getYAccel()
    {

        accels = imu.getLinearAcceleration();
        return accels.yAccel;
    }

    public double getZAccel()
    {

        accels = imu.getLinearAcceleration();
        return accels.xAccel;
    }

	public void resetGyro() {
		imu.initialize(parameters);
	}
}
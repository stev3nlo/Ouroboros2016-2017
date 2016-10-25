package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Steven on 10/20/2016.
 */

public class SensorMRRange extends MyOpMode {

	I2cDevice rangeSensor;

	public SensorMRRange(I2cDevice rangeSensor) {
		this.rangeSensor = rangeSensor;
	}

	public double getDistance() {
		//	return rangeSensor.getDistance(DistanceUnit.INCH);
		return 0;
	}
}
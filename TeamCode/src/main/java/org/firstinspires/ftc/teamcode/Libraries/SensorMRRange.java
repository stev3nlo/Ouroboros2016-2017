package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Steven on 10/20/2016.
 */

public class SensorMRRange extends MyOpMode {

	ModernRoboticsI2cRangeSensor rangeSensor;

	public SensorMRRange(ModernRoboticsI2cRangeSensor rangeSensor) {
		this.rangeSensor = rangeSensor;
	}

	public double getDistance() {
		return rangeSensor.getDistance(DistanceUnit.INCH);
	}
}
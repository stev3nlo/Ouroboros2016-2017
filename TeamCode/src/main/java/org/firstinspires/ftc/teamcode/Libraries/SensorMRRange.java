package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

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
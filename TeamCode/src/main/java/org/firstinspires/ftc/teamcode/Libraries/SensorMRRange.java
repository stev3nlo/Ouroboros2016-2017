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
	I2cAddr rangeAddress = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
	int rangeRegisterStart = 0x04; //Register to start reading
	int rangeArrayLength = 2; //Number of byte to read
	byte[] rangeCache; //The read will return an array of bytes.
	I2cDeviceSynch rangeReader;


	public SensorMRRange(I2cDevice rangeSensor) {
		this.rangeSensor = rangeSensor;
		rangeReader = new I2cDeviceSynchImpl(rangeSensor, rangeAddress, false);
		rangeReader.engage();
		rangeCache = rangeReader.read(rangeRegisterStart, rangeArrayLength);
	}

	public double getOpticDistance() {
		return (double) (rangeCache[1] & 0xFF);
	}

	public double getUltraSonicDistance(){
		return (double) (rangeCache[0] & 0xFF);
	}
}
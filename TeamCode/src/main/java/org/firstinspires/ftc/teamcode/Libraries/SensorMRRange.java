package org.firstinspires.ftc.teamcode.Libraries;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.ArrayList;

/**
 * Created by Steven on 10/20/2016.
 */


public class SensorMRRange extends MyOpMode {

	private ElapsedTime runtime = new ElapsedTime();
	byte[] rangeCache; //The read will return an array of bytes.

	I2cAddr rangeAddress = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
	public static final int RANGE_REG_START = 0x04; //Register to start reading
	public static final int RANGE_READ_LENGTH = 2; //Number of byte to read

	I2cDevice rangeSensor;
	I2cDeviceSynch rangeReader;

	public SensorMRRange(I2cDevice rangeSensor) {
		this.rangeSensor = rangeSensor;
		initReader();
	}

	public void initReader() {
		rangeReader = new I2cDeviceSynchImpl(rangeSensor, rangeAddress, false);
		rangeReader.engage();
	}

	public int getUltraSonicDistance() {
		return rangeCache[0] & 0xFF;
	}

	public int getOpticDistance() {
		return rangeCache[1] & 0xFF;
	}

	public String getRunTime() {
		return runtime.toString();
	}

	public ArrayList<String> getRangeCache() {
		rangeCache = rangeReader.read(RANGE_REG_START, RANGE_READ_LENGTH);
		ArrayList output = new ArrayList<String>();

		output.add(rangeCache[0] & 0xFF);
		output.add(rangeCache[1] & 0xFF);
		output.add("Run Time: " + runtime.toString());

		return output;
	}

	@Override
	public String toString() {
		ArrayList<String> cache = getRangeCache();
		String output = "";

		output += "Ultra Sonic" + cache.get(0);
		output += "\nODS" + cache.get(1);
		output += "\nStatus: " + cache.get(2);

		return output;
	}
}
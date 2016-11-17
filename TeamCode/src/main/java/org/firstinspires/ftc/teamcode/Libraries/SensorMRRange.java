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

	public int prevUSDVal;
	private ElapsedTime runtime = new ElapsedTime();
	byte[] rangeCache; //The read will return an array of bytes.

	I2cAddr rangeAddress = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
	public static final int RANGE_REG_START = 0x04; //Register to start reading
	public static final int RANGE_READ_LENGTH = 2; //Number of byte to read

	I2cDevice rangeSensor;
	I2cDeviceSynch rangeReader;

	int ultraSonicDistanceValue;

	public SensorMRRange(I2cDevice rangeSensor) {
		this.rangeSensor = rangeSensor;
		initReader();
		ultraSonicDistanceValue = -1;
		prevUSDVal = ultraSonicDistanceValue;
	}

	public void initReader() {
		rangeReader = new I2cDeviceSynchImpl(rangeSensor, rangeAddress, false);
		rangeReader.engage();
	}

	public void sensorSetup(int i) {
		rangeReader.setI2cAddress(I2cAddr.create8bit(i));
	}

	public int getRawUltraSonicDistance() {
		return rangeCache[0] & 0xFF;
	}

	public int getUltraSonicDistance() {
		int usd = rangeCache[0] & 0xFF;
		if (usd == 255) {
			return prevUSDVal;
		} else {
			prevUSDVal = usd;
			return usd;
		}
	}

	//
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

	public boolean inFrontOfBeacon() {
		if (getUltraSonicDistance() < 8) {		//value needs to be tested/calculated
			return true;
		}
		return false;
	}

	@Override
	public String toString() {
		ArrayList<String> cache = getRangeCache();
		String output = "";

		//output += "Ultra Sonic " + String.valueOf(cache.get(0));
		output += "\nRaw USD: " + getRawUltraSonicDistance() + "\n";
		output += "Ultra Sonic: " + ultraSonicDistanceValue;
		output += "\nODS: " + String.valueOf(cache.get(1));
		output += "\nStatus: " + String.valueOf(cache.get(2));

		return output;
	}
}
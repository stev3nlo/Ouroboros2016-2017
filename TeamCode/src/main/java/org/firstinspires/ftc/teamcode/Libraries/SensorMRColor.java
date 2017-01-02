package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.Hardware;

/**
 * Created by Steven on 9/20/2016.
 */
public class SensorMRColor {

	//color sensor object
	ColorSensor RGB;

	public int grayAlpha;

	public SensorMRColor(ColorSensor colorSensor) {
		//Initialize color sensor
		RGB = colorSensor;
		grayAlpha = getGrayAlpha();
	}

	//returns the R, G, and B values that the color sensor senses in a list
	public int[] getColor() {
		int[] color = new int[3];
		color[0] = getRed();
		color[1] = getBlue();
		color[2] = getGreen();
		return color;
	}

	//gets the RGB and alpha values of the color sensor
	public int[] getColorAndAlpha() {
		int[] color = new int[4];
		int[] rgb = new int[3];
		rgb = getColor();
		color[0] = rgb[0];
		color[1] = rgb[1];
		color[2] = rgb[2];
		color[3] = getAlpha();
		return color;
	}

	//converts RGB values to HSV (hue-saturation-value)
	//(method from stackoverflow.com/questions/2399150/convert-rgb-value-to-hsv)
	public double[] RGBtoHSV(double r, double g, double b) {

		double h, s, v;
		double min, max, delta;

		min = Math.min(Math.min(r, g), b);
		max = Math.max(Math.max(r, g), b);

		// V
		v = max;
		delta = max - min;

		// S
		if( max != 0 )
			s = delta / max;
		else {
			s = 0;
			h = -1;
			return new double[]{h,s,v};
		}

		// H
		if( r == max )
			h = ( g - b ) / delta; // between yellow & magenta
		else if( g == max )
			h = 2 + ( b - r ) / delta; // between cyan & yellow
		else
			h = 4 + ( r - g ) / delta; // between magenta & cyan

		h *= 60;    // degrees

		if( h < 0 )
			h += 360;

		return new double[]{h,s,v};
	}

	//gets the alpha value (level of whiteness)
	public int getAlpha() {
		if (RGB.alpha() == 255) {
			return grayAlpha;
		}
		return RGB.alpha();
	}

	public int getGrayAlpha() {
		return getAlpha();
	}

	//gets the red value
	public int getRed() {
		return RGB.red();
	}

	//gets the blue value
	public int getBlue() {
		return RGB.blue();
	}

	//gets the green value
	public int getGreen() {
		return RGB.green();
	}

	//returns the approximate color
	public String getApproxColor() {
		String[] colors = new String[4];
		colors[0] = "Gray/Other";
		colors[1] = "White";
		colors[2] = "Red";
		colors[3] = "Blue";

		if (getRed() > 100) { //needs to be tested
			return colors[2];
		} else if (getBlue() > 100) { //needs to be tested
			return colors[3];
		} else if (getAlpha() > 100) {
			return colors[1];
		} else {
			return colors[0];
		}
	}

	//returns whether the color of the ground is Gray or White
	public String groundColor() {
		if ((getAlpha() - grayAlpha) > 10) {	//need to retest value3
			return "White";
		} else
			return "Gray";
	}

	public boolean onWhite() {
		if (groundColor().equals("White")) {
			return true;
		}
		return false;
	}

	//returns whether the color of the beacon is Red or Blue
	public String beaconColor() {
		if (getBlue() >= 1 && getBlue() > getRed()){// && getBlue() > getRed()) {
			return "Blue";
		} else if (getRed() >= 2 && getRed() > getBlue()) {
			return "Red";
		} else {
			return "Neither";
		}
	}

	public void sensorSetup(int i) {
		RGB.setI2cAddress(I2cAddr.create8bit(i));
	}

	public void lightOff() {
		RGB.enableLed(false);
	}

	public void lightOn() {
		RGB.enableLed(true);
	}

	//formats the output with all the color values and the alpha value
	@Override
	public String toString() {
		String output = "Red: " + getRed() + "\n" + "Green: " + getGreen() + "\n" + "Blue: " + getBlue() +
				"\nAlpha: " + getAlpha();
		return output;
	}
}
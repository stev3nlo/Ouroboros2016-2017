package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Hardware;

/**
 * Created by Steven on 9/20/2016.
 */
public class SensorMRColor extends MyOpMode {

	//color sensor object
	ColorSensor RGB;

	public SensorMRColor(ColorSensor colorSensor) {

		//Initialize color sensor
		RGB = colorSensor;
	}

	public SensorMRColor(String name) {
		RGB = hardwareMap.colorSensor.get(name);
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
	public double[] RGBtoHSV(double r, double g, double b){

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
		return RGB.alpha();
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
}

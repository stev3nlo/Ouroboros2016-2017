package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Steven on 9/20/2016.
 */
public class SensorAdafruitRGB {

	//color sensor object
	ColorSensor RGB;

	//returns the R, G, and B values that the color sensor senses in a list
	public int[] getColor() {
		int[] color = new int[3];
		color[0] = RGB.red();
		color[1] = RGB.blue();
		color[2] = RGB.green();
		return color;
	}

	public int[] getColorAndAlpha() {
		int[] color = new int[4];
		color[0] = RGB.red();
		color[1] = RGB.blue();
		color[2] = RGB.green();
		color[3] = RGB.alpha();
		return color;
	}

	//converts RGB to HSV values
	public int[] convertRGBToHSV(int[] RGBColor) {
		int[] HSVColor = new int[3];
		HSVColor[0] = RGBColor[0] * 255 / 800;
		HSVColor[1] = RGBColor[1] * 255 / 800;
		HSVColor[2] = RGBColor[2] * 255 / 800;
		return HSVColor;
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

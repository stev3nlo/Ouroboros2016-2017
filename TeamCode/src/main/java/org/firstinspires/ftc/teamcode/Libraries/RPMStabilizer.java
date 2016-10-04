package org.firstinspires.ftc.teamcode.Libraries;

/**
 * Created by spencersharp on 9/27/16.
 */
public class RPMStabilizer
{
    private RPMStabilizer()
    {

    }

    public static double returnPowerToTry(double curPower, double curRPM, double goalRPM)
    {
        double scaledPower = MotorScaler.reverseScale(curPower); //Estimates actual power %
        double maxRPM = curRPM / scaledPower; //Estimates maximum RPM
        double percentage = goalRPM / maxRPM; //Percentage of total speed the motor should be run at
        double newPower = MotorScaler.scaleSimple(percentage); //
        return newPower;
    }
}

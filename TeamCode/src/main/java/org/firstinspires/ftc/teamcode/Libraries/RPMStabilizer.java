package org.firstinspires.ftc.teamcode.Libraries;

/**
 * Created by spencersharp on 9/27/16.
 */
public class RPMStabilizer
{
    private RPMStabilizer()
    {

    }

    public double returnPowerToTry(double curPower, double curRPM, double goalRPM)
    {
        double scaledPower = MotorScaler.reverseScale(curPower);
        double maxRPM = curRPM / scaledPower;
        double percentage = goalRPM / maxRPM;
        double newPower = MotorScaler.scaleSimple(percentage);
        return newPower;
    }
}

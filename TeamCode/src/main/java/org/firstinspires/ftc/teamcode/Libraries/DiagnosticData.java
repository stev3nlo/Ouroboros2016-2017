package org.firstinspires.ftc.teamcode.Libraries;

import java.io.Serializable;

/**
 * Created by spencersharp on 1/7/17.
 */
public class DiagnosticData implements Serializable
{
    protected String motorName;
    protected double batteryVoltage;
    protected double powerSent;
    protected double RPM;

    public DiagnosticData(String motorName, double batteryVoltage, double powerSent, double RPM)
    {
        this.motorName = motorName;
        this.batteryVoltage = batteryVoltage;
        this.powerSent = powerSent;
        this.RPM = RPM;
    }

    public String getMotorName()
    {
        return motorName;
    }

    protected double getBatteryVoltage()
    {
        return batteryVoltage;
    }

    protected double getPowerSent()
    {
        return powerSent;
    }

    protected double RPM()
    {
        return RPM;
    }

    public String toString()
    {
        return motorName + " " + batteryVoltage + " " + powerSent + " " + RPM;
    }
}

package org.firstinspires.ftc.teamcode.Libraries;

import java.io.Serializable;

/**
 * Created by spencersharp on 1/7/17.
 */
public class DiagnosticData implements Serializable
{
    protected String motorName;
    protected int batteryVoltage;
    protected double powerSent;
    protected double RPM;

    public DiagnosticData()
    {

    }

    public DiagnosticData(String motorName, int batteryVoltage, double powerSent, double RPM)
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

    public int getBatteryVoltage()
    {
        return batteryVoltage;
    }

    public double getPowerSent()
    {
        return powerSent;
    }

    public double RPM()
    {
        return RPM;
    }

    public String toString()
    {
        return motorName + " " + batteryVoltage + " " + powerSent + " " + RPM;
    }
}

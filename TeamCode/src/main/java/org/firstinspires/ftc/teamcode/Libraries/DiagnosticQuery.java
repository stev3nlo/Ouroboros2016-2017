package org.firstinspires.ftc.teamcode.Libraries;

import java.io.Serializable;

public class DiagnosticQuery implements Serializable
{
    protected String motorName;
    protected int batteryVoltage;
    protected double goalRPM;

    public DiagnosticQuery()
    {

    }

    public DiagnosticQuery(String motorName, int batteryVoltage, double goalRPM)
    {
        this.motorName = motorName;
        this.batteryVoltage = batteryVoltage;
        this.goalRPM = goalRPM;
    }

    public String getMotorName()
    {
        return motorName;
    }

    public int getBatteryVoltage()
    {
        return batteryVoltage;
    }

    public double getGoalRPM()
    {
        return goalRPM;
    }
}
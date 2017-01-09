package org.firstinspires.ftc.teamcode.Libraries;

import java.io.Serializable;
import java.util.ArrayList;

/**
 * Created by spencersharp on 1/7/17.
 */
public class DiagnosticMotorInfo implements Serializable
{
    public String motorName;
    public ArrayList<DiagnosticData> recordedData;

    public String getMotorName()
    {
        return motorName;
    }

    public void add(DiagnosticData d)
    {

    }


}

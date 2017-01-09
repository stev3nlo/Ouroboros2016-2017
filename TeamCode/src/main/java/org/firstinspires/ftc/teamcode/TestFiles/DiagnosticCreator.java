package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.DiagnosticData;
import org.firstinspires.ftc.teamcode.Libraries.DiagnosticLibrary;
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

import java.io.IOException;
import java.util.ArrayList;

/**
 * Created by spencersharp on 1/7/17.
 */
@TeleOp(name = "DiagnosticCreator", group = "Test")
public class DiagnosticCreator extends MyOpMode
{
    long motorL1StartEncoder;
    long motorL2StartEncoder;
    long motorR1StartEncoder;
    long motorR2StartEncoder;
    long motorSpinnerStartEncoder;
    double startTimeOfLoop;
    DiagnosticLibrary dl;

    public void runOpMode() throws InterruptedException
    {
        dl = new DiagnosticLibrary();
        double batteryLevel = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        while(batteryLevel>9.5)
        {
            double curSpeed = 1.0;
            while(curSpeed > 0.0)
            {
                motorL1.setPower(curSpeed);
                motorL2.setPower(curSpeed);
                motorR1.setPower(curSpeed);
                motorR2.setPower(curSpeed);
                motorSpinner.setPower(curSpeed);
                pause(0.5);
                batteryLevel = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
                initCurtime();
                startTimeOfLoop = getCurTime();
                motorL1StartEncoder = motorL1.getCurrentPosition();
                motorL2StartEncoder = motorL2.getCurrentPosition();
                motorR1StartEncoder = motorR1.getCurrentPosition();
                motorR2StartEncoder = motorR2.getCurrentPosition();
                motorSpinnerStartEncoder = motorSpinner.getCurrentPosition();
                pause(2.0);
                ArrayList<DiagnosticData> diagnosticData = new ArrayList<DiagnosticData>();
                diagnosticData.add(new DiagnosticData("motorL1",batteryLevel,curSpeed,calculateRPM("motorL1")));
                diagnosticData.add(new DiagnosticData("motorL2",batteryLevel,curSpeed,calculateRPM("motorL2")));
                diagnosticData.add(new DiagnosticData("motorR1",batteryLevel,curSpeed,calculateRPM("motorR1")));
                diagnosticData.add(new DiagnosticData("motorR2",batteryLevel,curSpeed,calculateRPM("motorR2")));
                diagnosticData.add(new DiagnosticData("motorSpinner",batteryLevel,curSpeed,calculateRPM("motorSpinner")));
                addAllToLibrary(diagnosticData);
                stopMotors();
                pause(5.0);
                curSpeed -= 0.05;
            }
            while(curSpeed >= -1.0)
            {
                motorL1.setPower(curSpeed);
                motorL2.setPower(curSpeed);
                motorR1.setPower(curSpeed);
                motorR2.setPower(curSpeed);
                pause(0.5);
                batteryLevel = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
                initCurtime();
                startTimeOfLoop = getCurTime();
                motorL1StartEncoder = motorL1.getCurrentPosition();
                motorL2StartEncoder = motorL2.getCurrentPosition();
                motorR1StartEncoder = motorR1.getCurrentPosition();
                motorR2StartEncoder = motorR2.getCurrentPosition();

                pause(2.0);
                ArrayList<DiagnosticData> diagnosticData = new ArrayList<DiagnosticData>();
                diagnosticData.add(new DiagnosticData("motorL1",batteryLevel,curSpeed,calculateRPM("motorL1")));
                diagnosticData.add(new DiagnosticData("motorL2",batteryLevel,curSpeed,calculateRPM("motorL2")));
                diagnosticData.add(new DiagnosticData("motorR1",batteryLevel,curSpeed,calculateRPM("motorR1")));
                diagnosticData.add(new DiagnosticData("motorR2",batteryLevel,curSpeed,calculateRPM("motorR2")));
                addAllToLibrary(diagnosticData);
                stopMotors();
                pause(2.0);
                curSpeed -= 0.05;
            }
        }
        try {
            dl.saveDiagnosticLibrary();
        }
        catch(IOException ioe)
        {

        }
    }

    public void addAllToLibrary(ArrayList<DiagnosticData> diagnosticData)
    {
        for(DiagnosticData d : diagnosticData)
            dl.addDiagnosticData(d);
    }

    public double calculateRPM(String motorName)
    {
        double startEncoder;
        double curEncoder;
        double timePassed;
        if(motorName.equals("motorL1"))
        {
            startEncoder = motorL1StartEncoder;
            curEncoder = motorL1.getCurrentPosition();
        }
        else if(motorName.equals("motorL2"))
        {
            startEncoder = motorL2StartEncoder;
            curEncoder = motorL2.getCurrentPosition();
        }
        else if(motorName.equals("motorR1"))
        {
            startEncoder = motorR1StartEncoder;
            curEncoder = motorR1.getCurrentPosition();
        }
        else if(motorName.equals("motorR2"))
        {
            startEncoder = motorR2StartEncoder;
            curEncoder = motorR2.getCurrentPosition();
        }
        else
        {
            startEncoder = motorSpinnerStartEncoder;
            curEncoder = motorSpinner.getCurrentPosition();
        }
        initCurtime();
        double thisTime = getCurTime();
        timePassed = thisTime - startTimeOfLoop;
        double RPM = (curEncoder - startEncoder) / timePassed;
        return RPM;
    }
}

package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.DiagnosticData;
import org.firstinspires.ftc.teamcode.Libraries.DiagnosticLibrary;
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.ArrayList;

/**
 * Created by spencersharp on 1/7/17.
 */
@Autonomous(name = "DiagnosticCreator", group = "Test")
public class DiagnosticCreator extends MyOpMode
{
    long motorL1StartEncoder;
    long motorL2StartEncoder;
    long motorR1StartEncoder;
    long motorR2StartEncoder;
    long motorSpinnerStartEncoder;
    double startTimeOfLoop;
    DiagnosticLibrary dl;

    double batteryLevelM1;
    double batteryLevelM2;
    double batteryLevelM3;
    int changedBatteryLevelM1;
    int changedBatteryLevelM2;
    int changedBatteryLevelM3;



    public int getBatteryLevelAverage()
    {
        return (changedBatteryLevelM1 + changedBatteryLevelM2 + changedBatteryLevelM3) / 3;
    }

    public void updateBatteryLevels()
    {
        batteryLevelM1 = getVoltage("Motor Controller 1");
        batteryLevelM2 = getVoltage("Motor Controller 2");
        batteryLevelM3 = getVoltage("Motor Controller 3");
        changedBatteryLevelM1 = (int)(batteryLevelM1 * 100);
        changedBatteryLevelM2 = (int)(batteryLevelM2 * 100);
        changedBatteryLevelM3 = (int)(batteryLevelM3*100);
        telemetry.addData("changedBatteryLevelM1",changedBatteryLevelM1);
        telemetry.addData("changedBatteryLevelM2",changedBatteryLevelM2);
        telemetry.addData("changedBatteryLevelM3",changedBatteryLevelM3);
    }

    public void initializeDiagnosticLibrary()  throws IOException, ClassNotFoundException
    {
        try{
            FileInputStream fis = new FileInputStream("/sdcard/FIRST/DiagnosticLibraryStorage.txt");
            ObjectInputStream ois = new ObjectInputStream(fis);
            //out.println("hello");
            dl = (DiagnosticLibrary) ois.readObject();
            fis.close();
        }
        catch (FileNotFoundException fnes)
        {
            try{
                File file = new File("/sdcard/FIRST/DiagnosticLibraryStorage.txt");
                FileOutputStream fos = new FileOutputStream(file);
                dl = new DiagnosticLibrary();
                ObjectOutputStream oos = new ObjectOutputStream(fos);
                oos.writeObject(dl);
                oos.flush();
                oos.close();
            }
            catch(FileNotFoundException fnfe)
            {

            }
            catch(IOException ioe)
            {

            }
        }
        catch(IOException ioe)
        {

        }
        catch(ClassNotFoundException cnfe)
        {

        }
    }

    public void runOpMode() throws InterruptedException
    {
        super.runOpMode();
        try{initializeDiagnosticLibrary();} catch(ClassNotFoundException cnfe){} catch(IOException ioe){}
        telemetry.addData("File and diagnostic library initialized",dl);
        telemetry.update();
        waitForStart();
        pause(2.0);
        updateBatteryLevels();
        telemetry.update();
        pause(2.0);
        telemetry.addData("batteryLevel", (changedBatteryLevelM1 + changedBatteryLevelM2 + changedBatteryLevelM3) / 3);
        telemetry.update();
        pause(2.0);
        int startVal = getBatteryLevelAverage()-10;
        while(getBatteryLevelAverage()>1240)
        {
            telemetry.addData("loop","started");
            telemetry.update();
            double curSpeed = 1.0;
            while(curSpeed >= 0.01 && getBatteryLevelAverage() > 1240)
            {
                if(opModeIsActive()) {
                    motorL1.setPower(curSpeed);
                    motorL2.setPower(curSpeed);
                    motorR1.setPower(curSpeed);
                    motorR2.setPower(curSpeed);
                    motorSpinner.setPower(curSpeed);
                }
                //telemetry.addData("setting initial powers","true");
                //telemetry.update();
                pause(4.0);
                updateBatteryLevels();
                initCurtime();
                startTimeOfLoop = getCurTime();
                motorL1StartEncoder = motorL1.getCurrentPosition();
                motorL2StartEncoder = motorL2.getCurrentPosition();
                motorR1StartEncoder = motorR1.getCurrentPosition();
                motorR2StartEncoder = motorR2.getCurrentPosition();
                motorSpinnerStartEncoder = motorSpinner.getCurrentPosition();
                pause(5.0);
                ArrayList<DiagnosticData> diagnosticData = new ArrayList<DiagnosticData>();
                DiagnosticData motorL1dd = new DiagnosticData("motorL1",changedBatteryLevelM1,curSpeed,calculateRPM("motorL1"));
                diagnosticData.add(motorL1dd);
                telemetry.addData("motorL1", motorL1dd);
                DiagnosticData motorL2dd = new DiagnosticData("motorL2",changedBatteryLevelM1,curSpeed,calculateRPM("motorL2"));
                diagnosticData.add(motorL2dd);
                telemetry.addData("motorL2", motorL2dd);
                DiagnosticData motorR1dd = new DiagnosticData("motorR1", changedBatteryLevelM3, curSpeed, calculateRPM("motorR1"));
                diagnosticData.add(motorR1dd);
                telemetry.addData("motorR1", motorR1dd);
                DiagnosticData motorR2dd = new DiagnosticData("motorR2",changedBatteryLevelM3,curSpeed,calculateRPM("motorR2"));
                diagnosticData.add(motorR2dd);
                telemetry.addData("motorR2", motorR2dd);
                if(curSpeed > 0.2) {
                    DiagnosticData motorSpinnerdd = new DiagnosticData("motorSpinner", changedBatteryLevelM2, curSpeed, calculateRPM("motorSpinner"));
                    diagnosticData.add(motorSpinnerdd);
                    telemetry.addData("motorSpinner", motorSpinnerdd);
                }
                telemetry.update();
                addAllToLibrary(diagnosticData);
                stopMotors();
                motorSpinner.setPower(0.0);
                pause(6.5);
                curSpeed -= 0.05;
            }
            if(opModeIsActive()) {
                try {
                    dl.saveDiagnosticLibrary();
                } catch (IOException ioe) {

                }
            }
            curSpeed = -0.05;
            while(curSpeed >= -1.02 && getBatteryLevelAverage() > 1240)
            {
                if(opModeIsActive()) {
                    motorL1.setPower(curSpeed);
                    motorL2.setPower(curSpeed);
                    motorR1.setPower(curSpeed);
                    motorR2.setPower(curSpeed);
                }
                pause(0.5);
                updateBatteryLevels();
                initCurtime();
                startTimeOfLoop = getCurTime();
                motorL1StartEncoder = motorL1.getCurrentPosition();
                motorL2StartEncoder = motorL2.getCurrentPosition();
                motorR1StartEncoder = motorR1.getCurrentPosition();
                motorR2StartEncoder = motorR2.getCurrentPosition();

                pause(5.0);
                ArrayList<DiagnosticData> diagnosticData = new ArrayList<DiagnosticData>();
                DiagnosticData motorL1dd = new DiagnosticData("motorL1",changedBatteryLevelM1,curSpeed,calculateRPM("motorL1"));
                diagnosticData.add(motorL1dd);
                telemetry.addData("motorL1", motorL1dd);
                DiagnosticData motorL2dd = new DiagnosticData("motorL2",changedBatteryLevelM1,curSpeed,calculateRPM("motorL2"));
                diagnosticData.add(motorL2dd);
                telemetry.addData("motorL2", motorL2dd);
                DiagnosticData motorR1dd = new DiagnosticData("motorR1", changedBatteryLevelM3, curSpeed, calculateRPM("motorR1"));
                diagnosticData.add(motorR1dd);
                telemetry.addData("motorR1", motorR1dd);
                DiagnosticData motorR2dd = new DiagnosticData("motorR2",changedBatteryLevelM3,curSpeed,calculateRPM("motorR2"));
                diagnosticData.add(motorR2dd);
                telemetry.addData("motorR2", motorR2dd);
                telemetry.update();
                addAllToLibrary(diagnosticData);
                stopMotors();
                pause(2.0);
                curSpeed -= 0.05;
                if(curSpeed <= -0.99 && curSpeed >= -1.02)
                    curSpeed = -1.0;
            }
            if(opModeIsActive()) {
                try {
                    dl.saveDiagnosticLibrary();
                } catch (IOException ioe) {

                }
            }
        }
        if(opModeIsActive()){
            try {
                dl.saveDiagnosticLibrary();
            }
            catch(IOException ioe)
            {

            }
            pause(10.0);
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
        RPM /= 1120;
        return RPM;
    }
}

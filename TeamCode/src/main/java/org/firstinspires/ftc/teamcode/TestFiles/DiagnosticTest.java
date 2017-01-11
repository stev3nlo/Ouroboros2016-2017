package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libraries.DiagnosticData;
import org.firstinspires.ftc.teamcode.Libraries.DiagnosticLibrary;
import org.firstinspires.ftc.teamcode.Libraries.DiagnosticQuery;
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutput;
import java.io.ObjectOutputStream;

/**
 * Created by spencersharp on 1/7/17.
 */
@TeleOp(name = "DiagnosticTest", group = "Test")

public class DiagnosticTest extends MyOpMode
{
    DiagnosticLibrary dl;
    double motorL1MaxRPM;
    double motorL2MaxRPM;
    double motorR1MaxRPM;
    double motorR2MaxRPM;
    double motorSpinnerIdealRPM = 2.285;
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

    public void initializeMaxRPMs()
    {
    }
    public void runOpMode() throws InterruptedException
    {
        try{
            initializeDiagnosticLibrary();
        }
        catch(Exception e)
        {
        }
        telemetry.addData("dl",dl);
        /*
        boolean shooterIsRunning = false;
        while(opModeIsActive())
        {
            double batteryVoltage = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
            int changedBatteryVoltage = (int) batteryVoltage*100;
            double g1y1 = gamepad1.left_stick_y;
            double g1y2 = gamepad1.right_stick_y;
            boolean g1XPressed = gamepad1.x;
            boolean g1YPressed = gamepad1.y;
            DiagnosticQuery dqMotorL1 = new DiagnosticQuery("motorL1",changedBatteryVoltage,-motorL1MaxRPM*g1y1);
            DiagnosticQuery dqMotorL2 = new DiagnosticQuery("motorL2",changedBatteryVoltage,-motorL2MaxRPM*g1y1);
            DiagnosticQuery dqMotorR1 = new DiagnosticQuery("motorR1",changedBatteryVoltage,motorR1MaxRPM*g1y2);
            DiagnosticQuery dqMotorR2 = new DiagnosticQuery("motorR2",changedBatteryVoltage,motorR2MaxRPM*g1y2);
            double motorL1Speed = dl.getSpeedForDiagnosticQuery(dqMotorL1);
            double motorL2Speed = dl.getSpeedForDiagnosticQuery(dqMotorL2);
            double motorR1Speed = dl.getSpeedForDiagnosticQuery(dqMotorR1);
            double motorR2Speed = dl.getSpeedForDiagnosticQuery(dqMotorR2);
            motorL1.setPower(motorL1Speed);
            motorL2.setPower(motorL2Speed);
            motorR1.setPower(motorR1Speed);
            motorR2.setPower(motorR2Speed);
            if(g1XPressed || shooterIsRunning)
            {
                shooterIsRunning = true;
                DiagnosticQuery dqMotorSpinner = new DiagnosticQuery("motorSpinner",changedBatteryVoltage,motorSpinnerIdealRPM);
                double motorSpinnerSpeed = dl.getSpeedForDiagnosticQuery(dqMotorSpinner);
                motorSpinner.setPower(motorSpinnerSpeed);
            }
            else if (g1YPressed || !shooterIsRunning)
            {
                shooterIsRunning = false;
                motorSpinner.setPower(0.0);
            }
        }*/
        pause(300);
    }
}

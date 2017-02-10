package org.firstinspires.ftc.teamcode.Autonomouses;

import org.firstinspires.ftc.teamcode.Libraries.DiagnosticLibrary;
import org.firstinspires.ftc.teamcode.Libraries.DiagnosticQuery;
import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRColor;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRRange;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;


/**
 * Created by Steven on 11/14/2016.
 */
@Disabled//(name="Auto Blue Simple Test", group="Autonomous")
public class AutoBlueSimpleTest extends MyAutonomous {
    //travel 3ft to preferred shooting place
    //shoot
    //travel across the field to far beacon
    //using range1, stop x distance away
    //turn until both range1 and range2 are equal
    //press far beacon
    //back up until at closest beacon using color sensor
    //press beacon
    DiagnosticLibrary dl;

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


    public void runOpMode() throws InterruptedException {
        //rangeF.initializeSensors();
        //rangeB.initializeSensors();
        //initializeBlueServos();
        super.runOpMode();
        //try{initializeDiagnosticLibrary();}catch (IOException ioe){} catch(ClassNotFoundException cnfe){}
        //initializeSensors();
        double startAngle = gyro.getYaw();
        double yawDiff = 0.0;
        while (!opModeIsActive() && !isStopRequested()) {
            yawDiff = getAngleDiff(startAngle, gyro.getYaw());
            telemetry.addData("OGangle", startAngle);
            telemetry.addData("newANGLE", gyro.getYaw());
            telemetry.addData("yawDiff", yawDiff);
            telemetry.update();
            idle();
        }
        waitForStart();
        //moveAlongWallToBeacon(.3, 2.0, true);
        initCurtime();
        double startTime = getCurTime();
        double batteryLevel = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        //int adjustedBatteryLevel = (int)(batteryLevel * 100);
        //DiagnosticQuery dq = new DiagnosticQuery("motorSpinner",adjustedBatteryLevel,2.285);
        //double powerToSend = dl.getSpeedForDiagnosticQuery(dq);
        //runSpinner(powerToSend);
        //if (batteryLevel > 13.0)
        //    runSpinner(0.82);
        //else
        //    runSpinner(0.92);
        runSpinner(0.84);
        pause(0.1);
        moveWithEncoders(0.32, 3600,.93,1);
        pause(0.1);
        openServoDropper();
        pause(1.5);
        closeServoDropper();
        runSpinner(0.0);
        pause(0.25);
        moveWithEncoders(0.25, 3300,.93,1);
        gyroArcTurnRight(0.2, yawDiff - 13.0);

        while (opModeIsActive())
        {
            telemetry.addData("USDF",rangeF.getUltraSonicDistance());
            telemetry.addData("USDB",rangeB.getUltraSonicDistance());
            telemetry.update();
        }
        /*
        //moveWithEncoders(.5, 1000);

        pause(0.1);
        turnParallelToWallWithGyroSimple(0.195, 0);
        //17 max
        //9 min
        //stabilizeAlongWallWithRangeForEncoderDist(0.14, 1.0, 3.0, 11, true, 1000);
        //stabilizeAlongWallWithRangeToBeacon(0.115, 1.0, 3.0, 11, true);
        moveWithEncoders(0.18,1500);
        driveAlongWallToBeacon(0.95,true);

        pause(0.1);
        turnParallelToWallWithGyro(0.195, 0);
        pause(0.1);

        boolean foundBeacon = driveAlongWallToBeaconOrForUnits(0.105, true, 400);
        if (!foundBeacon) {
            pause(0.2);
            driveAlongWallToBeacon(-0.105, true);
        }
        pause(0.1);
        pushButton();
        pause(0.25);

        if (colorB.getColor().equals("Red")) {
            pause(5.0);
            pushButton();
        }

        turnParallelToWallWithGyro(0.195, 0);

        //Stabilizes along the wall for encoder distance
        //stabilizeAlongWall(-0.16, 4.0, 11, true, true, 2000);
        moveWithEncoders(-0.25,2000);
        pause(0.1);
        driveAlongWallToBeacon(-0.125, true);
        pause(0.1);
        pushButton();

        /*
        initCurtime();

        if (getCurTime() - startTime < 20.0) {
            turnParallelToWallWithGyro(0.195, 0);
            pause(0.1);
            foundBeacon = driveAlongWallToBeaconOrForUnits(-0.105, true, 400);
            if (!foundBeacon) {
                pause(0.2);
                driveAlongWallToBeacon(0.105, true);
            }
            pause(0.1);
            turnParallelToWallWithGyro(0.195, 0);
            pause(0.1);
            pushButton();
        } else if (getCurTime() - startTime < 24.0) {
            turnParallelToWallWithGyro(0.195, 0);
            pause(0.1);
            foundBeacon = driveAlongWallToBeaconOrForUnits(-0.105, true, 400);
            if (!foundBeacon) {
                pause(0.2);
                driveAlongWallToBeacon(0.105, true);
            }
            pushButton();
        } else if (getCurTime() - startTime < 28.0) {
            foundBeacon = driveAlongWallToBeaconOrForUnits(0.105, true, 400);
            pushButton();
        } else {
            pushButton();
        }*/
    }
}

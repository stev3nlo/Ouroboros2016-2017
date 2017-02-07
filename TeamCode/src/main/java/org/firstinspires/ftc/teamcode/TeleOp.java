package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Libraries.MotorScaler;
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.RPMStabilizer;
import org.firstinspires.ftc.teamcode.Libraries.SensorAdafruitIMU;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRColor;

import java.io.IOException;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Teleop", group="Teleop")  // @MyAutonomous(...) is the other common choice
public class TeleOp extends MyOpMode
{
    public boolean shooterIsRunning = false;
    //Controller values

    public double g1y1;    //left drive
    public double g1y2;    //right drive
    public double g1x1;
    public double g1x2;
    public boolean g1Lbump;
    public boolean g1Rbump;
    public double g1Ltrig;
    public double g1Rtrig;
    public boolean g1XPressed;
    public boolean g1YPressed;
    public boolean g1APressed;
    public boolean g1BPressed;

    int numLoopsRemainining;
    double timeAtLastLoop;
    double timePerLoop = 1.0;
    public int driverModeLeft = 1;
    public int driverModeRight = 1;

    public double g2y1;    //lift
    public double g2y2;    //manipulator
    public double g2x1;
    public double g2x2;
    public boolean g2Lbump;    //grab cap ball
    public boolean g2Rbump;    //release cap ball
    public double g2Ltrig;
    public double g2Rtrig;
    public boolean g2XPressed;     //start shooter spinner
    public boolean g2YPressed;     //stop shooter spinner
    public boolean g2APressed;     //toggle servo dropper
    public boolean g2BPressed;

    long timeAtEndOfLastCycle;

    double timeBallsFinishDropping;

    public boolean firstCycleOfSpinner;

    boolean isInterruptibleRoutineRedToBlueRunning = false;
    boolean isInterruptibleRoutineBlueToRedRunning = false;
    boolean hasGyroInitValueForInterruptibleRoutineBlueToRedOrRedToBlueBeenInitialized = false;
    boolean hasInterruptibleRoutineBeganRollingAlongWall = false;
    double gyroInitValueForInterruptibleRoutineBlueToRedOrRedToBlue = 0.0;

    boolean isInterruptibleRoutineDriveForwardsToRedAndPressRunning = false;
    boolean isInterruptibleRoutineDriveBackwardsToRedAndPressRunning = false;
    boolean isInterruptibleRoutineDriveForwardsToBlueAndPressRunning = false;
    boolean isInterruptibleRoutineDriveBackwardsToBlueAndPressRunning = false;
    boolean inDrivingRoutineHasColorBeenSeen = false;

    double startTimeInterruptibleRoutineRedToBlue = 0.0;
    double startTimeInterruptibleRoutineBlueToRed = 0.0;

    boolean areRollersRaised = true;

    public void updateControllerVals()
    {
        g1y1 = gamepad1.left_stick_y;
        g1y2 = gamepad1.right_stick_y;
        g1x1 = gamepad1.left_stick_x;
        g1x2 = gamepad1.right_stick_x;
        g1Lbump = gamepad1.left_bumper;
        g1Rbump = gamepad1.right_bumper;
        g1Ltrig = gamepad1.left_trigger;
        g1Rtrig = gamepad1.right_trigger;
        g1XPressed = gamepad1.x;
        g1APressed = gamepad1.a;
        g1YPressed = gamepad1.y;
        g1BPressed = gamepad1.b;

        g2y1 = gamepad2.left_stick_y;
        g2y2 = gamepad2.right_stick_y;
        g2x1 = gamepad2.left_stick_x;
        g2x2 = gamepad2.right_stick_x;
        g2Lbump = gamepad2.left_bumper;
        g2Rbump = gamepad2.right_bumper;
        g2Ltrig = gamepad2.left_trigger;
        g2Rtrig = gamepad2.right_trigger;
        g2XPressed = gamepad2.x;
        g2APressed = gamepad2.a;    //toggle servo dropper
        g2YPressed = gamepad2.y;
        g2BPressed = gamepad2.b;
    }

    public void updateTelemetry()
    {
        telemetry.addData("BEACON PUSH INFO","");
        telemetry.addData("curPowerOfMotorSpinner",curPowerOfMotorSpinner);
        if(g2Ltrig > 0.6)
        {
            telemetry.addData("g2Ltrig being pressed", "LEFT");
        }
        else if(g2Rtrig > 0.6)
        {
            telemetry.addData("g2Rtrig being pressed", "RIGHT");
        }
        else if(g2Rbump || g2Lbump)
        {
            telemetry.addData("g2Rbump or g2Lbump being pressed", "MIDDLE");
        }
        else
        {
            telemetry.addData("No beacon control being pressed","");
        }
        if(g1Ltrig > 0.6)
        {
            telemetry.addData("g1LTrig","PRESSED");
        }
        else
        {
            telemetry.addData("g1LTrig","NOT PRESSED");
        }
        if(g1Rtrig > 0.6)
        {
            telemetry.addData("g1RTrig","PRESSED");
        }
        else
        {
            telemetry.addData("g1RTrig","NOT PRESSED");
        }
        telemetry.update();
    }

    public void initializeBeaconColorSensor()
    {
        colorB = new SensorMRColor(hardwareMap.colorSensor.get("colorB"));
        colorB.sensorSetup(0x2c);
    }

    public void initializeGyro()
    {
        gyro = new SensorAdafruitIMU(hardwareMap.get(BNO055IMU.class, "gyro"));
        telemetry.addData("Gyro", "Initialized");
        telemetry.update();
    }

    public boolean isInterruptibleRoutineRunning()
    {
        return isInterruptibleRoutineBlueToRedRunning || isInterruptibleRoutineRedToBlueRunning;
    }

    public void runOpMode() throws InterruptedException
    {
        super.runOpMode();
        initializeBeaconColorSensor();
        initializeGyro();
        waitForStart();

        timeAtLastLoop = getCurTime();
        while (opModeIsActive())
        {
            updateControllerVals();
            initCurtime(); //gets real time timer
            String beaconColor = colorB.beaconColor();
            double curYaw = gyro.getYaw();

            if(g1Ltrig > 0.6)
                driverModeLeft = 3;
            else if(g1Lbump)
                driverModeLeft = 2;
            else
                driverModeLeft = 1;

            if(g1Rtrig > 0.6)
                driverModeRight = 3;
            else if(g1Rbump)
                driverModeRight = 2;
            else
                driverModeRight = 1;
            telemetry.addData("driverModeLeft",driverModeLeft);
            telemetry.addData("driverModeRight",driverModeRight);
            if(g1y1 > 0.1 || g1y1 < -0.1) {
                motorL1.setPower(g1y1/driverModeLeft);
                motorL2.setPower(g1y1/driverModeLeft);
            }
            else if(!isInterruptibleRoutineRunning())
            {
                motorL1.setPower(0.0);
                motorL2.setPower(0.0);
            }
            if(g1y2 > 0.1 || g1y2 < -0.1) {
                motorR1.setPower(-g1y2/driverModeRight);
                motorR2.setPower(-g1y2/driverModeRight);
            }
            else if(!isInterruptibleRoutineRunning())
            {
                motorR1.setPower(0.0);
                motorR2.setPower(0.0);
            }
            moveManip(g2y1);

            //Stabilization

            if (g2YPressed || !shooterIsRunning)
            {
                shooterIsRunning = false;
                runSpinner(0.0);
            }
            //creates constant speed of spinner throughout game
            if(g2XPressed || shooterIsRunning)
            {
                shooterIsRunning = true;
                runSpinner(0.88);
            }
            // stops spinner

            //releases balls from basket into spinner
            if (g2APressed)// && shooterIsRunning)
            {
                openServoDropper();
            }
            else if(g2BPressed)// || !shooterIsRunning)
            {
                closeServoDropper();
            }

            if(g1BPressed) //red
            {
                if(gamepad1.dpad_up)
                {

                }
                else if(gamepad1.dpad_down)
                {

                }
            }
            else if(g1XPressed) //blue
            {
                if(gamepad1.dpad_up)
                {

                }
                else if(gamepad1.dpad_down)
                {

                }
            }
            else if(!isInterruptibleRoutineBlueToRedRunning && g1YPressed) //blue to red (forwards)
            {
                isInterruptibleRoutineBlueToRedRunning = true;
                hasGyroInitValueForInterruptibleRoutineBlueToRedOrRedToBlueBeenInitialized = false;
                hasInterruptibleRoutineBeganRollingAlongWall = false;
                startTimeInterruptibleRoutineBlueToRed = getCurTime();
            }
            else if(!isInterruptibleRoutineRedToBlueRunning && g1APressed) //red to blue (backwards)
            {
                isInterruptibleRoutineRedToBlueRunning = true;
                hasGyroInitValueForInterruptibleRoutineBlueToRedOrRedToBlueBeenInitialized = false;
                hasInterruptibleRoutineBeganRollingAlongWall = false;
                startTimeInterruptibleRoutineRedToBlue = getCurTime();
            }

            if(areRollersDropping && getCurTime() - rollerMovementTimeDown > startTimeOfDroppingRollers)
            {
                stopRollers();
                areRollersDropping = false;
                areRollersRaised = false;
            }
            else if(areRollersRaising && getCurTime() - rollerMovementTimeUp > startTimeOfRaisingRollers)
            {
                holdRollersUp();
                areRollersRaising = false;
                areRollersRaised = true;
            }

            //Check if is interrupted
            telemetry.addData("yaw", curYaw);
            telemetry.addData("beaconColor",beaconColor);
            if(isInterruptibleRoutineBlueToRedRunning)
            {
                if(g1y1>0.1 || g1y1<-0.1 || g1y2 > 0.1 || g1y2 < -0.1) {
                    isInterruptibleRoutineBlueToRedRunning = false;
                    telemetry.addData("BROKEN","ROUTINE");
                    moveForwards(0.0);
                }
                else {
                    telemetry.addData("interruptibleRoutineRunning", "true");
                    if (!hasGyroInitValueForInterruptibleRoutineBlueToRedOrRedToBlueBeenInitialized) {
                        gyroInitValueForInterruptibleRoutineBlueToRedOrRedToBlue = gyro.getYaw();
                        hasGyroInitValueForInterruptibleRoutineBlueToRedOrRedToBlueBeenInitialized = true;
                    }
                    if (hasInterruptibleRoutineBeganRollingAlongWall) {
                        moveForwards(0.18, 0.13);
                        telemetry.addData("driving along next wall",true);
                    } else if (getAngleDiff(gyroInitValueForInterruptibleRoutineBlueToRedOrRedToBlue, curYaw) < 25) {
                        moveForwards(0.05, 0.3);
                        telemetry.addData("initial drift",true);
                    } else if (getAngleDiff(gyroInitValueForInterruptibleRoutineBlueToRedOrRedToBlue, curYaw) >= 25 && getAngleDiff(gyroInitValueForInterruptibleRoutineBlueToRedOrRedToBlue, curYaw) < 85) {
                        moveForwards(0.0, 0.5);
                        telemetry.addData("hard turn",true);
                    } else if (getAngleDiff(gyroInitValueForInterruptibleRoutineBlueToRedOrRedToBlue, curYaw) >= 65) {
                        hasInterruptibleRoutineBeganRollingAlongWall = true;
                        moveForwards(0.18, 0.13);
                    }
                    telemetry.addData("timeRunningIt",getCurTime()-startTimeInterruptibleRoutineBlueToRed);
                    /*
                    if(beaconColor.equals("Blue"))
                    {
                        isInterruptibleRoutineBlueToRedRunning = false;
                        hasInterruptibleRoutineBeganRollingAlongWall = false;
                        moveForwards(0.0);
                    }*/
                    /*
                    if (getCurTime() - startTimeInterruptibleRoutineRedToBlue > 1.0 && !beaconColor.equals("Neither")) {
                        if ((g1BPressed && beaconColor.equals("Red")) || (g1XPressed && beaconColor.equals("Blue"))) {
                            isInterruptibleRoutineBlueToRedRunning = false;
                            hasInterruptibleRoutineBeganRollingAlongWall = false;
                            moveForwards(0.0);
                            pushButtonWithRollersQuick();
                        } else if (!g1BPressed && !g1XPressed) {
                            isInterruptibleRoutineBlueToRedRunning = false;
                            hasInterruptibleRoutineBeganRollingAlongWall = false;
                            moveForwards(0.0);
                        }
                    }*/
                }
            }

            if(isInterruptibleRoutineRedToBlueRunning)
            {
                if(g1y1>0.1 || g1y1<-0.1 || g1y2 > 0.1 || g1y2 < -0.1) {
                    isInterruptibleRoutineRedToBlueRunning = false;
                    moveForwards(0.0);
                }
                else {
                    telemetry.addData("interruptibleRoutineRedToBlueRunning", "true");
                    moveForwards(0.5, 0.4);
                    if (getCurTime() - startTimeInterruptibleRoutineRedToBlue > 1.0 && !beaconColor.equals("Neither")) {
                        isInterruptibleRoutineRedToBlueRunning = false;
                        moveForwards(0.0);
                    }
                }
            }

            if(g2Ltrig > 0.6) //drop rollers
            {
                if(!areRollersDropping)
                {
                    if(areRollersRaising)
                    {
                        startTimeOfDroppingRollers = getCurTime() + getCurTime() - rollerMovementTimeDown - startTimeOfRaisingRollers;
                        areRollersRaising = false;
                    }
                    else
                    {
                        startTimeOfDroppingRollers = getCurTime();
                    }
                    moveRollersDown();
                    areRollersDropping = true;
                }
            }
            else if(g2Lbump) //raise rollers
            {
                if(!areRollersRaising)
                {
                    if(areRollersDropping)
                    {
                        startTimeOfRaisingRollers = getCurTime() + getCurTime() - rollerMovementTimeUp - startTimeOfDroppingRollers;
                        areRollersDropping = false;
                    }
                    else
                    {
                        startTimeOfRaisingRollers = getCurTime();
                    }
                    moveRollersUp();
                    areRollersRaising = true;
                }
            }
            else if(g2Rtrig > 0.6)
            {
                moveBeaconPusherOut();
            }
            else if(g2Rbump)
            {
                moveBeaconPusherIn();
            }

            updateTelemetry();
            idle();
        }
    }
}
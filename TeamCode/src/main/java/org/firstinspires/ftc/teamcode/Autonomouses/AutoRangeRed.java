package org.firstinspires.ftc.teamcode.Autonomouses;

        import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
        import org.firstinspires.ftc.teamcode.Libraries.SensorMRColor;
        import org.firstinspires.ftc.teamcode.Libraries.SensorMRRange;

//RangeF and RangeB will be backwards due to the robot being on the opposite side and goin backwards!!

/**
 * Created by Steven on 11/14/2016.
 */
public class AutoRangeRed extends MyOpMode {
    //travel 3ft to preferred shooting place
    //shoot
    //travel across the field to far beacon
    //using range1, stop x distance away
    //turn until both range1 and range2 are equal
    //press far beacon
    //back up until at closest beacon using color sensor
    //press beacon

    SensorMRRange rangeF;
    SensorMRRange rangeB;
    public void runOpMode() throws InterruptedException {
        rangeF.initializeSensors();
        rangeB.initializeSensors();
        //initializeRedServos();//
        super.runOpMode();
        waitForStart();

        //travel forward to prine shooting spot
        moveWithEncoders(.3, 2920); //needs to be tested

        //shoot
        //runs spinner
        initCurtime();
        double timeAtSpinnerStart = getCurTime();
        while(getCurTime()<timeAtSpinnerStart+1.0)
        {
            initCurtime();
            //shoot();
            idle();
        }

        //drops preplaced balls into spinner
        openServoDropper();
        initCurtime();
        double timeAtBallDrop = getCurTime();
        while(getCurTime()<timeAtBallDrop+5.0)
        {
            initCurtime();
            //shoot();
            idle();
        }
        closeServoDropper();

        //turns off spinnner
        runSpinner(0.0);

        //travel across the field to far beacon
        //using range1, stop x distance away
        while(rangeB.getUltraSonicDistance() > 10) //random number!!!! needs to be replaced
        {
            moveBackwards(1.0);
        }
        stopMotors();

        //sets servos parallel to Wall
        //setServosParallel();

        //turn until both range1 and range2 are equal
        while(!(rangeF.getUltraSonicDistance() == rangeB.getUltraSonicDistance()))
        {
            move(1.0, 0.0);
        }
        stopMotors();


        //far beacon
        //move back until colorsensor senses right color
        moveForwardToBeacon("Red");
        //press far beacon
        pushButton();

        //close beacon
        //move back until colorsensor senses right color
        moveForwardToBeacon("Red");
        //press far beacon
        pushButton();


    }
}

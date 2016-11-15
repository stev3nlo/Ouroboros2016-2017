package org.firstinspires.ftc.teamcode.Autonomouses;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRRange;

/**
 * Created by Steven on 11/14/2016.
 */
public class AutoBlueRange extends MyOpMode {
    //travel across the field to far beacon
    //using range1, stop x distance away
    //turn until both range1 and range2 are equal
    //press far beacon
    //back up until at closest beacon
    //press beacon
    //shoot
    SensorMRRange rangeF;
    SensorMRRange rangeB;
    int firstDistance;

    public void runOpMode() throws InterruptedException {
        rangeF.initializeSensors();
        rangeB.initializeSensors();
        super.runOpMode();
        waitForStart();


        //travel across the field to far beacon
        //using range1, stop x distance away
        while(rangeF.getUltraSonicDistance() > 10) //random number
        {
          moveForwards(1.0);
        }

        //turn until both range1 and range2 are equal
        while(!(rangeF.getUltraSonicDistance() == rangeB.getUltraSonicDistance()))
        {
            firstDistance = rangeB.getUltraSonicDistance();
            gyroTurnLeft(1.0, 45);
            gyroTurnLeftCorrection(1.0, 45);
        }
        //press far beacon
        pushButton("Blue");

        //back up until at closest beacon
        while(!(rangeB.getUltraSonicDistance() < firstDistance ) )
        {
            moveBackwards();
        }



    }
}

package org.firstinspires.ftc.teamcode.Libraries;

/**
 * Created by Morgannanez on 11/15/16.
 */
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

public class MROpticalDistanceSensor extends MyOpMode{

  OpticalDistanceSensor ods;

  public MROpticalDistanceSensor(OpticalDistanceSensor ods)
  {
    this.ods = ods;

  }

  public double lightDetected()
  {
    return ods.getLightDetected();
  }

  public double rawLightDetected()
  {
    return ods.getRawLightDetected();
  }

  public double rawLightDetectedMax()
  {
    return ods.getRawLightDetectedMax();
  }



}


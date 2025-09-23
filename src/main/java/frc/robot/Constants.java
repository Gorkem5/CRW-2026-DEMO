// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class DriveConstants {
        
    public static final double MaxAcc_X = 15;
    public static final double MaxAcc_Y = 15; 

    public static final double JOYDeadzone_X = 0.1;
    public static final double JOYDeadzone_Y = 0.1;
    public static final double JOYDeadzone_Rot = 0.1;

    public static final boolean FAKE_ODOM = true ;

}

public static class SwerveConstants {

    public static final double GearRatio_Angle = 18;
    public static final double SpeedCalibValue = 0.33 / 2.0;

    public static final double AnglePID_P = 0.01; 
    public static final double AnglePID_I = 0; 
    public static final double AnglePID_D = 0.2; 

    public static final double DrivePID_P = 0.01; 
    public static final double DrivePID_I = 0; 
    public static final double DrivePID_D = 0;



    public static final int AngleCANID_FL = 4;
    public static final int DriveCANID_FL = 6;

    public static final int AngleCANID_FR = 2;
    public static final int DriveCANID_FR = 5;

    public static final int AngleCANID_BL = 8;
    public static final int DriveCANID_BL = 7;

    public static final int AngleCANID_BR = 1;
    public static final int DriveCANID_BR = 3;
}

}

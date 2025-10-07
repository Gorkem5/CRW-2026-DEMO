package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
  private Constants() {}

  public static final class DriveConstants {
    private DriveConstants() {}

    public static final double MaxAcc_X = 15.0;
    public static final double MaxAcc_Y = 15.0;

    public static final double JOYDeadzone_X = 0.1;
    public static final double JOYDeadzone_Y = 0.1;
    public static final double JOYDeadzone_Rot = 0.1;

    public static final boolean FAKE_ODOM = false;

    public static final double MaxAngularRadPerSec = Math.PI * 2.0;
    public static final double MaxLinearSpeedMps = 3.0;

    public static final double HeadingHoldKp = 6.0;
    public static final double HeadingHoldKi = 0.0;
    public static final double HeadingHoldKd = 0.2;

    public static final double HeadingHoldMaxVelRadPerSec = Math.PI * 4.0;
    public static final double HeadingHoldMaxAccelRadPerSecSq = Math.PI * 8.0;

    public static final double HeadingHoldCancelThreshold = 0.2;
    public static final double HeadingHoldToleranceDeg = 1.5;
    public static final double HeadingHoldCompletionOmegaRadPerSec = Math.toRadians(20.0);

    public static final int DRIVE_CURRENT_LIMIT_AMPS = 40;
    public static final int STEER_CURRENT_LIMIT_AMPS = 30;
    public static final double VOLTAGE_COMPENSATION = 12.0;
  }

  public static final class SwerveConstants {
    private SwerveConstants() {}

    public static final int AngleCANID_FL = 4;
    public static final int DriveCANID_FL = 6;

    public static final int AngleCANID_FR = 2;
    public static final int DriveCANID_FR = 5;

    public static final int AngleCANID_BL = 8;
    public static final int DriveCANID_BL = 7;

    public static final int AngleCANID_BR = 1;
    public static final int DriveCANID_BR = 3;

    public static final double HALF_WHEELBASE_METERS = Units.inchesToMeters(11.2);
    public static final double HALF_TRACKWIDTH_METERS = Units.inchesToMeters(10.5);
    public static final double WHEELBASE_METERS = HALF_WHEELBASE_METERS * 2.0;
    public static final double TRACKWIDTH_METERS = HALF_TRACKWIDTH_METERS * 2.0;

    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
    public static final double DRIVE_GEAR_RATIO = 6.75;

    public static final double GearRatio_Angle = 18.0;
    public static final double SpeedCalibValue = 0.33 / 2.0;

    public static final double AnglePID_P = 0.01;
    public static final double AnglePID_I = 0.0;
    public static final double AnglePID_D = 0.2;

    public static final double DrivePID_P = 0.01;
    public static final double DrivePID_I = 0.0;
    public static final double DrivePID_D = 0.0;

    public static final double MAX_MODULE_SPEED_MPS = 3.0;
  }
}

package frc.robot.drivetrain;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;

import frc.robot.Constants.SwerveConstants;

public class SwerveModule {

    private SparkMax angleSpark;
    private SparkMax driveSpark;

    private RelativeEncoder angleEncoder;
    private RelativeEncoder driveEncoder;

    private SparkClosedLoopController angleController;
    private SparkClosedLoopController driveController;

    private boolean inverted = false;
    private double targetAngleAbsolute = 0.0;
    private double distanceDriven = 0.0;

    private double simAngleRad = 0.0;
    private double simTurnRate = Math.toRadians(720.0);
    private double commandedSpeedMps = 0.0;

    public SwerveModule(int AngleCANID, int DriveCANID) {
        angleSpark = new SparkMax(AngleCANID, MotorType.kBrushless);
        driveSpark = new SparkMax(DriveCANID, MotorType.kBrushless);

        SparkMaxConfig angleCfg = new SparkMaxConfig();
        SparkMaxConfig driveCfg = new SparkMaxConfig();

        double angleConvFactor = (2.0 * Math.PI) / SwerveConstants.GearRatio_Angle;

        angleCfg.closedLoop.pid(SwerveConstants.AnglePID_P, SwerveConstants.AnglePID_I, SwerveConstants.AnglePID_D);
        angleCfg.closedLoop.outputRange(-0.5, 0.5);
        angleCfg.idleMode(IdleMode.kBrake);
        angleCfg.encoder.positionConversionFactor(angleConvFactor);

        driveCfg.closedLoop.pid(SwerveConstants.DrivePID_P, SwerveConstants.DrivePID_I, SwerveConstants.DrivePID_D);
        driveCfg.idleMode(IdleMode.kCoast);

        angleSpark.configure(angleCfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        driveSpark.configure(driveCfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        angleEncoder = angleSpark.getEncoder();
        driveEncoder = driveSpark.getEncoder();
        angleController = angleSpark.getClosedLoopController();
        driveController = driveSpark.getClosedLoopController();
    }

    private double shortestTo(double currentAngle, double targetAngle) {
        double curNorm = currentAngle % (2.0 * Math.PI);
        if (curNorm < 0) curNorm += 2.0 * Math.PI;

        double guess = targetAngle + currentAngle - curNorm;
        double diff  = targetAngle - curNorm;

        if (diff > Math.PI)  guess -= 2.0 * Math.PI;
        if (diff < -Math.PI) guess += 2.0 * Math.PI;

        return guess;
    }

    public double GetAngle() {
        if (RobotBase.isSimulation()) {
            return simAngleRad;
        }
        return angleEncoder.getPosition();
    }

    public void SetTargetAngle(double angDeg, boolean forceForward) {
        double current = GetAngle();
        double desired = Math.toRadians(angDeg);

        double targetNoFlip   = shortestTo(current, desired);
        double desiredFlipped = (desired > Math.PI) ? desired - Math.PI : desired + Math.PI;
        double targetWithFlip = shortestTo(current, desiredFlipped);

        double errNoFlip = Math.abs(targetNoFlip   - current);
        double errFlip   = Math.abs(targetWithFlip - current);

        boolean useFlip = (!forceForward) && (errFlip < errNoFlip);

        targetAngleAbsolute = useFlip ? targetWithFlip : targetNoFlip;
        inverted = useFlip;

        angleController.setReference(targetAngleAbsolute, ControlType.kPosition);
    }

    public void SetTargetAngle(double angleDegrees) {
        SetTargetAngle(angleDegrees, false);
    }

    public void Drive(double speedMps) {
        SetSpeed(speedMps);
    }

    public double GetSpeed() {
        if (RobotBase.isSimulation()) {
            return (inverted ? -1.0 : 1.0) * commandedSpeedMps;
        }
        double rpm = driveEncoder.getVelocity();
        return rpm / 250.0 / SwerveConstants.SpeedCalibValue;
    }

    void SetSpeed(double speedMps) {
        commandedSpeedMps = speedMps;
        double sign = inverted ? -1.0 : 1.0;
        double motorVelRef = sign * speedMps * 250.0 * SwerveConstants.SpeedCalibValue;
        driveController.setReference(motorVelRef, ControlType.kVelocity);
    }

    public void Update(double deltaTimeSeconds) {
        if (RobotBase.isSimulation()) {
            double err = targetAngleAbsolute - simAngleRad;
            err = ((err + Math.PI) % (2.0 * Math.PI));
            if (err < 0) err += 2.0 * Math.PI;
            err -= Math.PI;

            double maxStep = simTurnRate * deltaTimeSeconds;
            double step = Math.max(-maxStep, Math.min(maxStep, err));
            simAngleRad += step;
        }

        distanceDriven += GetSpeed() * deltaTimeSeconds;
    }

    public SwerveModulePosition GetPosition() {
        return new SwerveModulePosition(distanceDriven, Rotation2d.fromRadians(GetAngle()));
    }

    public SwerveModuleState GetState() {
        return new SwerveModuleState(GetSpeed(), Rotation2d.fromRadians(GetAngle()));
    }
}

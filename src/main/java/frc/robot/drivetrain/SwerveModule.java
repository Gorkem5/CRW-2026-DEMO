package frc.robot.drivetrain;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants.SwerveConstants;

public class SwerveModule {

    private SparkMax angleMotor;
    private SparkMax driveMotor;

    private RelativeEncoder angleEnc;
    private RelativeEncoder driveEnc;

    private SparkClosedLoopController anglePID;
    private SparkClosedLoopController drivePID;

    private double distDriven = 0.0;
    private double targetAngleRad = 0.0;
    private boolean inverted = false;

    public SwerveModule(int angleId, int driveId) {
        angleMotor = new SparkMax(angleId, MotorType.kBrushless);
        driveMotor = new SparkMax(driveId, MotorType.kBrushless);

        SparkMaxConfig angleCfg = new SparkMaxConfig();
        SparkMaxConfig driveCfg = new SparkMaxConfig();

        double angleFactor = (2 * Math.PI) / SwerveConstants.GearRatio_Angle;

        angleCfg.closedLoop.pid(SwerveConstants.AnglePID_P, SwerveConstants.AnglePID_I, SwerveConstants.AnglePID_D);
        angleCfg.closedLoop.outputRange(-0.5, 0.5);
        angleCfg.encoder.positionConversionFactor(angleFactor);
        angleCfg.idleMode(IdleMode.kBrake);

        driveCfg.closedLoop.pid(SwerveConstants.DrivePID_P, SwerveConstants.DrivePID_I, SwerveConstants.DrivePID_D);
        driveCfg.idleMode(IdleMode.kCoast);

        angleMotor.configure(angleCfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        driveMotor.configure(driveCfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        angleEnc = angleMotor.getEncoder();
        driveEnc = driveMotor.getEncoder();

        anglePID = angleMotor.getClosedLoopController();
        drivePID = driveMotor.getClosedLoopController();
    }

    public double getAngleRad() {
        return angleEnc.getPosition();
    }

    private double findShortestPath(double current, double target) {
        double curNorm = current % (2 * Math.PI);
        if (curNorm < 0) curNorm += 2 * Math.PI;

        double guess = target + (current - curNorm);

        if (target - curNorm > Math.PI) guess -= 2 * Math.PI;
        if (target - curNorm < -Math.PI) guess += 2 * Math.PI;

        return guess;
    }

    public void setTargetAngle(double deg, boolean forceForward) {
        double now = getAngleRad();
        double desired = Math.toRadians(deg);

        double option1 = findShortestPath(now, desired);
        double option2 = findShortestPath(now, (desired > Math.PI) ? desired - Math.PI : desired + Math.PI);

        double diff1 = Math.abs(option1 - now);
        double diff2 = Math.abs(option2 - now);

        if (!forceForward && diff1 > diff2) {
            targetAngleRad = option2;
            inverted = false;
        } else {
            targetAngleRad = option1;
            inverted = true;
        }

        anglePID.setReference(targetAngleRad, ControlType.kPosition);
    }

    public void setTargetAngle(double deg) {
        setTargetAngle(deg, false);
    }

    public void drive(double speedMps) {
        double adjSpeed = inverted ? -speedMps : speedMps;
        drivePID.setReference(adjSpeed * 250.0 * SwerveConstants.SpeedCalibValue, ControlType.kVelocity);
    }

    public double getSpeedMps() {
        double rpm = driveEnc.getVelocity();
        return rpm / (250.0 * SwerveConstants.SpeedCalibValue);
    }

    public void update(double dt) {
        distDriven += getSpeedMps() * dt;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(distDriven, Rotation2d.fromRadians(getAngleRad()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeedMps(), Rotation2d.fromRadians(getAngleRad()));
    }
}

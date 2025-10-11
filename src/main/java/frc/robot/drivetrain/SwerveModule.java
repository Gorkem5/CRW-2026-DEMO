package frc.robot.drivetrain;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;

import frc.robot.Constants.SwerveConstants;

public class SwerveModule {

    private final SparkMax angleSpark;
    private final SparkMax driveSpark;

    private final RelativeEncoder angleEncoder;
    private final RelativeEncoder driveEncoder;

    private final SparkClosedLoopController angleController;
    private final SparkClosedLoopController driveController;

    private boolean inverted = false;
    private double targetAngleAbsolute = 0.0;

    private SparkSim driveSim;
    private SparkSim turnSim;
    private SparkRelativeEncoderSim driveEncoderSim;
    private SparkRelativeEncoderSim turnEncoderSim;

    private double lastDriveVelocityRpm = 0.0;
    private double simAngleRad = 0.0;
    private double simDriveRotations = 0.0;

    private static final DCMotor DRIVE_MOTOR_MODEL = DCMotor.getNEO(1);
    private static final DCMotor TURN_MOTOR_MODEL = DCMotor.getNEO(1);
    private static final double SIM_TURN_RATE_RAD_PER_SEC = Math.toRadians(720.0);

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
        angleCfg.encoder.velocityConversionFactor(angleConvFactor);

        driveCfg.closedLoop.pid(SwerveConstants.DrivePID_P, SwerveConstants.DrivePID_I, SwerveConstants.DrivePID_D);
        driveCfg.idleMode(IdleMode.kCoast);

        angleSpark.configure(angleCfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        driveSpark.configure(driveCfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        angleEncoder = angleSpark.getEncoder();
        driveEncoder = driveSpark.getEncoder();
        angleController = angleSpark.getClosedLoopController();
        driveController = driveSpark.getClosedLoopController();

        if (RobotBase.isSimulation()) {
            driveSim = new SparkSim(driveSpark, DRIVE_MOTOR_MODEL);
            turnSim = new SparkSim(angleSpark, TURN_MOTOR_MODEL);

            driveSim.setBusVoltage(12.0);
            turnSim.setBusVoltage(12.0);

            driveEncoderSim = driveSim.getRelativeEncoderSim();
            turnEncoderSim = turnSim.getRelativeEncoderSim();

            if (driveEncoderSim != null) {
                driveEncoderSim.setPosition(0.0);
                driveEncoderSim.setVelocity(0.0);
            }
            if (turnEncoderSim != null) {
                turnEncoderSim.setPosition(0.0);
                turnEncoderSim.setVelocity(0.0);
            }
        }
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
            return (lastDriveVelocityRpm / 250.0) / SwerveConstants.SpeedCalibValue;
        }
        double rpm = driveEncoder.getVelocity();
        return rpm / 250.0 / SwerveConstants.SpeedCalibValue;
    }

    void SetSpeed(double speedMps) {
        double sign = inverted ? -1.0 : 1.0;
        double motorVelRef = sign * speedMps * 250.0 * SwerveConstants.SpeedCalibValue;
        lastDriveVelocityRpm = motorVelRef;
        driveController.setReference(motorVelRef, ControlType.kVelocity);
    }

    public void Update(double deltaTimeSeconds) {
        if (!RobotBase.isSimulation()) {
            return;
        }

        double busVoltage = 12.0;

        double err = targetAngleAbsolute - simAngleRad;
        err = ((err + Math.PI) % (2.0 * Math.PI));
        if (err < 0) err += 2.0 * Math.PI;
        err -= Math.PI;

        double maxStep = SIM_TURN_RATE_RAD_PER_SEC * deltaTimeSeconds;
        double step = Math.max(-maxStep, Math.min(maxStep, err));
        simAngleRad += step;
        double turnVel = deltaTimeSeconds > 0.0 ? step / deltaTimeSeconds : 0.0;

        if (turnSim != null) {
            turnSim.iterate(turnVel, busVoltage, deltaTimeSeconds);
            turnSim.setPosition(simAngleRad);
            turnSim.setVelocity(turnVel);
        }
        if (turnEncoderSim != null) {
            turnEncoderSim.setPosition(simAngleRad);
            turnEncoderSim.setVelocity(turnVel);
        }

        simDriveRotations += (lastDriveVelocityRpm / 60.0) * deltaTimeSeconds;

        if (driveSim != null) {
            driveSim.iterate(lastDriveVelocityRpm, busVoltage, deltaTimeSeconds);
            driveSim.setPosition(simDriveRotations);
            driveSim.setVelocity(lastDriveVelocityRpm);
        }
        if (driveEncoderSim != null) {
            driveEncoderSim.setPosition(simDriveRotations);
            driveEncoderSim.setVelocity(lastDriveVelocityRpm);
        }
    }

    public SwerveModulePosition GetPosition() {
        double rotations = RobotBase.isSimulation() ? simDriveRotations : driveEncoder.getPosition();
        double distanceMeters = rotations / 250.0 / SwerveConstants.SpeedCalibValue;
        return new SwerveModulePosition(distanceMeters, Rotation2d.fromRadians(GetAngle()));
    }

    public SwerveModuleState GetState() {
        return new SwerveModuleState(GetSpeed(), Rotation2d.fromRadians(GetAngle()));
    }
}

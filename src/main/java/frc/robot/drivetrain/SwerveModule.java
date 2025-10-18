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

    private static final DCMotor DRIVE_MOTOR_MODEL = DCMotor.getNEO(1);
    private static final DCMotor TURN_MOTOR_MODEL = DCMotor.getNEO(1);
    private static final double SIM_TURN_RATE_RAD_PER_SEC = Math.toRadians(720.0);
    private static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * SwerveConstants.WHEEL_DIAMETER_METERS;

    private final SparkMax angleSpark;
    private final SparkMax driveSpark;

    private final RelativeEncoder angleEncoder;
    private final RelativeEncoder driveEncoder;

    private final SparkClosedLoopController angleController;
    private final SparkClosedLoopController driveController;

    private final double angleConvFactor;

    private boolean inverted = false;
    private double targetAngleAbsolute = 0.0;

    private SparkSim driveSim;
    private SparkSim turnSim;
    private SparkRelativeEncoderSim driveEncoderSim;
    private SparkRelativeEncoderSim turnEncoderSim;

    private double lastMotorRpm = 0.0;
    private double simAngleRad = 0.0;
    private double simDriveRotations = 0.0;

    public SwerveModule(int angleCANID, int driveCANID) {
        angleSpark = new SparkMax(angleCANID, MotorType.kBrushless);
        driveSpark = new SparkMax(driveCANID, MotorType.kBrushless);

        angleConvFactor = (2.0 * Math.PI) / SwerveConstants.GearRatio_Angle;

        SparkMaxConfig angleCfg = new SparkMaxConfig();
        SparkMaxConfig driveCfg = new SparkMaxConfig();

        angleCfg.closedLoop.pid(
            SwerveConstants.AnglePID_P,
            SwerveConstants.AnglePID_I,
            SwerveConstants.AnglePID_D);
        angleCfg.closedLoop.outputRange(-0.5, 0.5);
        angleCfg.idleMode(IdleMode.kBrake);

        driveCfg.closedLoop.pid(
            SwerveConstants.DrivePID_P,
            SwerveConstants.DrivePID_I,
            SwerveConstants.DrivePID_D);
        driveCfg.idleMode(IdleMode.kCoast);

        angleSpark.configure(angleCfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        driveSpark.configure(driveCfg, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        angleEncoder = angleSpark.getEncoder();
        driveEncoder = driveSpark.getEncoder();

        angleController = angleSpark.getClosedLoopController();
        driveController = driveSpark.getClosedLoopController();

        zeroSteerRelative();

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

    private void zeroSteerRelative() {
        setModuleAngleReference(0.0);
    }

    private void setModuleAngleReference(double moduleAngleRad) {
        targetAngleAbsolute = moduleAngleRad;
        inverted = false;

        double motorRotations = moduleAngleRad / angleConvFactor;
        angleEncoder.setPosition(motorRotations);

        if (RobotBase.isSimulation()) {
            simAngleRad = moduleAngleRad;
        }
    }

    public void forceSteerToZero() {
        zeroSteerRelative();
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
        return motorRotationsToModuleRadians(angleEncoder.getPosition());
    }

    private double motorRotationsToModuleRadians(double motorRotations) {
        return motorRotations * angleConvFactor;
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

        double motorReference = targetAngleAbsolute / angleConvFactor;
        angleController.setReference(motorReference, ControlType.kPosition);
    }

    public void SetTargetAngle(double angleDegrees) {
        SetTargetAngle(angleDegrees, false);
    }

    public void Drive(double speedMps) {
        SetSpeed(speedMps);
    }

    private static double motorRotationsToMeters(double motorRotations) {
        return motorRotations * WHEEL_CIRCUMFERENCE_METERS / SwerveConstants.DRIVE_GEAR_RATIO;
    }

    private static double motorRpmToMetersPerSecond(double motorRpm) {
        return (motorRpm / 60.0) * WHEEL_CIRCUMFERENCE_METERS / SwerveConstants.DRIVE_GEAR_RATIO;
    }

    private static double metersPerSecondToMotorRpm(double metersPerSecond) {
        double wheelRps = metersPerSecond / WHEEL_CIRCUMFERENCE_METERS;
        double motorRps = wheelRps * SwerveConstants.DRIVE_GEAR_RATIO;
        return motorRps * 60.0;
    }

    public double GetSpeed() {
        double motorRpm = driveEncoder.getVelocity();
        return motorRpmToMetersPerSecond(motorRpm);
    }

    void SetSpeed(double speedMps) {
        double commanded = inverted ? -speedMps : speedMps;
        lastMotorRpm = metersPerSecondToMotorRpm(commanded);
        driveController.setReference(lastMotorRpm, ControlType.kVelocity);
    }

    public void Update(double deltaTimeSeconds) {
        if (!RobotBase.isSimulation()) {
            return;
        }

        double err = targetAngleAbsolute - simAngleRad;
        err = ((err + Math.PI) % (2.0 * Math.PI));
        if (err < 0) err += 2.0 * Math.PI;
        err -= Math.PI;

        double maxStep = SIM_TURN_RATE_RAD_PER_SEC * deltaTimeSeconds;
        double step = Math.max(-maxStep, Math.min(maxStep, err));
        simAngleRad += step;
        double turnVel = deltaTimeSeconds > 0.0 ? step / deltaTimeSeconds : 0.0;

        double angleRotations = simAngleRad / angleConvFactor;
        double angleRpm = (turnVel / angleConvFactor) * 60.0;

        if (turnSim != null) {
            turnSim.setPosition(angleRotations);
            turnSim.setVelocity(angleRpm);
        }
        if (turnEncoderSim != null) {
            turnEncoderSim.setPosition(angleRotations);
            turnEncoderSim.setVelocity(angleRpm);
        }

        double motorRotPerSec = lastMotorRpm / 60.0;
        simDriveRotations += motorRotPerSec * deltaTimeSeconds;

        if (driveSim != null) {
            driveSim.setPosition(simDriveRotations);
            driveSim.setVelocity(lastMotorRpm);
        }
        if (driveEncoderSim != null) {
            driveEncoderSim.setPosition(simDriveRotations);
            driveEncoderSim.setVelocity(lastMotorRpm);
        }
    }

    public SwerveModulePosition GetPosition() {
        double motorRotations = driveEncoder.getPosition();
        double distanceMeters = motorRotationsToMeters(motorRotations);
        return new SwerveModulePosition(distanceMeters, Rotation2d.fromRadians(GetAngle()));
    }

    public SwerveModuleState GetState() {
        return new SwerveModuleState(GetSpeed(), Rotation2d.fromRadians(GetAngle()));
    }
}

package frc.robot.drivetrain.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.drivetrain.SwerveSubsystem;

public class SwerveJoystickDriveCommand extends Command {
    private final SwerveSubsystem swerve;
    private final Supplier<Double> xInput;
    private final Supplier<Double> yInput;
    private final Supplier<Double> rotInput;
    private final boolean useDeadzone;

    private final ProfiledPIDController headingController = new ProfiledPIDController(
        DriveConstants.HeadingHoldKp,
        DriveConstants.HeadingHoldKi,
        DriveConstants.HeadingHoldKd,
        new TrapezoidProfile.Constraints(
            DriveConstants.HeadingHoldMaxVelRadPerSec,
            DriveConstants.HeadingHoldMaxAccelRadPerSecSq));

    private double limitedX = 0.0;
    private double limitedY = 0.0;
    private double lastTimestamp = Timer.getFPGATimestamp();

    private boolean fieldRelative = true;
    private boolean headingHoldEnabled = false;
    private boolean headingHoldOneShot = false;
    private Rotation2d headingTarget = new Rotation2d();

    public SwerveJoystickDriveCommand(
            SwerveSubsystem swerve,
            Supplier<Double> xSupplier,
            Supplier<Double> ySupplier,
            Supplier<Double> rotSupplier,
            boolean useDeadzone) {

        this.swerve = swerve;
        this.xInput = xSupplier;
        this.yInput = ySupplier;
        this.rotInput = rotSupplier;
        this.useDeadzone = useDeadzone;

        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(Math.toRadians(DriveConstants.HeadingHoldToleranceDeg));

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        lastTimestamp = Timer.getFPGATimestamp();
        headingTarget = swerve.getHeading();
        headingHoldEnabled = false;
        headingHoldOneShot = false;
        limitedX = 0.0;
        limitedY = 0.0;
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTimestamp;
        if (dt <= 0.0 || dt > 0.1) dt = 0.02;
        lastTimestamp = now;

        double x = xInput.get();
        double y = yInput.get();
        double rot = rotInput.get();

        if (useDeadzone) {
            x = applyDeadzone(x, DriveConstants.JOYDeadzone_X);
            y = applyDeadzone(y, DriveConstants.JOYDeadzone_Y);
            rot = applyDeadzone(rot, DriveConstants.JOYDeadzone_Rot);
        }

        limitedX = rampTowards(limitedX, x, DriveConstants.MaxAcc_X, dt);
        limitedY = rampTowards(limitedY, y, DriveConstants.MaxAcc_Y, dt);

        double omega = calculateOmega(rot);
        double vx = limitedX * DriveConstants.MaxLinearSpeedMps;
        double vy = -limitedY * DriveConstants.MaxLinearSpeedMps;

        ChassisSpeeds speeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, swerve.getHeading())
            : new ChassisSpeeds(vx, vy, omega);

        swerve.driveRobotRelative(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void toggleFieldRelative() {
        fieldRelative = !fieldRelative;
    }

    public boolean isFieldRelative() {
        return fieldRelative;
    }

    public void requestHeadingHold(Rotation2d targetHeading, boolean oneShot) {
        headingTarget = targetHeading;
        headingHoldEnabled = true;
        headingHoldOneShot = oneShot;
        headingController.reset(swerve.getHeading().getRadians());
    }

    public void clearHeadingHold() {
        headingHoldEnabled = false;
        headingHoldOneShot = false;
        headingTarget = swerve.getHeading();
    }

    public void quickFlipHeading() {
        requestHeadingHold(swerve.getHeading().plus(Rotation2d.fromDegrees(180.0)), true);
    }

    public void setHeadingHoldTarget(Rotation2d targetHeading) {
        if (headingHoldEnabled) {
            headingTarget = targetHeading;
        }
    }

    private double calculateOmega(double rotationInput) {
        if (headingHoldEnabled && Math.abs(rotationInput) > DriveConstants.HeadingHoldCancelThreshold) {
            clearHeadingHold();
        }

        if (!headingHoldEnabled) {
            headingTarget = swerve.getHeading();
            return -rotationInput * DriveConstants.MaxAngularRadPerSec;
        }

        double omega = MathUtil.clamp(
            headingController.calculate(swerve.getHeading().getRadians(), headingTarget.getRadians()),
            -DriveConstants.MaxAngularRadPerSec,
            DriveConstants.MaxAngularRadPerSec);

        double headingError = headingTarget.minus(swerve.getHeading()).getRadians();
        if (headingHoldOneShot
            && Math.abs(headingError) <= Math.toRadians(DriveConstants.HeadingHoldToleranceDeg)
            && Math.abs(omega) < DriveConstants.HeadingHoldCompletionOmegaRadPerSec) {
            clearHeadingHold();
            return -rotationInput * DriveConstants.MaxAngularRadPerSec;
        }

        return omega;
    }

    private static double applyDeadzone(double value, double deadzone) {
        if (deadzone <= 0.0) return value;
        double abs = Math.abs(value);
        return (abs > 0.0 && abs <= deadzone) ? 0.0 : value;
    }

    private static double rampTowards(double current, double target, double rate, double dt) {
        if (current < target) {
            current += dt * rate;
            if (current > target) current = target;
        } else if (current > target) {
            current -= dt * rate;
            if (current < target) current = target;
        }
        return current;
    }
}

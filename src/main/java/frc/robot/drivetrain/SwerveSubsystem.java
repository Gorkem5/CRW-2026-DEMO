package frc.robot.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {

    private final double WIDTH  = Units.inchesToMeters(11.2);
    private final double HEIGHT = Units.inchesToMeters(10.5);

    private final SwerveModule fl = new SwerveModule(SwerveConstants.AngleCANID_FL, SwerveConstants.DriveCANID_FL);
    private final SwerveModule fr = new SwerveModule(SwerveConstants.AngleCANID_FR, SwerveConstants.DriveCANID_FR);
    private final SwerveModule bl = new SwerveModule(SwerveConstants.AngleCANID_BL, SwerveConstants.DriveCANID_BL);
    private final SwerveModule br = new SwerveModule(SwerveConstants.AngleCANID_BR, SwerveConstants.DriveCANID_BR);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(WIDTH,  HEIGHT),
        new Translation2d(WIDTH, -HEIGHT),
        new Translation2d(-WIDTH,  HEIGHT),
        new Translation2d(-WIDTH, -HEIGHT)
    );

    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    private final Field2d fieldVis = new Field2d();

    private final StructPublisher<Pose2d> pose2dPub =
        NetworkTableInstance.getDefault()
            .getStructTopic("/AdvantageScope/Pose2d", Pose2d.struct)
            .publish();

    private final StructPublisher<Pose3d> pose3dPub =
        NetworkTableInstance.getDefault()
            .getStructTopic("/AdvantageScope/Pose3d", Pose3d.struct)
            .publish();

    private Rotation2d heading = new Rotation2d();
    private final SwerveModulePosition[] positions = new SwerveModulePosition[4];
    private final SwerveModuleState[] states = new SwerveModuleState[4];
    private SwerveDriveOdometry odometry;

    private Pose2d currentPose = new Pose2d();

    private double xSpd = 0.0;
    private double ySpd = 0.0;
    private double lastTimeDrive = Timer.getFPGATimestamp();
    private double lastTimeOdom = Timer.getFPGATimestamp();
    private Translation3d driveReq = new Translation3d();
    private double rotReq = 0.0;

    private boolean fieldRelative = true;

    private boolean headingHoldEnabled = false;
    private boolean headingHoldOneShot = false;
    private Rotation2d headingTarget = new Rotation2d();
    private final ProfiledPIDController headingController = new ProfiledPIDController(
        DriveConstants.HeadingHoldKp,
        DriveConstants.HeadingHoldKi,
        DriveConstants.HeadingHoldKd,
        new TrapezoidProfile.Constraints(
            DriveConstants.HeadingHoldMaxVelRadPerSec,
            DriveConstants.HeadingHoldMaxAccelRadPerSecSq));

    private ChassisSpeeds lastCommandedSpeeds = new ChassisSpeeds();

    public SwerveSubsystem() {
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(Math.toRadians(DriveConstants.HeadingHoldToleranceDeg));

        updateModules();
        odometry = new SwerveDriveOdometry(kinematics, heading, positions);
        headingTarget = heading;
        SmartDashboard.putData("field", fieldVis);
    }

    public void setDriveRequest(Translation3d req) {
        driveReq = req;
        rotReq = req.getZ();
    }

    public void toggleFieldRelative() {
        fieldRelative = !fieldRelative;
    }

    public void setFieldRelative(boolean enable) {
        fieldRelative = enable;
    }

    public boolean isFieldRelative() {
        return fieldRelative;
    }

    public void requestHeadingHold(Rotation2d targetHeading) {
        requestHeadingHold(targetHeading, false);
    }

    public void requestHeadingHold(Rotation2d targetHeading, boolean oneShot) {
        headingTarget = targetHeading;
        if (!headingHoldEnabled) {
            headingController.reset(heading.getRadians());
        }
        headingHoldEnabled = true;
        headingHoldOneShot = oneShot;
    }

    public void clearHeadingHold() {
        headingHoldEnabled = false;
        headingHoldOneShot = false;
        headingTarget = heading;
        headingController.reset(heading.getRadians());
    }

    public boolean isHeadingHoldEnabled() {
        return headingHoldEnabled;
    }

    public void quickFlipHeading() {
        requestHeadingHold(heading.plus(Rotation2d.fromDegrees(180.0)), true);
    }

    public Rotation2d getHeading() {
        return heading;
    }

    private void executeDrive() {
        SmartDashboard.putNumber("xSpd", xSpd);
        SmartDashboard.putNumber("ySpd", ySpd);
        SmartDashboard.putNumber("reqX", driveReq.getX());
        SmartDashboard.putNumber("reqY", driveReq.getY());
        SmartDashboard.putNumber("reqRot", rotReq);

        double now = Timer.getFPGATimestamp();
        double dt = now - lastTimeDrive;
        lastTimeDrive = now;

        applyAccelLimit(driveReq, dt);

        if (headingHoldEnabled && Math.abs(rotReq) > DriveConstants.HeadingHoldCancelThreshold) {
            clearHeadingHold();
        }

        double vx = -xSpd * DriveConstants.MaxLinearSpeedMps;
        double vy = -ySpd * DriveConstants.MaxLinearSpeedMps;

        double omega;
        if (headingHoldEnabled) {
            omega = MathUtil.clamp(
                headingController.calculate(heading.getRadians(), headingTarget.getRadians()),
                -DriveConstants.MaxAngularRadPerSec,
                DriveConstants.MaxAngularRadPerSec);
            double headingError = headingTarget.minus(heading).getRadians();
            if (headingHoldOneShot
                && Math.abs(headingError) <= Math.toRadians(DriveConstants.HeadingHoldToleranceDeg)
                && Math.abs(omega) < DriveConstants.HeadingHoldCompletionOmegaRadPerSec) {
                clearHeadingHold();
            }
        } else {
            omega = -rotReq * DriveConstants.MaxAngularRadPerSec;
        }

        ChassisSpeeds speeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, heading)
            : new ChassisSpeeds(vx, vy, omega);
        lastCommandedSpeeds = speeds;

        SwerveModuleState[] desired = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desired, DriveConstants.MaxLinearSpeedMps);

        fl.Drive(desired[0].speedMetersPerSecond);
        fr.Drive(desired[1].speedMetersPerSecond);
        bl.Drive(desired[2].speedMetersPerSecond);
        br.Drive(desired[3].speedMetersPerSecond);

        if (Math.abs(speeds.vxMetersPerSecond)
            + Math.abs(speeds.vyMetersPerSecond)
            + Math.abs(speeds.omegaRadiansPerSecond) < 1e-4) {
            return;
        }

        fl.SetTargetAngle(desired[0].angle.getDegrees());
        fr.SetTargetAngle(desired[1].angle.getDegrees());
        bl.SetTargetAngle(desired[2].angle.getDegrees());
        br.SetTargetAngle(desired[3].angle.getDegrees());
    }

    private void applyAccelLimit(Translation3d target, double dt) {
        if (xSpd < target.getX()) {
            xSpd += dt * DriveConstants.MaxAcc_X;
            if (xSpd > target.getX()) xSpd = target.getX();
        } else if (xSpd > target.getX()) {
            xSpd -= dt * DriveConstants.MaxAcc_X;
            if (xSpd < target.getX()) xSpd = target.getX();
        }

        if (ySpd < target.getY()) {
            ySpd += dt * DriveConstants.MaxAcc_Y;
            if (ySpd > target.getY()) ySpd = target.getY();
        } else if (ySpd > target.getY()) {
            ySpd -= dt * DriveConstants.MaxAcc_Y;
            if (ySpd < target.getY()) ySpd = target.getY();
        }
    }

    private void updateModules() {
        fl.Update(0.02);
        fr.Update(0.02);
        bl.Update(0.02);
        br.Update(0.02);

        positions[0] = fl.GetPosition();
        positions[1] = fr.GetPosition();
        positions[2] = bl.GetPosition();
        positions[3] = br.GetPosition();

        states[0] = fl.GetState();
        states[1] = fr.GetState();
        states[2] = bl.GetState();
        states[3] = br.GetState();
    }

    private void updateOdometry() {
        if (currentPose == null) currentPose = new Pose2d();

        boolean useFake = DriveConstants.FAKE_ODOM || RobotBase.isSimulation();

        if (useFake) {
            double now = Timer.getFPGATimestamp();
            double dt = now - lastTimeOdom;
            if (dt <= 0 || dt > 0.1) dt = 0.02;
            lastTimeOdom = now;

            ChassisSpeeds robotRelative = lastCommandedSpeeds;
            Rotation2d priorHeading = heading;
            Translation2d velocityField = new Translation2d(
                robotRelative.vxMetersPerSecond,
                robotRelative.vyMetersPerSecond).rotateBy(priorHeading);

            double nx = currentPose.getX() + velocityField.getX() * dt;
            double ny = currentPose.getY() + velocityField.getY() * dt;
            double ntheta = priorHeading.getRadians()
                + robotRelative.omegaRadiansPerSecond * dt;

            heading = new Rotation2d(ntheta);
            currentPose = new Pose2d(nx, ny, heading);

            odometry.resetPosition(heading, positions, currentPose);
        } else {
            heading = Rotation2d.fromDegrees(-gyro.getFusedHeading());
            odometry.update(heading, positions);
            currentPose = odometry.getPoseMeters();
        }
    }

    private void updateDashboard() {
        SmartDashboard.putString("pose", "x=" + currentPose.getX() + " y=" + currentPose.getY());
        SmartDashboard.putNumber("gyro angle", heading.getDegrees());
        SmartDashboard.putBoolean("fieldRelative", fieldRelative);
        SmartDashboard.putBoolean("headingHold", headingHoldEnabled);
        SmartDashboard.putNumber("headingTargetDeg", headingTarget.getDegrees());
        fieldVis.setRobotPose(currentPose);

        pose2dPub.set(currentPose);
        pose3dPub.set(new Pose3d(
            currentPose.getX(),
            currentPose.getY(),
            0.0,
            new Rotation3d(0.0, 0.0, currentPose.getRotation().getRadians())
        ));
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(heading, positions, pose);
        headingTarget = pose.getRotation();
        headingController.reset(headingTarget.getRadians());
    }

    @Override
    public void periodic() {
        updateModules();
        updateOdometry();
        executeDrive();
        updateDashboard();
    }
}
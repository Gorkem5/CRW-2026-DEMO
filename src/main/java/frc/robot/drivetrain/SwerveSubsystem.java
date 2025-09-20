package frc.robot.drivetrain;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {

    private final double WIDTH = 11.2;
    private final double HEIGHT = 10.5;

    private SwerveModule fl = new SwerveModule(SwerveConstants.AngleCANID_FL, SwerveConstants.DriveCANID_FL);
    private SwerveModule fr = new SwerveModule(SwerveConstants.AngleCANID_FR, SwerveConstants.DriveCANID_FR);
    private SwerveModule bl = new SwerveModule(SwerveConstants.AngleCANID_BL, SwerveConstants.DriveCANID_BL);
    private SwerveModule br = new SwerveModule(SwerveConstants.AngleCANID_BR, SwerveConstants.DriveCANID_BR);

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(WIDTH, HEIGHT),
        new Translation2d(WIDTH, -HEIGHT),
        new Translation2d(-WIDTH, HEIGHT),
        new Translation2d(-WIDTH, -HEIGHT)
    );

    private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    private Field2d fieldVis = new Field2d();

    private Rotation2d heading = new Rotation2d();
    private SwerveModulePosition[] positions = new SwerveModulePosition[4];
    private SwerveModuleState[] states = new SwerveModuleState[4];
    private SwerveDriveOdometry odometry;

    private Pose2d currentPose = new Pose2d();

    private double xSpd = 0.0, ySpd = 0.0;
    private double lastTime = Timer.getFPGATimestamp();
    private Translation3d driveReq = new Translation3d();

    public SwerveSubsystem() {
        updateModules();
        odometry = new SwerveDriveOdometry(kinematics, heading, positions);
        SmartDashboard.putData("field", fieldVis);
    }

    public void setDriveRequest(Translation3d req) {
        driveReq = req;
    }

    private void executeDrive() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTime;
        lastTime = now;

        applyAccelLimit(driveReq, dt);

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            -xSpd, -ySpd, -driveReq.getZ(), heading
        );

        SwerveModuleState[] desired = kinematics.toSwerveModuleStates(speeds);

        fl.drive(desired[0].speedMetersPerSecond);
        fr.drive(desired[1].speedMetersPerSecond);
        bl.drive(desired[2].speedMetersPerSecond);
        br.drive(desired[3].speedMetersPerSecond);

        if (Math.abs(speeds.vxMetersPerSecond) +
            Math.abs(speeds.vyMetersPerSecond) +
            Math.abs(speeds.omegaRadiansPerSecond) < 1e-4) {
            return;
        }

        fl.setTargetAngle(desired[0].angle.getDegrees());
        fr.setTargetAngle(desired[1].angle.getDegrees());
        bl.setTargetAngle(desired[2].angle.getDegrees());
        br.setTargetAngle(desired[3].angle.getDegrees());
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
        fl.update(0.02);
        fr.update(0.02);
        bl.update(0.02);
        br.update(0.02);

        positions[0] = fl.getPosition();
        positions[1] = fr.getPosition();
        positions[2] = bl.getPosition();
        positions[3] = br.getPosition();

        states[0] = fl.getState();
        states[1] = fr.getState();
        states[2] = bl.getState();
        states[3] = br.getState();
    }

    private void updateOdometry() {
        heading = Rotation2d.fromDegrees(-gyro.getFusedHeading());
        odometry.update(heading, positions);
        currentPose = odometry.getPoseMeters();
    }

    private void updateDashboard() {
        SmartDashboard.putString("pose", "x=" + currentPose.getX() + " y=" + currentPose.getY());
        SmartDashboard.putNumber("gyro angle", heading.getDegrees());
        fieldVis.setRobotPose(currentPose);
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(heading, positions, pose);
    }

    @Override
    public void periodic() {
        updateModules();
        updateOdometry();
        executeDrive();
        updateDashboard();
    }
}

package frc.robot.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule fl = new SwerveModule(
        SwerveConstants.AngleCANID_FL,
        SwerveConstants.DriveCANID_FL);
    private final SwerveModule fr = new SwerveModule(
        SwerveConstants.AngleCANID_FR,
        SwerveConstants.DriveCANID_FR);
    private final SwerveModule bl = new SwerveModule(
        SwerveConstants.AngleCANID_BL,
        SwerveConstants.DriveCANID_BL);
    private final SwerveModule br = new SwerveModule(
        SwerveConstants.AngleCANID_BR,
        SwerveConstants.DriveCANID_BR);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(SwerveConstants.HALF_WHEELBASE_METERS, SwerveConstants.HALF_TRACKWIDTH_METERS),
        new Translation2d(SwerveConstants.HALF_WHEELBASE_METERS, -SwerveConstants.HALF_TRACKWIDTH_METERS),
        new Translation2d(-SwerveConstants.HALF_WHEELBASE_METERS, SwerveConstants.HALF_TRACKWIDTH_METERS),
        new Translation2d(-SwerveConstants.HALF_WHEELBASE_METERS, -SwerveConstants.HALF_TRACKWIDTH_METERS)
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

    private final SwerveModulePosition[] positions = new SwerveModulePosition[4];
    private final SwerveModuleState[] states = new SwerveModuleState[4];
    private final SwerveDrivePoseEstimator estimator;

    private final Translation2d fieldCenter = computeFieldCenter();

    private Pose2d currentPose = new Pose2d();
    private Rotation2d simHeading = new Rotation2d();
    private ChassisSpeeds driveSetpoint = new ChassisSpeeds();

    private double lastSimTimestamp = Timer.getFPGATimestamp();

    public SwerveSubsystem() {
        refreshModuleMeasurements();
        estimator = new SwerveDrivePoseEstimator(kinematics, getHeading(), positions, currentPose);
        fieldVis.setRobotPose(currentPose);
    }

    public void driveRobotRelative(ChassisSpeeds robotSpeeds) {
        driveSetpoint = robotSpeeds;
        applyDriveSetpoint();
    }

    public void driveFieldRelative(ChassisSpeeds fieldSpeeds) {
        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldSpeeds.vxMetersPerSecond,
            fieldSpeeds.vyMetersPerSecond,
            fieldSpeeds.omegaRadiansPerSecond,
            getHeading());
        driveRobotRelative(robotSpeeds);
    }

    public void stop() {
        driveRobotRelative(new ChassisSpeeds());
    }

    public Rotation2d getHeading() {
        if (RobotBase.isSimulation()) {
            return simHeading;
        }
        return Rotation2d.fromDegrees(-gyro.getFusedHeading());
    }

    public Pose2d getPose() {
        return currentPose;
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> visionStdDevs) {
        estimator.addVisionMeasurement(visionPose, timestampSeconds, visionStdDevs);
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, double positionStdDevMeters, double rotationStdDevRad) {
        estimator.addVisionMeasurement(visionPose, timestampSeconds, VecBuilder.fill(positionStdDevMeters, positionStdDevMeters, rotationStdDevRad));
    }

    public void resetPose(Pose2d pose) {
        estimator.resetPosition(getHeading(), positions, pose);
        currentPose = pose;
        if (RobotBase.isSimulation()) {
            simHeading = pose.getRotation();
        }
    }

    public SwerveModulePosition[] getModulePositions() {
        return positions.clone();
    }

    private static Translation2d computeFieldCenter() {
        try {
            AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
            return new Translation2d(layout.getFieldLength() / 2.0, layout.getFieldWidth() / 2.0);
        } catch (Exception ex) {
            return new Translation2d();
        }
    }

    public Rotation2d getHeadingToFieldCenter() {
        Translation2d centerDelta = fieldCenter.minus(currentPose.getTranslation());
        if (centerDelta.getNorm() < 1e-6) {
            return currentPose.getRotation();
        }
        return new Rotation2d(centerDelta.getX(), centerDelta.getY());
    }

    @Override
    public void periodic() {
        refreshModuleMeasurements();
        currentPose = estimator.update(getHeading(), positions);
        applyDriveSetpoint();
        fieldVis.setRobotPose(currentPose);

        pose2dPub.set(currentPose);
        pose3dPub.set(new Pose3d(
            currentPose.getX(),
            currentPose.getY(),
            0.0,
            new Rotation3d(0.0, 0.0, currentPose.getRotation().getRadians())
        ));
    }

    @Override
    public void simulationPeriodic() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastSimTimestamp;
        if (dt <= 0.0 || dt > 0.1) {
            dt = 0.02;
        }
        lastSimTimestamp = now;

        simHeading = Rotation2d.fromRadians(simHeading.getRadians() + driveSetpoint.omegaRadiansPerSecond * dt);

        fl.Update(dt);
        fr.Update(dt);
        bl.Update(dt);
        br.Update(dt);
    }

    private void refreshModuleMeasurements() {
        positions[0] = fl.GetPosition();
        positions[1] = fr.GetPosition();
        positions[2] = bl.GetPosition();
        positions[3] = br.GetPosition();

        states[0] = fl.GetState();
        states[1] = fr.GetState();
        states[2] = bl.GetState();
        states[3] = br.GetState();
    }

    private void applyDriveSetpoint() {
        SwerveModuleState[] desired = kinematics.toSwerveModuleStates(driveSetpoint);
        SwerveDriveKinematics.desaturateWheelSpeeds(desired, SwerveConstants.MAX_MODULE_SPEED_MPS);

        fl.SetTargetAngle(desired[0].angle.getDegrees());
        fr.SetTargetAngle(desired[1].angle.getDegrees());
        bl.SetTargetAngle(desired[2].angle.getDegrees());
        br.SetTargetAngle(desired[3].angle.getDegrees());

        fl.Drive(desired[0].speedMetersPerSecond);
        fr.Drive(desired[1].speedMetersPerSecond);
        bl.Drive(desired[2].speedMetersPerSecond);
        br.Drive(desired[3].speedMetersPerSecond);
    }
}

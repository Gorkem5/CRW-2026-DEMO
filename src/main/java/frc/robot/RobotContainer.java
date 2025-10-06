package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.DriveConstants;
import frc.robot.drivetrain.SwerveSubsystem;
import frc.robot.drivetrain.commands.SwerveJoystickDriveCommand;

public class RobotContainer {
  private final SwerveSubsystem swerve = new SwerveSubsystem();

  private final CommandPS4Controller driver = new CommandPS4Controller(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
        new SwerveJoystickDriveCommand(
            swerve,
            () -> shape(deadzone(-driver.getHID().getLeftY(),  DriveConstants.JOYDeadzone_Y)) * -1.0,
            () -> shape(deadzone( driver.getHID().getLeftX(),  DriveConstants.JOYDeadzone_X)),
            () -> shape(deadzone( driver.getHID().getRawAxis(4), DriveConstants.JOYDeadzone_Rot)),
            true));

    driver.options()
        .onTrue(new InstantCommand(swerve::toggleFieldRelative, swerve));

    driver.square()
        .onTrue(new InstantCommand(swerve::quickFlipHeading, swerve));

    driver.triangle()
        .onTrue(new InstantCommand(
            () -> swerve.requestHeadingHold(Rotation2d.fromDegrees(0.0)),
            swerve));

    driver.cross()
        .onTrue(new InstantCommand(swerve::clearHeadingHold, swerve));
  }

  private static double deadzone(double v, double dz) {
    return (Math.abs(v) < dz) ? 0.0 : v;
  }

  private static double shape(double v) {
    return Math.copySign(v * v, v);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
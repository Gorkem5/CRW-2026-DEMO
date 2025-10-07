package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.drivetrain.SwerveSubsystem;
import frc.robot.drivetrain.commands.SwerveJoystickDriveCommand;

public class RobotContainer {
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final CommandPS4Controller driver = new CommandPS4Controller(0);
  private final SwerveJoystickDriveCommand driveCommand = new SwerveJoystickDriveCommand(
      swerve,
      () -> -driver.getHID().getLeftY(),
      () -> driver.getHID().getLeftX(),
      () -> driver.getHID().getRawAxis(4),
      true);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(driveCommand);

    driver.options().onTrue(Commands.runOnce(driveCommand::toggleFieldRelative));
    driver.square().onTrue(Commands.runOnce(driveCommand::quickFlipHeading));
    driver.triangle().onTrue(Commands.runOnce(
        () -> driveCommand.requestHeadingHold(Rotation2d.fromDegrees(0.0), false)));
    driver.cross().onTrue(Commands.runOnce(driveCommand::clearHeadingHold));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}

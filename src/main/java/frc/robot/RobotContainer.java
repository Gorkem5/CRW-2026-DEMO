// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.drivetrain.SwerveSubsystem;
import frc.robot.drivetrain.commands.SwerveJoystickDriveCommand;
import frc.robot.Constants.DriveConstants;

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
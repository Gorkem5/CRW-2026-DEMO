// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.drivetrain.SwerveSubsystem;
import frc.robot.drivetrain.commands.SwerveJoystickDriveCommand;


public class RobotContainer {
  // sub,command define
  private final SwerveSubsystem swerve = new SwerveSubsystem();

  private final CommandPS4Controller m_driverController = new CommandPS4Controller(0);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(new SwerveJoystickDriveCommand(
      swerve,
      () -> m_driverController.getHID().getLeftX(),
      () -> m_driverController.getHID().getLeftY(),
      () -> m_driverController.getHID().getRightX(), 
      true
    )); 
  }
  
  public Command getAutonomousCommand() {
    return null;
  }
  
}
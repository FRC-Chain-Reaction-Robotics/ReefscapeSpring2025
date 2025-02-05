// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private SwerveSubsystem drivebase = new SwerveSubsystem();
  final CommandXboxController driverXbox = new CommandXboxController(0);
  private final SendableChooser<Command> autoChooser;
  public DigitalInput input;

  Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), 0.15),
      () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), 0.15),
      () -> MathUtil.applyDeadband(-driverXbox.getRightX(), 0.15),
      () -> MathUtil.applyDeadband(-driverXbox.getRightY(), 0.15)
    );

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
      () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), 0.15),
      () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), 0.15),
      () -> MathUtil.applyDeadband(-driverXbox.getRightX(), 0.15),
      () -> MathUtil.applyDeadband(-driverXbox.getRightY(), 0.15)
    );

  public RobotContainer() {
    // autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser = null;
    // or alternatively to select a default use
    // autoChooser = AutoBuilder.buildAutoChooser("Default Auto");
    // NamedCommands.registerCommand(null, driveFieldOrientedDirectAngleSim);
    configureBindings();
    // SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleSim);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  // private SwerveSubsystem drivebase = new SwerveSubsystem();
  // private Elevator elevator = new Elevator();
  private Coral coral = new Coral();
  final CommandXboxController driverXbox = new CommandXboxController(0);
  private final SendableChooser<Command> autoChooser;
  final CommandXboxController operatorXbox = new CommandXboxController(1);

  // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
  // () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), 0.05),
  // () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), 0.05),
  // () -> MathUtil.applyDeadband(-driverXbox.getRightX(), 0.05)
  // // ,() -> MathUtil.applyDeadband(-driverXbox.getRightY(), 0.05)
  // );

  // Command driveFieldOrientedDirectAngleSim = drivebase.driveCommand(
  // () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), 0.05),
  // () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), 0.05),
  // () -> MathUtil.applyDeadband(-driverXbox.getRightX(), 0.05)
  // // ,() -> MathUtil.applyDeadband(-driverXbox.getRightY(), 0.05)
  // );

  public RobotContainer() {
    // autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser = null;
    // NamedCommands.registerCommand(null, driveFieldOrientedDirectAngleSim);
    configureBindings();
    // SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // Swerve Drivebase configuaration
    // if (RobotBase.isSimulation()) {
    // drivebase.setDefaultCommand(driveFieldOrientedDirectAngleSim);
    // } else {
    // drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    // }
    Trigger upTrigger, downTrigger;
    upTrigger = driverXbox.povUp();
    downTrigger = driverXbox.povDown();
    // Elevator configuration
    System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAa");
    
    // elevator.setDefaultCommand(elevator.elevatorCommand(upTrigger, downTrigger));
    coral.setDefaultCommand(coral.coralPivotCommand(() -> { return driverXbox.getRightX(); }));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private SwerveSubsystem drivebase = new SwerveSubsystem();
  private Elevator elevator = new Elevator();
  private Algae algae = new Algae();
  // private Coral coral = new Coral();
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);

  Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
  () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), 0.05),
  () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), 0.05),
  () -> MathUtil.applyDeadband(-driverXbox.getRightX(), 0.05)
  // ,() -> MathUtil.applyDeadband(-driverXbox.getRightY(), 0.05)
  );

  Command driveFieldOrientedDirectAngleSim = drivebase.driveCommand(
  () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), 0.05),
  () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), 0.05),
  () -> MathUtil.applyDeadband(-driverXbox.getRightX(), 0.05)
  // ,() -> MathUtil.applyDeadband(-driverXbox.getRightY(), 0.05)
  );

  public RobotContainer() {
    // autoChooser = AutoBuilder.buildAutoChooser();
    // NamedCommands.registerCommand(null, driveFieldOrientedDirectAngleSim);
    LimelightHelpers.setPipelineIndex(Constants.Limelight.NAME, 0);
    configureBindings();
    // SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // Swerve Drivebase configuaration
    if (RobotBase.isSimulation()) {
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngleSim);
    } else {
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    }
    Trigger upElevTrigger, downElevTrigger, upAlgaeTrigger, downAlgaeTrigger, inAlgaeTrigger, outAlgaeTrigger;
    upElevTrigger = driverXbox.povUp();
    downElevTrigger = driverXbox.povDown();
    upAlgaeTrigger = operatorXbox.povUp();
    downAlgaeTrigger = operatorXbox.povDown();

    inAlgaeTrigger = operatorXbox.a();
    outAlgaeTrigger = operatorXbox.b();

    // Elevator configuration

    algae.setDefaultCommand(algae.defaultCommand(upAlgaeTrigger, downAlgaeTrigger, inAlgaeTrigger, outAlgaeTrigger));

    elevator.setDefaultCommand(elevator.elevatorCommand(upElevTrigger, downElevTrigger));
    // driverXbox.a().whileTrue(coral.dynamicTestCommF());
    // driverXbox.b().whileTrue(coral.dynamicTestCommR());
    // driverXbox.x().whileTrue(coral.quasistaticTestCommF());
    // driverXbox.y().whileTrue(coral.quasistaticTestCommR());
    // driverXbox.a().whileTrue(coral.coralPivotCommand(driverXbox.b()));
    // driverXbox.a().onFalse(new InstantCommand(() -> {coral.pivotOff();}));
    // driverXbox.x().onTrue(new InstantCommand(() -> {
    //   coral.resetEncoder();
    // }));
  }

  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    return drivebase.simpleAutoCommand();
  }
}

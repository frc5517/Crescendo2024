// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final IntakeSubsystem intakebase = new IntakeSubsystem();
  private final ArmSubsystem armbase = new ArmSubsystem();

  CommandXboxController driverXbox = new CommandXboxController(0);
  CommandXboxController operatorXbox = new CommandXboxController(1);

  PhotonCamera camera = new PhotonCamera(Constants.Vision.kCameraName);

  public RobotContainer() {
    configureBindings();

    // Register the commands for FRC PathPlanner
    NamedCommands.registerCommand("Raise Arm", armbase.ArmCommandForTime(.3, 1));
    NamedCommands.registerCommand("Lower Arm", armbase.ArmCommandForTime(-.3, 1));
    NamedCommands.registerCommand("Shoot High", intakebase.ShootCommandForTime(1, .7, 1, 2));
    NamedCommands.registerCommand("Shoot Low", intakebase.ShootCommandForTime(.5, .3, 1, 2));
    NamedCommands.registerCommand("Aim at note", drivebase.aimAtTargetForTime(camera, 1));

    // Creating the robot centric swerve drive
    Command closedDrive = drivebase.driveCommand(false, 
    () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_Y_DEADBAND), 
    () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_X_DEADBAND), 
    () -> MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
    driverXbox.getHID()::getAButton,
    driverXbox.getHID()::getBButton);

    // Creating the field centric swerve drive
    Command fieldDrive = drivebase.driveCommand(true, 
    () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_Y_DEADBAND), 
    () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_X_DEADBAND), 
    () -> MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
    driverXbox.leftBumper(),
    driverXbox.rightBumper());

    drivebase.setDefaultCommand(fieldDrive); // Set default drive command to field centric drive

    driverXbox.rightTrigger().toggleOnTrue(closedDrive); // Toggle robot centric swerve drive
    driverXbox.start().onTrue(new InstantCommand(drivebase::lock));    // Lock drive train to limit pushing
    driverXbox.back().onTrue(new InstantCommand(drivebase::zeroGyro)); // Zero the gyro to avoid odd drive due to gyro drift
    driverXbox.leftTrigger().whileTrue(drivebase.aimAtTarget(camera)); // Look at the note

    operatorXbox.y().whileTrue(armbase.ArmCommand(.3)); // Raise the arm
    operatorXbox.a().whileTrue(armbase.ArmCommand(-.3));  // Lower the arm
    operatorXbox.b().whileTrue(intakebase.IntakeCommand(-.7));  // Outtake the note
    operatorXbox.leftBumper().whileTrue(intakebase.IntakeCommand(.7));  // Intake the note
    operatorXbox.x().whileTrue(intakebase.ShootCommand(.6, .5, 0)); // Spit the note into the amp
    operatorXbox.rightBumper().whileTrue(intakebase.ShootCommand(1, .7, 1));  // Shoot the note into the speaker
  }

  public void configureBindings() {}

  public void teleopInit() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.AbsoluteDrive;
import frc.robot.commands.swervedrive.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.AbsoluteFieldDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final ArmSubsystem armbase = new ArmSubsystem();
  private final IntakeSubsystem intakebase = new IntakeSubsystem();

  XboxController driverXbox = new XboxController(0);
  XboxController operatorXbox = new XboxController(1);

  SendableChooser<Command> swerveChooser = new SendableChooser<>();
  SendableChooser<Command> autonChooser = new SendableChooser<>();


  public RobotContainer() {
    

    configureBindings();

    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase,
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), 
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());

    AbsoluteFieldDrive absoluteFieldDrive = new AbsoluteFieldDrive(drivebase, 
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX());

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase, 
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX(), 
        driverXbox::getYButtonPressed, 
        driverXbox::getAButtonPressed, 
        driverXbox::getXButtonPressed,
        driverXbox::getBButtonPressed);

    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    Command closedDrive = drivebase.driveCommand(
        () -> driverXbox.getLeftY(), 
        () -> driverXbox.getLeftX(), 
        () -> driverXbox.getRightX());

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);

    swerveChooser.addOption("Closed Drive", closedDrive);
    swerveChooser.addOption("Closed Absolute Drive", closedAbsoluteDrive);
    swerveChooser.addOption("Field Absolute Drive", absoluteFieldDrive);
    swerveChooser.setDefaultOption("Closed Absolute Drive Adv", closedAbsoluteDriveAdv);
    swerveChooser.addOption("Field Direct Angle Drive", driveFieldOrientedDirectAngle);
    swerveChooser.addOption("Field Angular Velocity Drive", driveFieldOrientedAnglularVelocity); 

    new JoystickButton(driverXbox, 8).toggleOnTrue(new InstantCommand(drivebase::lock));    // Lock drive train toggle
    new JoystickButton(driverXbox, 5).whileTrue(Commands.deferredProxy(
        () -> drivebase.driveToPose(new Pose2d(new Translation2d(4, 4),Rotation2d.fromDegrees(0)))));

    new JoystickButton(operatorXbox, 1).whileTrue(armbase.ArmCommand(-.3));       // Lower arm while held
    new JoystickButton(operatorXbox, 3).whileTrue(armbase.ArmCommand(.3));  // Raise arm while held

    new JoystickButton(operatorXbox, 5).whileTrue(intakebase.IntakeCommand(.5));                                // Intake while held
    new JoystickButton(operatorXbox, 6).whileTrue(intakebase.ShootCommand(.7, .3, 2));  // Fast shoot for speaker while held
    new JoystickButton(operatorXbox, 2).whileTrue(intakebase.ShootCommand(.5, .3, 0));  // Slow shoot for amp while held

  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.AbsoluteDrive;
import frc.robot.commands.swervedrive.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.AbsoluteFieldDrive;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  XboxController driverXbox = new XboxController(0);

  PhotonCamera camera = new PhotonCamera("Camera");

  SendableChooser<Command> swerveChooser = new SendableChooser<>();
  SendableChooser<Command> autonChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();

    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase, 
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), 
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        () -> driverXbox.getRightY());

    AbsoluteFieldDrive absoluteFieldDrive = new AbsoluteFieldDrive(drivebase, 
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getRawAxis(2), OperatorConstants.RIGHT_X_DEADBAND));

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase, 
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        driverXbox::getYButtonPressed, 
        driverXbox::getAButtonPressed, 
        driverXbox::getXButtonPressed,
        driverXbox::getBButtonPressed);

    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        () -> driverXbox.getRightY());

    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND));

    swerveChooser.addOption("Closed Absolute Drive", closedAbsoluteDrive);
    swerveChooser.addOption("Field Absolute Drive", absoluteFieldDrive);
    swerveChooser.setDefaultOption("Closed Absolute Drive Adv", closedAbsoluteDriveAdv);
    swerveChooser.addOption("Field Direct Angle Drive", driveFieldOrientedDirectAngle);
    swerveChooser.addOption("Field Angular Velocity Drive", driveFieldOrientedAnglularVelocity);

    SmartDashboard.putData(swerveChooser);

    drivebase.setDefaultCommand(swerveChooser.getSelected());

    new JoystickButton(driverXbox, 8).toggleOnTrue(new InstantCommand(drivebase::lock));    // Lock drive train toggle
    new JoystickButton(driverXbox, 5).whileTrue(drivebase.aimAtTarget(camera));

  }

  public void configureBindings() {}

  public void teleopInit() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
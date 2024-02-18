// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.DoubleSupplier;

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
import frc.robot.commands.swervedrive.AbsoluteFieldDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final IntakeSubsystem intakebase = new IntakeSubsystem();
  private final ArmSubsystem armbase = new ArmSubsystem();

  XboxController driverXbox = new XboxController(0);
  XboxController operatorXbox = new XboxController(1);

  PhotonCamera camera = new PhotonCamera("Camera");

  SendableChooser<Command> swerveChooser = new SendableChooser<>();
  SendableChooser<Command> autonChooser = new SendableChooser<>();
  SendableChooser<Double> speedChooser = new SendableChooser<>();

  double maxSpeed;

  public void updateSpeed() {
    speedChooser.addOption("0 Percent", 0.0);
    speedChooser.addOption("25 Percent", .25);
    speedChooser.addOption("50 Percent", .5);
    speedChooser.setDefaultOption("75 Percent", .75);
    speedChooser .addOption("100 Percent", 1.0);

    SmartDashboard.putData(speedChooser);

    double maxSpeed = speedChooser.getSelected();
    this.maxSpeed = maxSpeed;
  }

  public RobotContainer() {
    configureBindings();

    DoubleSupplier vX = () -> MathUtil.applyDeadband(-driverXbox.getLeftX() * maxSpeed, OperatorConstants.LEFT_Y_DEADBAND);
    DoubleSupplier vY = () -> MathUtil.applyDeadband(-driverXbox.getLeftY() * maxSpeed, OperatorConstants.LEFT_X_DEADBAND);
    DoubleSupplier headingHorizontal = () -> MathUtil.applyDeadband(-driverXbox.getRightX() * maxSpeed, OperatorConstants.RIGHT_X_DEADBAND);
    DoubleSupplier headingVertical = () -> -driverXbox.getRightY() * maxSpeed;

    AbsoluteDrive closedAbsoluteDrive = new AbsoluteDrive(drivebase, vX, vY, headingHorizontal, headingVertical);
    AbsoluteFieldDrive absoluteFieldDrive = new AbsoluteFieldDrive(drivebase, vX, vY, headingHorizontal);
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(vY, vX, headingHorizontal, headingVertical);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(vY, vX, headingHorizontal);

    swerveChooser.addOption("Closed Absolute Drive", closedAbsoluteDrive);
    swerveChooser.addOption("Field Absolute Drive", absoluteFieldDrive);
    swerveChooser.addOption("Field Direct Angle Drive", driveFieldOrientedDirectAngle);
    swerveChooser.setDefaultOption("Field Angular Velocity Drive", driveFieldOrientedAnglularVelocity);

    SmartDashboard.putData(swerveChooser);

    new JoystickButton(driverXbox, 8).toggleOnTrue(new InstantCommand(drivebase::lock));    // Lock drive train toggle
    new JoystickButton(driverXbox, 5).whileTrue(drivebase.aimAtTarget(camera));

    new JoystickButton(operatorXbox, 3).whileTrue(armbase.ArmCommand(.3));
    new JoystickButton(operatorXbox, 1).whileTrue(armbase.ArmCommand(-.3));
    new JoystickButton(operatorXbox, 2).whileTrue(intakebase.IntakeCommand(-.7));
    new JoystickButton(operatorXbox, 5).whileTrue(intakebase.IntakeCommand(.7));
    new JoystickButton(operatorXbox, 3).whileTrue(intakebase.ShootCommand(.6, .5, 0));
    new JoystickButton(operatorXbox, 6).whileTrue(intakebase.ShootCommand(1, .7, 1));
  }

  public void updateSwerve() {
    drivebase.setDefaultCommand(swerveChooser.getSelected());
  }

  public void configureBindings() {}

  public void teleopInit() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
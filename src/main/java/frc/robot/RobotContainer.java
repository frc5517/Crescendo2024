// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
  double maxSpeed;

  SendableChooser<Command> autonChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();

    NamedCommands.registerCommand("Lower Arm", armbase.ArmCommandForTime(-.3, 1));
    NamedCommands.registerCommand("Shoot High", intakebase.ShootCommand(1, .7, 1));

    Command closedDrive = drivebase.driveCommand(false, 
    () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_Y_DEADBAND), 
    () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_X_DEADBAND), 
    () -> MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
    driverXbox.getHID()::getAButton,
    driverXbox.getHID()::getBButton);

    Command fieldDrive = drivebase.driveCommand(true, 
    () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_Y_DEADBAND), 
    () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_X_DEADBAND), 
    () -> MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
    driverXbox.leftBumper(),
    driverXbox.rightBumper());

    drivebase.setDefaultCommand(fieldDrive);

    driverXbox.rightTrigger().toggleOnTrue(closedDrive);
    driverXbox.start().toggleOnTrue(new InstantCommand(drivebase::lock));    // Lock drive train toggle
    driverXbox.back().onTrue(new InstantCommand(drivebase::zeroGyro));
    driverXbox.leftTrigger().whileTrue(drivebase.aimAtTarget(camera));
    driverXbox.a().whileTrue(drivebase.getAutonomousCommand("Shoot and Park"));

    operatorXbox.y().whileTrue(armbase.ArmCommand(.3));
    operatorXbox.a().whileTrue(armbase.ArmCommand(-.3));
    operatorXbox.b().whileTrue(intakebase.IntakeCommand(-.7));
    operatorXbox.leftBumper().whileTrue(intakebase.IntakeCommand(.7));
    operatorXbox.x().whileTrue(intakebase.ShootCommand(.6, .5, 0));
    operatorXbox.rightBumper().whileTrue(intakebase.ShootCommand(1, .7, 1));
  }

  public void configureBindings() {}

  public void teleopInit() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
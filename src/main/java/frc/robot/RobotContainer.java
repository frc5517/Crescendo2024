// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final IntakeSubsystem intakebase = new IntakeSubsystem();
  private final ClimberSubsystem climbbase = new ClimberSubsystem();
  private final ArmSubsystem armbase = new ArmSubsystem();

  private final SendableChooser<Command> autoChooser;

  CommandXboxController driverXbox = new CommandXboxController(0);
  CommandXboxController operatorXbox = new CommandXboxController(1);

  PhotonCamera camera = new PhotonCamera(Constants.Vision.kCameraName);

  public RobotContainer() {
    configureBindings();
    // Register the commands for FRC PathPlanner
    NamedCommands.registerCommand("Raise Arm", armbase.ArmCommandForTime(.3, 1));
    NamedCommands.registerCommand("Lower Arm", armbase.ArmCommandForTime(-.3, 1));
    NamedCommands.registerCommand("Move To Setpoint", armbase.moveToSetpoint(.3, 30).withTimeout(2));
    NamedCommands.registerCommand("Shoot High", intakebase.ShootCommandForTime(1, .7, 1, 1));
    NamedCommands.registerCommand("Shoot Low", intakebase.ShootCommandForTime(.5, .3, 1, 2));
    NamedCommands.registerCommand("Aim at Note", drivebase.aimAtTargetForTime(camera, 1));
    NamedCommands.registerCommand("Intake Note",intakebase.IntakeWithSensor(.7));

    autoChooser = AutoBuilder.buildAutoChooser(); // Builds auton sendable chooser for pathplanner.
    SmartDashboard.putData(autoChooser);  // Sends autoBuilder to smartdashboard.

    // Creating the robot centric swerve drive
    Command closedDrive = drivebase.driveCommand(false, 
    () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), 
    () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), 
    () -> MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
    driverXbox.leftBumper(),
    driverXbox.rightBumper());

    // Creating the field centric swerve drive
    Command fieldDrive = drivebase.driveCommand(true, 
    () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), 
    () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), 
    () -> MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
    driverXbox.leftBumper(),  // Left bumper for slow speed
    driverXbox.rightBumper());  // Right bumper for high speed

    drivebase.setDefaultCommand(fieldDrive); // Set default drive command to field centric drive

    driverXbox.leftTrigger(.3).toggleOnTrue(closedDrive); // Toggle robot centric swerve drive
    driverXbox.start().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());    // Lock drive train to limit pushing
    driverXbox.back().onTrue(new InstantCommand(drivebase::zeroGyro)); // Zero the gyro to avoid odd drive due to gyro drift
    //driverXbox.rightTrigger(.3).whileTrue(drivebase.aimAtTarget(camera)); // Look at the note
    driverXbox.y().whileTrue(Commands.deferredProxy(() -> drivebase.driveToPose // Drive to position on field
    (new Pose2d(new Translation2d(1.47, 5.55), Rotation2d.fromDegrees(90)))));

    driverXbox.rightTrigger(.3).whileTrue(drivebase.aimAtTargetNew(camera, 
    () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), 
    () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND)));


    operatorXbox.y().whileTrue(armbase.ArmCommand(.3)); // Raise the arm
    operatorXbox.a().whileTrue(armbase.ArmCommand(-.3));  // Lower the arm
    operatorXbox.b().whileTrue(intakebase.IntakeCommand(-.3));  // Outtake the note
    operatorXbox.leftBumper().whileTrue(intakebase.IntakeWithSensor(.6));  // Intake the note
    operatorXbox.x().whileTrue(intakebase.ShootCommand(.6, .5, .2)); // Spit the note into the amp
    operatorXbox.rightBumper().whileTrue(intakebase.ShootCommand(1, .7, 1));  // Shoot the note into the speaker
    operatorXbox.start().whileTrue(climbbase.climberCommand(.8)); // Spin the climb motor forwards.
    operatorXbox.back().whileTrue(climbbase.climberCommand(-.8)); // Spin the climb motor in reverse.
    operatorXbox.pov(0).whileTrue(armbase.moveToSetpoint(.3, 30));  // Move the arm to setpoint // When held will oscillate around setpoint
  }

  public void configureBindings() {}

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
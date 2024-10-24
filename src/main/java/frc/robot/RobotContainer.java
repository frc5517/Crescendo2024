// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSubsytem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RobotContainer {
  // Creates the subsystems
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final IntakeSubsystem intakebase = new IntakeSubsystem();
  private final ClimberSubsystem climbbase = new ClimberSubsystem();
  private final ArmSubsytem armbase = new ArmSubsytem();

  // Creates the auton sendable chooser
  private final SendableChooser<Command> autoChooser;

  

  // Creates the controllers
  CommandXboxController operatorXbox = new CommandXboxController(1);

  Joystick driverStick = new Joystick(0);
    JoystickButton 
    button0 = new JoystickButton(driverStick, 0),
    button1 = new JoystickButton(driverStick, 1),
    button2 = new JoystickButton(driverStick, 2),
    button3 = new JoystickButton(driverStick, 3),
    button4 = new JoystickButton(driverStick, 4),
    button5 = new JoystickButton(driverStick, 5),
    button6 = new JoystickButton(driverStick, 6);

  // Creates the photon camera
  PhotonCamera camera = new PhotonCamera("Camera");

  public RobotContainer() {
    configureBindings();
    // Register the commands for FRC PathPlanner
    NamedCommands.registerCommand("Raise Arm", armbase.MoveToSetpoint(15).withTimeout(1));
    NamedCommands.registerCommand("Lower Arm", armbase.MoveToSetpoint(.2).withTimeout(1));
    NamedCommands.registerCommand("Move To Setpoint", armbase.MoveToSetpoint(6).withTimeout(2));
    NamedCommands.registerCommand("Shoot High", intakebase.ShootCommand(1, .7, .5).withTimeout(2));
    NamedCommands.registerCommand("Shoot Low", intakebase.ShootCommand(.5, .3, .2).withTimeout(2));
    //NamedCommands.registerCommand("Aim at Note", drivebase.aimAtTarget(camera).withTimeout(2));
    NamedCommands.registerCommand("Intake Note",intakebase.IntakeWithSensor(.7).withTimeout(2));
    NamedCommands.registerCommand("Outdex Slightly", intakebase.IntakeCommand(-.3).withTimeout(.25));
    NamedCommands.registerCommand("Arm Encoder Up", new InstantCommand(armbase::ArmEncoderUp));
    NamedCommands.registerCommand("Reset IMU", new InstantCommand(drivebase::zeroGyro));

    autoChooser = AutoBuilder.buildAutoChooser(); // Builds auton sendable chooser for pathplanner.
    SmartDashboard.putData(autoChooser);  // Sends autoBuilder to smartdashboard.

    if (Robot.isReal()) {
    CameraServer.startAutomaticCapture().setVideoMode(PixelFormat.kMJPEG, 480, 320, 10);}

    // Creating the robot centric swerve drive
    Command closedDrive = drivebase.driveCommand(false, 
    () -> MathUtil.applyDeadband(-driverStick.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND), 
    () -> MathUtil.applyDeadband(-driverStick.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND), 
    () -> MathUtil.applyDeadband(-driverStick.getRawAxis(2), OperatorConstants.RIGHT_X_DEADBAND),
    () -> -driverStick.getRawAxis(3));

    // Creating the field centric swerve drive
    Command fieldDrive = drivebase.driveCommand(true, 
    () -> MathUtil.applyDeadband(-driverStick.getY(), OperatorConstants.LEFT_Y_DEADBAND), 
    () -> MathUtil.applyDeadband(-driverStick.getX(), OperatorConstants.LEFT_X_DEADBAND), 
    () -> MathUtil.applyDeadband(-driverStick.getTwist(), OperatorConstants.RIGHT_X_DEADBAND),
    () -> -driverStick.getRawAxis(3));

    drivebase.setDefaultCommand(fieldDrive); // Set default drive command to field centric drive

    // Driver Controls
    button1.toggleOnTrue(closedDrive); // Toggle robot centric swerve drive
    button3.whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());    // Lock drive train to limit pushing
    button4.onTrue(new InstantCommand(drivebase::zeroGyro)); // Zero the gyro to avoid odd drive due to gyro drift

    // Operator Controls
    operatorXbox.y().whileTrue(armbase.ArmCommand(.3));  // Raise the arm
    operatorXbox.a().whileTrue(armbase.ArmCommand(-.3)); // Lower the arm
    operatorXbox.b().whileTrue(intakebase.IntakeCommand(-.3));  // Outtake the note
    operatorXbox.leftBumper().whileTrue(intakebase.IntakeCommand(.6));  // Intake the note
    operatorXbox.x().whileTrue(intakebase.ShootCommand(.6, .5, .2)); // Spit the note into the amp
    operatorXbox.rightBumper().whileTrue(intakebase.ShootCommand(1, .7, .7));  // Shoot the note into the speaker
    operatorXbox.start().whileTrue(climbbase.ClimbCommand(1)); // Spin the climb motor forwards.
    operatorXbox.back().whileTrue(climbbase.ClimbCommand(-1)); // Spin the climb motor in reverse. 
    operatorXbox.pov(0).whileTrue(armbase.MoveToSetpoint(6)); // Move the arm to setpoint // When held will oscillate around setpoint
  }

  public void configureBindings() {}

  public Command getAutonomousCommand() {
    return autoChooser.getSelected(); // Gets Auton
  }
}
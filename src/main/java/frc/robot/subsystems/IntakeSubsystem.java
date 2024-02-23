// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  CANSparkMax intakeMotor = new CANSparkMax(ManipulatorConstants.intakeMotorPort, MotorType.kBrushed);
  CANSparkMax shooterMotor = new CANSparkMax(ManipulatorConstants.shooterMotorPort, MotorType.kBrushed);
  DigitalInput noteSensor = new DigitalInput(0);

  /**
   * Initialize {@link IntakeSubsystem} with idle modes.
   */
  public IntakeSubsystem() {
    intakeMotor.setInverted(false);
    shooterMotor.setInverted(true);
  }

  @Override
  public void periodic() {}

  /**
   * Run the intake at set speed.
   * @param speed to intake note.
   * @return A {@link Command} to run the intake.
   */
  public Command IntakeCommand(Double speed)
  {
    return runEnd(() -> { 
      intakeMotor.set(speed); // intake note at "speed".
    }, () -> {
      intakeMotor.stopMotor(); // stop motor when done.
    }
   );
  }

  /**
   * Run the intake at set speed for set time.
   * @param speed to intake note.
   * @param time to run command for.
   * @return A {@link Command} to run the intake for set time.
   */
  public Command IntakeCommandForTime(Double speed, double time)
  {
    return runEnd(() -> {
      intakeMotor.set(speed); // intake note at "speed".
      Timer.delay(time);  // keep running for "time".
    }, () -> {
      intakeMotor.stopMotor();  // stop intake when done.
    }
   );
  }

  /**
   * Shoot the note at given speeds and ramp up time.
   * @param shooterSpeed to shoot the note.
   * @param intakeSpeed to index into the shooter.
   * @param time to ramp up shooter.
   * @return A {@link Command} to shoot the note.
   */
  public Command ShootCommand(double shooterSpeed, double intakeSpeed, double time)
  {
    return runEnd(() -> {
      shooterMotor.set(shooterSpeed); // run shooter at set speed.
      Timer.delay(time);  // ramp up shooter for time.
      intakeMotor.set(intakeSpeed); // index the note into the shooter at set speed.
    }, () -> {
      shooterMotor.stopMotor(); // stop motors when done.
      intakeMotor.stopMotor();
    }
   );
  }

  /**
   * Shoot the note at given speeds and times.
   * @param shooterSpeed to shoot the note.
   * @param intakeSpeed to index into the shooter.
   * @param rampUpTime to ramp up the shooter.
   * @param forTime to run command for.
   * @return a {@link Command} to shoot the note for set time.
   */
  public Command ShootCommandForTime(double shooterSpeed, double intakeSpeed, double rampUpTime, double forTime)
  {
    return runEnd(() -> {
      shooterMotor.set(shooterSpeed); // run shooter at set speed.
      Timer.delay(rampUpTime);  // ramp up shooter for time.
      intakeMotor.set(intakeSpeed); // index the note into shooter at set speed.
      Timer.delay(forTime); // keep running "forTime".
    }, () -> {
      shooterMotor.stopMotor(); // stop motors when done.
      intakeMotor.stopMotor();
    }
   );
  }

}

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

  public IntakeSubsystem() {
    intakeMotor.setInverted(false);
    shooterMotor.setInverted(true);
  }

  @Override
  public void periodic() {}

  public Command IntakeCommand(Double speed)
  {
    return runEnd(() -> { /* 
      if (noteSensor.get() == false) {
          intakeMotor.set(speed);  
        }
      else {
        intakeMotor.stopMotor();
      } */
      intakeMotor.set(speed);
    }, () -> {
      intakeMotor.stopMotor();
    }
   );
  }

  public Command ShootCommand(double shooterSpeed, double intakeSpeed, double time)
  {
    return runEnd(() -> {
      shooterMotor.set(shooterSpeed);
      Timer.delay(time);
      intakeMotor.set(intakeSpeed);
    }, () -> {
      shooterMotor.stopMotor();
      intakeMotor.stopMotor();
    }
   );
  }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ManipulatorConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  CANSparkMax intakeMotor = new CANSparkMax(ManipulatorConstants.intakeMotorPort, MotorType.kBrushless);
  CANSparkMax shooterMotor = new CANSparkMax(ManipulatorConstants.shooterMotorPort, MotorType.kBrushless);
  DigitalInput noteSensor = new DigitalInput(0);

  public IntakeSubsystem() {
    intakeMotor.setInverted(false);
    intakeMotor.setInverted(false);
  }

  @Override
  public void periodic() {}

  public Command IntakeCommand(Double speed)
  {
    return run(() -> {
      if (noteSensor.get() == false) {
          intakeMotor.set(speed);  
        }
      else {
        intakeMotor.stopMotor();
      }
    }
   );
  }

  public Command ShootCommand(double shooterSpeed, double intakeSpeed, double time)
  {
    return run(() -> {
      shooterMotor.set(shooterSpeed);
      new WaitCommand(time);
      intakeMotor.set(-intakeSpeed);
    }
   );
  }

}

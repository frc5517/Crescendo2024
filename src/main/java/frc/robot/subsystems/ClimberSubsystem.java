// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  CANSparkMax climberMoter = new CANSparkMax(ClimberConstants.climberMotorPort, MotorType.kBrushless);
  DigitalInput climbLimit = new DigitalInput(2);
  public ClimberSubsystem() {
    climberMoter.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Climber Limit", climbLimit.get());
  }

  public Command climberUp(double speed) {
    return runEnd(() -> {
      climberMoter.set(speed);
    }, () -> {
      climberMoter.stopMotor();
    });
  }

  public Command climberDown(double speed) {
    return runEnd(() -> {
      if (climbLimit.get()) { // If climber is going down and limit switch is hit stop climber.
        climberMoter.stopMotor();
      } else {  // Else go down as usual.
        climberMoter.set(-speed);
      }
    }, () -> {
      climberMoter.stopMotor();
    });
  }

}

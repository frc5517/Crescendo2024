// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  CANSparkMax climberMoter = new CANSparkMax(ClimberConstants.climberMotorPort, MotorType.kBrushless);
  RelativeEncoder encoder = climberMoter.getEncoder();
  public ClimberSubsystem() {
    climberMoter.setIdleMode(IdleMode.kBrake);
    
    climberMoter.setSoftLimit(SoftLimitDirection.kForward, 570);
    climberMoter.setSoftLimit(SoftLimitDirection.kReverse, -80);
    climberMoter.enableSoftLimit(SoftLimitDirection.kForward, false);
    climberMoter.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Encoder", encoder.getPosition());
  }

  public Command climberCommand(double speed) {
    return runEnd(() -> {
      climberMoter.set(speed);
    }, () -> {
      climberMoter.stopMotor();
    });
  }
}

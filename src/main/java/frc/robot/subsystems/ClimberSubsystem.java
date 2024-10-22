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
  CANSparkMax climbMoter = new CANSparkMax(ClimberConstants.climberMotorPort, MotorType.kBrushless);
  DigitalInput climbLimit = new DigitalInput(2);
  public ClimberSubsystem() {
    climbMoter.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Climber Limit", climbLimit.get());
  }

  public Command ClimbCommand(double speed) {
    return runEnd(() -> {
      if (speed < 0) {
        if (climbLimit.get()) { // We are going up and top limit is tripped so stop
            climbMoter.stopMotor();
        } else {  // We are going up but top limit is not tripped so go at commanded speed
            climbMoter.set(speed);
        } } else {
          climbMoter.set(speed);
        } 
    }, () -> {
      climbMoter.stopMotor();
    });
  }
}

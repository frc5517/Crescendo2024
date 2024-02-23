// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  CANSparkMax leftArmMotor = new CANSparkMax(ManipulatorConstants.leftArmMotorPort, MotorType.kBrushless);
  CANSparkMax rightArmMotor = new CANSparkMax(ManipulatorConstants.rightArmMotorPort, MotorType.kBrushless);

  public ArmSubsystem() {
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);

    rightArmMotor.follow(leftArmMotor, true);
  }
  
  @Override
  public void periodic() {}

  public Command ArmCommand(double speed) 
  {
    return runEnd(() -> {
      leftArmMotor.set(speed);
    }, () -> {
      rightArmMotor.stopMotor();
      leftArmMotor.stopMotor();
    }
   );
  }

  public Command ArmCommandForTime(double speed, double time) {
    return runEnd(() -> {
      leftArmMotor.set(speed);
      Timer.delay(time);
      leftArmMotor.stopMotor();
      rightArmMotor.stopMotor();
    }, () -> {
      leftArmMotor.stopMotor();
      rightArmMotor.stopMotor();
    });
  }
}

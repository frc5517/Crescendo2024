// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  CANSparkMax leftArmMotor = new CANSparkMax(ManipulatorConstants.leftArmMotorPort, MotorType.kBrushless);
  CANSparkMax rightArmMotor = new CANSparkMax(ManipulatorConstants.rightArmMotorPort, MotorType.kBrushless);

  public ArmSubsystem() {
    leftArmMotor.follow(rightArmMotor);

    rightArmMotor.setInverted(false);
    leftArmMotor.setInverted(false);
  }
  @Override
  public void periodic() {}

  public Command ArmCommand(double speed) 
  {
    return run(() -> {
      rightArmMotor.set(speed);
    }
   );
  }

}

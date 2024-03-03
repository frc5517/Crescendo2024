// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  // Initializes arm motors
  CANSparkMax leftArmMotor = new CANSparkMax(ManipulatorConstants.leftArmMotorPort, MotorType.kBrushless);
  CANSparkMax rightArmMotor = new CANSparkMax(ManipulatorConstants.rightArmMotorPort, MotorType.kBrushless);
  RelativeEncoder encoder = leftArmMotor.getEncoder();

  /**
   * Initialize {@link ArmSubsystem} with idle modes, inversions, and followers.
   */
  public ArmSubsystem() {
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);

    leftArmMotor.setSoftLimit(SoftLimitDirection.kForward, 93);
    leftArmMotor.setSoftLimit(SoftLimitDirection.kReverse, 1);
    leftArmMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    leftArmMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    rightArmMotor.follow(leftArmMotor, true);
  }

  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder Raw", encoder.getPosition());
  }

  public void resetEncoder() {
    encoder.setPosition(0);
  }

  public void disableLimit() {
    leftArmMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    leftArmMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }

  public Command moveToSetpoint(double speed, double setpoint) {
    return runEnd(() -> {
      double finalSpeed = speed; //* Math.abs(encoder.getPosition() - setpoint);
      while (encoder.getPosition() < setpoint) {
        leftArmMotor.set(finalSpeed);
      }
      while (encoder.getPosition() > setpoint) {
        leftArmMotor.set(-finalSpeed);
      }
    }, () -> {
      rightArmMotor.stopMotor();
      leftArmMotor.stopMotor();
    });
  }

  /**
   * Move the arm at set speed.
   * @param speed to move the arm.
   * @return A {@link Command} to move the arm.
   */
  public Command ArmCommand(double speed) 
  {
    return runEnd(() -> {
      leftArmMotor.set(speed); // move arm at "speed".
    }, () -> {
      rightArmMotor.stopMotor();  // stop both motors when done.
      leftArmMotor.stopMotor();
    }
   );
  }

  /**
   * Move the arm at set speed for set time.
   * @param speed to move the arm.
   * @param time to run command for.
   * @return A {@link Command} to move the arm for set time.
   */
  public Command ArmCommandForTime(double speed, double time) {
    return runEnd(() -> {
      leftArmMotor.set(speed); // move arm at "speed".
      Timer.delay(time);  // keep running for "time".
    }, () -> {
      leftArmMotor.stopMotor(); // stop both motors when done.
      rightArmMotor.stopMotor();
    });
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
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
  DigitalInput topLimit = new DigitalInput(0);
  DigitalInput bottomLimit = new DigitalInput(1);
  RelativeEncoder encoder = leftArmMotor.getEncoder();

  /**
   * Initialize {@link ArmSubsystem} with idle modes, inversions, and followers.
   */
  public ArmSubsystem() {
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);

    rightArmMotor.follow(leftArmMotor, true);
  }

  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder Raw", encoder.getPosition());
  }

  public Command moveToSetpoint(double speed, double setpoint) {
    return runEnd(() -> {

      if (speed > 0) {
        if (topLimit.get()) { // We are going up and top limit is tripped so stop
            leftArmMotor.set(0);
            encoder.setPosition(95);
        } else {  // We are going up but top limit is not tripped so go at commanded speed
            leftArmMotor.set(speed);
        }
    } else {
        if (bottomLimit.get()) {  // We are going down and bottom limit is tripped so stop
            leftArmMotor.set(0);
            encoder.setPosition(0);
        } else {  // We are going down but bottom limit is not tripped so go at commanded speed
            leftArmMotor.set(speed);
        } 
      }

      while (encoder.getPosition() < setpoint) {
        leftArmMotor.set(speed);
      }
      while (encoder.getPosition() > setpoint) {
        leftArmMotor.set(-speed);
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
      if (speed > 0) {
        if (topLimit.get()) { // We are going up and top limit is tripped so stop
            leftArmMotor.set(0);
            encoder.setPosition(100);
        } else {  // We are going up but top limit is not tripped so go at commanded speed
            leftArmMotor.set(speed);
        }
    } else {
        if (bottomLimit.get()) {  // We are going down and bottom limit is tripped so stop
            leftArmMotor.set(0);
            encoder.setPosition(0);
        } else {  // We are going down but bottom limit is not tripped so go at commanded speed
            leftArmMotor.set(speed);
        } 
      }
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
      if (speed > 0) {
        if (topLimit.get()) { // We are going up and top limit is tripped so stop
            leftArmMotor.set(0);
            encoder.setPosition(100);
        } else {  // We are going up but top limit is not tripped so go at commanded speed
            leftArmMotor.set(speed);
        }
    } else {
        if (bottomLimit.get()) {  // We are going down and bottom limit is tripped so stop
            leftArmMotor.set(0);
            encoder.setPosition(0);
        } else {  // We are going down but bottom limit is not tripped so go at commanded speed
            leftArmMotor.set(speed);
        }
      }
      Timer.delay(time);  // keep running for "time".
    }, () -> {
      leftArmMotor.stopMotor(); // stop both motors when done.
      rightArmMotor.stopMotor();
    });
  }
}

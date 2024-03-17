// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class ArmSubsytem extends SubsystemBase {
  /** Creates a new PIDArmSubsystem. */
  CANSparkMax leftMotor = new CANSparkMax(ManipulatorConstants.leftArmMotorPort, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(ManipulatorConstants.rightArmMotorPort, MotorType.kBrushless);
  SparkPIDController armController;
  RelativeEncoder armEncoder;

  DigitalInput topLimit = new DigitalInput(0);
  DigitalInput bottomLimit = new DigitalInput(1);

  public ArmSubsytem() {
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

    armEncoder = leftMotor.getEncoder();
    armController = leftMotor.getPIDController();
    armController.setPositionPIDWrappingEnabled(true);

    rightMotor.follow(leftMotor, true);

    armController.setP(ManipulatorConstants.armP);
    armController.setI(ManipulatorConstants.armI);
    armController.setD(ManipulatorConstants.armD);
    armController.setOutputRange(ManipulatorConstants.armMinOutput, ManipulatorConstants.armMaxOutput);
  }

  public Command ArmCommand(double speed) {
    return runEnd(() -> {
      if (speed > 0) {
        if (topLimit.get()) { // We are going up and top limit is tripped so stop
            leftMotor.stopMotor();
        } else {  // We are going up but top limit is not tripped so go at commanded speed
            armController.setReference(speed, ControlType.kDutyCycle);
        } } else {
        if (bottomLimit.get()) {  // We are going down and bottom limit is tripped so stop
            leftMotor.stopMotor();
        } else {  // We are going down but bottom limit is not tripped so go at commanded speed
            armController.setReference(speed, ControlType.kDutyCycle);
        } 
      }
    }, () -> {
      leftMotor.stopMotor();
      rightMotor.stopMotor();
    });
  }

  public Command MoveToSetpoint(double setpoint) {
    return runEnd(() -> {
      if (leftMotor.getAppliedOutput() > 0) {
        if (topLimit.get()) { // We are going up and top limit is tripped so stop
            leftMotor.stopMotor();
        } else {  // We are going up but top limit is not tripped so go at commanded speed
            armController.setReference(setpoint, ControlType.kPosition);
        } } else {
        if (bottomLimit.get()) {  // We are going down and bottom limit is tripped so stop
            leftMotor.stopMotor();
        } else {  // We are going down but bottom limit is not tripped so go at commanded speed
            armController.setReference(setpoint, ControlType.kPosition);
        } 
      }
    }, () -> {
      leftMotor.stopMotor();
      rightMotor.stopMotor();
    });
  }

  public void ArmEncoderUp() {
    armEncoder.setPosition(90);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      if (topLimit.get()) { // We are going up and top limit is tripped so set position
          armEncoder.setPosition(90);
      } else if (bottomLimit.get()) {  // We are going down and bottom limit is tripped so set position
          armEncoder.setPosition(0);
      } else {} // If neither do nothing
      SmartDashboard.putNumber("PID Arm Encoder", armEncoder.getPosition());
  }
}

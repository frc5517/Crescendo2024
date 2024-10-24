// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;
public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  CANSparkMax intakeMotor = new CANSparkMax(ManipulatorConstants.intakeMotorPort, MotorType.kBrushed);
  CANSparkMax shooterMotor = new CANSparkMax(ManipulatorConstants.shooterMotorPort, MotorType.kBrushed);

  ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  ColorMatch colorMatch = new ColorMatch();
  Color noteColor = new Color(.550, .366, .083);
  Boolean detected = false;
  String matchedString = "No Note";

  /**
   * Initialize {@link IntakeSubsystem} with idle modes.
   */
  public IntakeSubsystem() {
    intakeMotor.setInverted(false);
    shooterMotor.setInverted(true);

    colorMatch.addColorMatch(noteColor);  // Adds noteColor to match color stored colors. 
  }

  @Override
  public void periodic() {
    /* 
    SmartDashboard.putString("Note Sensor", matchedString); // Put data to dashboard. 
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);  */
  }

  /**
   * Run the intake at set speed.
   * @param speed to intake note.
   * @return A {@link Command} to run the intake for set time.
   */
  public Command IntakeCommand(Double speed)
  {
    return runEnd(() -> {
      intakeMotor.set(speed); // outtake note without sensor
    }, () -> {
      intakeMotor.stopMotor();  // stop intake when done.
    }
   );
  }

  /**
   * Run the intake at set speed if no note is detected.
   * @param speed to intake note.
   * @return A {@link Command} to run the intake if theres no note.
   */
  public Command IntakeWithSensor(Double speed)
  {
    return runEnd(() -> { 
    Color detectedColor = colorSensor.getColor(); // Get color sensor data. 
    ColorMatchResult matchedColor = colorMatch.matchColor(detectedColor); // Checks if note color is detected. 

      if (colorMatch.matchColor(detectedColor) != null) { // If no matched color don't run. 
      if (matchedColor.color == noteColor) {  // If matched color equals the note color detected, change values to note detected. 
        intakeMotor.stopMotor();
      } 
    } else {  // else set to no note values. 
        intakeMotor.set(speed);
    }
    }, () -> {
      intakeMotor.stopMotor(); // stop motor when done.
    }
   );
  }

  public Command IntakeBackOut(Double speed)
  {
    return runEnd(() -> { 
    Color detectedColor = colorSensor.getColor(); // Get color sensor data. 
    ColorMatchResult matchedColor = colorMatch.matchColor(detectedColor); // Checks if note color is detected. 

      if (colorMatch.matchColor(detectedColor) != null) { // If no matched color don't run. 
      if (matchedColor.color == noteColor) {  // If matched color equals the note color detected, change values to note detected. 
        intakeMotor.stopMotor();
        Timer.delay(.2);
        intakeMotor.set(speed/2);
        Timer.delay(.2);
        intakeMotor.stopMotor();
      } 
    } else {  // else set to no note values. 
        intakeMotor.set(speed);
    }
    }, () -> {
      intakeMotor.stopMotor();
    }
   );
  }

  /**
   * Shoot the note at given speeds and ramp up time.
   * @param shooterSpeed to shoot the note.
   * @param intakeSpeed to index into the shooter.
   * @param time to ramp up shooter.
   * @return A {@link Command} to shoot the note.
   */
  public Command ShootCommand(double shooterSpeed, double intakeSpeed, double time)
  {
    return runEnd(() -> {
      shooterMotor.set(shooterSpeed); // run shooter at set speed.
      Timer.delay(time);  // ramp up shooter for time.
      intakeMotor.set(intakeSpeed); // index the note into the shooter at set speed.
    }, () -> {
      shooterMotor.stopMotor(); // stop motors when done.
      intakeMotor.stopMotor();
    }
   );
  }
}

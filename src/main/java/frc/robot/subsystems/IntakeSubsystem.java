// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  CANSparkMax intakeMotor = new CANSparkMax(ManipulatorConstants.intakeMotorPort, MotorType.kBrushed);
  CANSparkMax shooterMotor = new CANSparkMax(ManipulatorConstants.shooterMotorPort, MotorType.kBrushed);
  DigitalInput noteSensor = new DigitalInput(0);

  ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  ColorMatch colorMatch = new ColorMatch();
  Color noteColor = new Color(.629, .318, .060);

  /**
   * Initialize {@link IntakeSubsystem} with idle modes.
   */
  public IntakeSubsystem() {
    intakeMotor.setInverted(false);
    shooterMotor.setInverted(true);

    colorMatch.setConfidenceThreshold(.95);

    colorMatch.addColorMatch(noteColor); 
    colorMatch.addColorMatch(Color.kOrange);
    colorMatch.addColorMatch(Color.kRed);
    colorMatch.addColorMatch(Color.kGreen);
    colorMatch.addColorMatch(Color.kBlue);
    colorMatch.addColorMatch(Color.kYellow);
    colorMatch.addColorMatch(Color.kBlack);
    
    SmartDashboard.putData("Note Sensor", noteSensor);
  }

  @Override
  public void periodic() {
    Color detectedColor = colorSensor.getColor();

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", colorSensor.getIR());
    SmartDashboard.putNumber("Prox", colorSensor.getProximity()); // <200

    ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);

    ColorMatchResult test = colorMatch.matchColor(detectedColor);
    
    String detected;
    
    if (test.color.equals(noteColor)) {
      detected = "Test Match Note";
    }
    else if (match.color == noteColor && colorSensor.getProximity() > 200) {
      detected = "Note Detected";
    }
    else if (match.color == noteColor) {
      detected = "Note Color";
    }
    else if (match.color == Color.kOrange) {
      detected = "kOrange";
    }
    else if (match.color == Color.kRed) {
      detected = "Red";
    }
    else if (match.color == Color.kGreen) {
      detected = "Green";
    }
    else if (match.color == Color.kBlue) {
      detected = "Blue";
    }
    else if (match.color == Color.kYellow) {
      detected = "Yellow";
    }
    else {
      detected = "nothing";
    }

    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Color Sensor", detected);
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
      if (noteSensor.get() == true) {  // if theres no note allow intake to run otherwise stop intake
        intakeMotor.set(speed);
      }
      else {
        intakeMotor.stopMotor();
      }
    }, () -> {
      intakeMotor.stopMotor(); // stop motor when done.
    }
   );
  }

  /**
   * Run the intake at set speed until note is detected, stop intake for time when note detected. 
   * @param speed to intake note.
   * @param time to stop intake for.
   * @return A {@link Command} to run the intake until a note is detected. 
   */
  public Command IntakeStop(Double speed, Double time) {
    return runEnd(() -> {
      if (noteSensor.get() == false) {  // If note detected stop motor for set time. 
        intakeMotor.stopMotor();
        Timer.delay(time);
      }
      else {
        intakeMotor.set(speed);
      }
    }, () -> {
      intakeMotor.stopMotor();  // Stop motor if ended. 
    });
  }

  /**
   * Run the intake at set speed for time. 
   * @param speed to run intake at. 
   * @param time to run intake for.
   * @return A {@link Command} to run intake for set time. 
   */
  public Command IntakeForTime(Double speed, Double time) {
    return runEnd(() -> {
      intakeMotor.set(speed); // Intake at speed for time then stop motor. 
      Timer.delay(time);
      intakeMotor.stopMotor();
    }, () -> {
      intakeMotor.stopMotor();  // Stop motor if ended. 
    });
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

  /**
   * Shoot the note at given speeds and times.
   * @param shooterSpeed to shoot the note.
   * @param intakeSpeed to index into the shooter.
   * @param rampUpTime to ramp up the shooter.
   * @param forTime to run command for.
   * @return a {@link Command} to shoot the note for set time.
   */
  public Command ShootCommandForTime(double shooterSpeed, double intakeSpeed, double rampUpTime, double forTime)
  {
    return runEnd(() -> {
      shooterMotor.set(shooterSpeed); // run shooter at set speed.
      Timer.delay(rampUpTime);  // ramp up shooter for time.
      intakeMotor.set(intakeSpeed); // index the note into shooter at set speed.
      Timer.delay(forTime); // keep running "forTime".
    }, () -> {
      shooterMotor.stopMotor(); // stop motors when done.
      intakeMotor.stopMotor();
    }
   );
  }

}

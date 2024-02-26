package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class VisionTrack extends Command {
  
  private final SwerveSubsystem drivebase;
  private final PhotonCamera photoncamera;

  private final PIDController pidControllerX = new PIDController(1, 0, 0);
  private final PIDController pidControllerY = new PIDController(1.5, 0, 0);
  private final PIDController pidControllerOmega = new PIDController(.5, 0, 0);

  public VisionTrack(SwerveSubsystem drivebase, PhotonCamera photoncamera) {
    this.drivebase = drivebase;
    this.photoncamera = photoncamera;

    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    super.initialize();
    pidControllerX.reset();
    pidControllerY.reset();
    pidControllerOmega.reset();

    pidControllerX.setSetpoint(Units.inchesToMeters(36)); // Move forward/backwork to keep 36 inches from the target
    pidControllerX.setTolerance(Units.inchesToMeters(2.5));

    pidControllerY.setSetpoint(0); // Move side to side to keep target centered
    pidControllerX.setTolerance(Units.inchesToMeters(2.5));

    pidControllerOmega.setSetpoint(Units.degreesToRadians(-90)); // Rotate the keep perpendicular with the target
    pidControllerOmega.setTolerance(Units.degreesToRadians(1));

  }

  @Override
  public void execute() {
    var result = photoncamera.getLatestResult();
    if (result.hasTargets()) {
      var cameraToTarget = result.getBestTarget().getBestCameraToTarget();

      // X - distance from camera in meters
      // Y - right and left of camera center (in meters?)
      // Z - above and below camera center (in meters?)
      // rotation X - pitch - 0-degrees is flat on floor - rotation is positive as tilted toward camera
      //                    - visible targets in range [0, 180]
      // rotation Y - roll - 0-degrees is straight upward (or straight down) - clockwise rotation is positive
      //                  - seems to give same results if the target is upside down (maybe need to research this one)
      //                  - visible targets are in range [-90, 90]
      // rotation Z - yaw - 0-degrees is perpendicular to the camera, rotated with the right side away from camera (not visible)
      //                  - -90-degrees is straight on with the camera
      //                  - from the camera's perspective, rotation the left side of the target closer is positive
      //                  - visible targets are in range [-180, 0]

      cameraToTarget.getRotation().getAngle();
      SmartDashboard.putNumber("Target X", cameraToTarget.getX());
      SmartDashboard.putNumber("Target Y", Units.metersToInches(cameraToTarget.getY()));
      SmartDashboard.putNumber("Target Z", Units.metersToInches(cameraToTarget.getZ()));
      SmartDashboard.putNumber("Target Rotation X", Units.radiansToDegrees(cameraToTarget.getRotation().getX()));
      SmartDashboard.putNumber("Target Rotation Y", Units.radiansToDegrees(cameraToTarget.getRotation().getY()));
      SmartDashboard.putNumber("Target Rotation Z", Units.radiansToDegrees(cameraToTarget.getRotation().getZ()));

      
      // Handle distance to target
      var distanceFromTarget = cameraToTarget.getX();
      var xSpeed = pidControllerX.calculate(distanceFromTarget);
      if (pidControllerX.atSetpoint()) {
        xSpeed = 0;
      }

      // Handle alignment side-to-side
      var targetY = cameraToTarget.getY();
      var ySpeed = pidControllerY.calculate(targetY);
      if (pidControllerY.atSetpoint()) {
        ySpeed = 0;
      }

      // Handle rotation using target Yaw/Z rotation
      var targetYaw = cameraToTarget.getRotation().getZ();
      var omegaSpeed = pidControllerOmega.calculate(targetYaw);
      if (pidControllerOmega.atSetpoint()) {
        omegaSpeed = 0;
      }
      
      drivebase.drive(new ChassisSpeeds(-xSpeed, -ySpeed, -omegaSpeed));
    } else {
      drivebase.lock();
    }
  }

  @Override
  public boolean isFinished() {
    return super.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.lock();
  }

}

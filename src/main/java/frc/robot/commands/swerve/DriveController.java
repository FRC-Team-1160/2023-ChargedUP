package frc.robot.commands.swerve;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

/**
 * Custom version of a @HolonomicDriveController specifically for following PathPlanner paths
 *
 * <p>This controller adds the following functionality over the WPILib version: - calculate() method
 * takes in a PathPlannerState directly - Continuous input is automatically enabled for the rotation
 * controller - Holonomic angular velocity is used as a feedforward for the rotation controller,
 * which no longer needs to be a @ProfiledPIDController
 */
public class DriveController {
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;

  private double translationXError = 0;
  private double translationYError = 0;
  private double rotationError = 0;
  private double toleranceX = 0;
  private double toleranceY = 0;
  private double toleranceRotation = 0;
  private boolean isEnabled = true;

  /**
   * Constructs a PPHolonomicDriveController
   *
   * @param xController A PID controller to respond to error in the field-relative X direction
   * @param yController A PID controller to respond to error in the field-relative Y direction
   * @param rotationController A PID controller to respond to error in rotation
   */
  public DriveController(
      PIDController xController, PIDController yController, PIDController rotationController) {
    this.xController = xController;
    this.yController = yController;
    this.rotationController = rotationController;

    // Auto-configure continuous input for rotation controller
    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns true if the pose error is within tolerance of the reference.
   *
   * @return True if the pose error is within tolerance of the reference.
   */
  public boolean atReference() {
    double translationToleranceX = this.toleranceX;
    double translationToleranceY = this.toleranceY;
    double rotationTolerance = this.toleranceRotation;

    return Math.abs(this.translationXError) < translationToleranceX
        && Math.abs(this.translationYError) < translationToleranceY
        && Math.abs(this.rotationError) < Math.toRadians(rotationTolerance);
  }

  /**
   * Sets the pose error whic is considered tolerance for use with atReference()
   *
   * @param tolerance The pose error which is tolerable
   */
  public void setTolerance(double x, double y, double r) {
    this.toleranceX = x;
    this.toleranceY = y;
    this.toleranceRotation = r;
  }

  /**
   * Enables and disables the controller for troubleshooting. When calculate() is called on a
   * disabled controller, only feedforward values are returned.
   *
   * @param enabled If the controller is enabled or not
   */
  public void setEnabled(boolean enabled) {
    this.isEnabled = enabled;
  }

  /**
   * Calculates the next output of the holonomic drive controller
   *
   * @param currentPose The current pose
   * @param referenceState The desired trajectory state
   * @return The next output of the holonomic drive controller
   */
  public double[] calculate(double currentPoseX, double currentPoseY, double gyroAngle, PathPlannerState referenceState) {
    double[] speeds = new double[3];
    double xFF =
        referenceState.velocityMetersPerSecond * referenceState.poseMeters.getRotation().getCos();
    double yFF =
        referenceState.velocityMetersPerSecond * referenceState.poseMeters.getRotation().getSin();
    double rotationFF = referenceState.holonomicAngularVelocityRadPerSec;

    this.translationXError = referenceState.poseMeters.getX()-currentPoseX;
    this.translationYError = referenceState.poseMeters.getY()-currentPoseY;
    if (referenceState.angularVelocityRadPerSec > 0) {
      this.rotationError = angleToLoc(gyroAngle) - angleToLoc(referenceState.holonomicRotation.getDegrees());
      if (this.rotationError > 180) {
        this.rotationError = Math.abs(this.rotationError-360);
      }
      this.rotationError = Math.toRadians(this.rotationError);
    } else {
      this.rotationError =  angleToLoc(referenceState.holonomicRotation.getDegrees()) - angleToLoc(gyroAngle);
      if (this.rotationError > 180) {
        this.rotationError = Math.abs(this.rotationError-360);
      }
      this.rotationError = Math.toRadians(this.rotationError);
    }
    SmartDashboard.putNumber("reference degrees", referenceState.holonomicRotation.getDegrees());
    SmartDashboard.putNumber("rotationError", Math.toDegrees(rotationError));
    if (!this.isEnabled) {
      speeds[0] = xFF;
      speeds[1] = yFF;
      speeds[2] = rotationFF;
      return speeds;
    }

    double xFeedback =
        this.xController.calculate(currentPoseX, referenceState.poseMeters.getX());
    double yFeedback =
        this.yController.calculate(currentPoseY, referenceState.poseMeters.getY());
    double rotationFeedback =
        this.rotationController.calculate(
             Math.toRadians(gyroAngle), referenceState.holonomicRotation.getRadians());
    SmartDashboard.putNumber("rotationFF", rotationFF);
    SmartDashboard.putNumber("rotationFeedback", rotationFeedback);

    speeds[0] = xFF*0.9 + xFeedback;
    speeds[1] = yFF*0.9 + yFeedback;
    speeds[2] = rotationFF/SwerveConstants.AUTO_ROTATION + rotationFeedback;
    return speeds;
  }

  public double angleToLoc(double ogRot)
    {
        if(ogRot < 0)
        {
            return ogRot + 360;
        }
        return ogRot;
    }
  
}
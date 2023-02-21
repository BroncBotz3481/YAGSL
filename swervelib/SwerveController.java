package swervelib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import swervelib.parser.SwerveControllerConfiguration;

/**
 * Controller class used to convert raw inputs into robot speeds.
 */
public class SwerveController
{

  /**
   * {@link SwerveControllerConfiguration} object storing data to generate the {@link PIDController} for controlling the
   * robot heading, and deadband for heading joystick.
   */
  public final  SwerveControllerConfiguration config;
  /**
   * PID Controller for the robot heading.
   */
  private final PIDController                 thetaController; // TODO: Switch to ProfilePIDController
  /**
   * Last angle to robot was set to.
   */
  public        double                        lastAngle;

  /**
   * Construct the SwerveController object which is used for determining the speeds of the robot based on controller
   * input.
   *
   * @param cfg {@link SwerveControllerConfiguration} containing the PIDF variables for the heading PIDF.
   */
  public SwerveController(SwerveControllerConfiguration cfg)
  {
    config = cfg;
    thetaController = config.headingPIDF.createPIDController();
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    lastAngle = 0;
  }

  /**
   * Helper function to get the {@link Translation2d} of the chassis speeds given the {@link ChassisSpeeds}.
   *
   * @param speeds Chassis speeds.
   * @return {@link Translation2d} of the speed the robot is going in.
   */
  public static Translation2d getTranslation2d(ChassisSpeeds speeds)
  {
    return new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  /**
   * Calculate the hypot deadband and check if the joystick is within it.
   *
   * @param x The x value for the joystick in which the deadband should be applied.
   * @param y The y value for the joystick in which the deadband should be applied.
   * @return Whether the values are within the deadband from
   * {@link SwerveControllerConfiguration#angleJoyStickRadiusDeadband}.
   */
  public boolean withinHypotDeadband(double x, double y)
  {
    return Math.hypot(x, y) < config.angleJoyStickRadiusDeadband;
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and a angle.
   *
   * @param xInput                     X joystick input for the robot to move in the X direction.
   * @param yInput                     Y joystick input for the robot to move in the Y direction.
   * @param angle                      The desired angle of the robot in radians.
   * @param currentHeadingAngleRadians The current robot heading in radians.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(
      double xInput, double yInput, double angle, double currentHeadingAngleRadians)
  {
    // Calculates an angular rate using a PIDController and the commanded angle.  This is then
    // scaled by
    // the drivebase's maximum angular velocity.
    double omega =
        thetaController.calculate(currentHeadingAngleRadians, angle) * config.maxAngularVelocity;
    // Convert joystick inputs to m/s by scaling by max linear speed.  Also uses a cubic function
    // to allow for precise control and fast movement.
    double x = Math.pow(xInput, 3) * config.maxSpeed;
    double y = Math.pow(yInput, 3) * config.maxSpeed;

    return new ChassisSpeeds(x, y, omega);
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput                     X joystick input for the robot to move in the X direction.
   * @param yInput                     Y joystick input for the robot to move in the Y direction.
   * @param headingX                   X joystick which controls the angle of the robot.
   * @param headingY                   Y joystick which controls the angle of the robot.
   * @param currentHeadingAngleRadians The current robot heading in radians.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(
      double xInput,
      double yInput,
      double headingX,
      double headingY,
      double currentHeadingAngleRadians)
  {
    // Converts the horizontal and vertical components to the commanded angle, in radians, unless
    // the joystick is near
    // the center (i. e. has been released), in which case the angle is held at the last valid
    // joystick input (hold
    // position when stick released).
    double angle =
        withinHypotDeadband(headingX, headingY) ? lastAngle : Math.atan2(headingX, headingY);
    ChassisSpeeds speeds = getTargetSpeeds(xInput, yInput, angle, currentHeadingAngleRadians);

    // Used for the position hold feature
    lastAngle = angle;

    return speeds;
  }
}

package swervelib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
  public final SwerveControllerConfiguration config;
  /**
   * PID Controller for the robot heading.
   */
  public final PIDController                 thetaController; // TODO: Switch to ProfilePIDController
  /**
   * Last angle as a scalar [-1,1] the robot was set to.
   */
  public       double                        lastAngleScalar;
  /**
   * {@link SlewRateLimiter} for movement in the X direction in meters/second.
   */
  public       SlewRateLimiter               xLimiter     = null;
  /**
   * {@link SlewRateLimiter} for movement in the Y direction in meters/second.
   */
  public       SlewRateLimiter               yLimiter     = null;
  /**
   * {@link SlewRateLimiter} for angular movement in radians/second.
   */
  public       SlewRateLimiter               angleLimiter = null;

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
    lastAngleScalar = 0;
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
   * Add slew rate limiters to all controls. This prevents the robot from ramping up too much. To disable a
   * {@link SlewRateLimiter} set the desired one to null.
   *
   * @param x     The {@link SlewRateLimiter} for the X velocity in meters/second.
   * @param y     The {@link SlewRateLimiter} for the Y velocity in meters/second.
   * @param angle The {@link SlewRateLimiter} for the angular velocity in radians/second.
   */
  public void addSlewRateLimiters(SlewRateLimiter x, SlewRateLimiter y, SlewRateLimiter angle)
  {
    xLimiter = x;
    yLimiter = y;
    angleLimiter = angle;
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
   * Get the chassis speeds based on controller input of 1 joystick [-1,1] and an angle.
   *
   * @param xInput                     X joystick input for the robot to move in the X direction. X = xInput * maxSpeed
   * @param yInput                     Y joystick input for the robot to move in the Y direction. Y = yInput *
   *                                   maxSpeed;
   * @param angle                      The desired angle of the robot in radians.
   * @param currentHeadingAngleRadians The current robot heading in radians.
   * @param maxSpeed                   Maximum speed in meters per second.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(
      double xInput, double yInput, double angle, double currentHeadingAngleRadians, double maxSpeed)
  {
    // Convert joystick inputs to m/s by scaling by max linear speed.  Also uses a cubic function
    // to allow for precise control and fast movement.
    double x = xInput * maxSpeed;
    double y = yInput * maxSpeed;

    return getRawTargetSpeeds(x, y, angle, currentHeadingAngleRadians);
  }

  /**
   * Get the angle in radians based off of the heading joysticks.
   *
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return angle in radians from the joystick.
   */
  public double getJoystickAngle(double headingX, double headingY)
  {
    lastAngleScalar =
        withinHypotDeadband(headingX, headingY) ? lastAngleScalar : Math.atan2(headingX, headingY);
    return lastAngleScalar;
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
   * @param maxSpeed                   Maximum speed of the drive motors in meters per second, multiplier of the xInput
   *                                   and yInput.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(
      double xInput,
      double yInput,
      double headingX,
      double headingY,
      double currentHeadingAngleRadians,
      double maxSpeed)
  {
    // Converts the horizontal and vertical components to the commanded angle, in radians, unless
    // the joystick is near
    // the center (i. e. has been released), in which case the angle is held at the last valid
    // joystick input (hold
    // position when stick released).
    double angle =
        withinHypotDeadband(headingX, headingY) ? lastAngleScalar : Math.atan2(headingX, headingY);
    ChassisSpeeds speeds = getTargetSpeeds(xInput, yInput, angle, currentHeadingAngleRadians, maxSpeed);

    // Used for the position hold feature
    lastAngleScalar = angle;

    return speeds;
  }

  /**
   * Get the {@link ChassisSpeeds} based of raw speeds desired in meters/second and heading in radians.
   *
   * @param xSpeed X speed in meters per second.
   * @param ySpeed Y speed in meters per second.
   * @param omega  Angular velocity in radians/second.
   * @return {@link ChassisSpeeds} the robot should move to.
   */
  public ChassisSpeeds getRawTargetSpeeds(double xSpeed, double ySpeed, double omega)
  {
    if (xLimiter != null)
    {
      xSpeed = xLimiter.calculate(xSpeed);
    }
    if (yLimiter != null)
    {
      ySpeed = yLimiter.calculate(ySpeed);
    }
    if (angleLimiter != null)
    {
      omega = angleLimiter.calculate(omega);
    }

    return new ChassisSpeeds(xSpeed, ySpeed, omega);
  }

  /**
   * Get the {@link ChassisSpeeds} based of raw speeds desired in meters/second and heading in radians.
   *
   * @param xSpeed                     X speed in meters per second.
   * @param ySpeed                     Y speed in meters per second.
   * @param targetHeadingAngleRadians  Target heading in radians.
   * @param currentHeadingAngleRadians Current heading in radians.
   * @return {@link ChassisSpeeds} the robot should move to.
   */
  public ChassisSpeeds getRawTargetSpeeds(double xSpeed, double ySpeed, double targetHeadingAngleRadians,
                                          double currentHeadingAngleRadians)
  {
    // Calculates an angular rate using a PIDController and the commanded angle. Returns a value between -1 and 1
    // which is then scaled to be between -maxAngularVelocity and +maxAngularVelocity.
    return getRawTargetSpeeds(xSpeed, ySpeed,
                              thetaController.calculate(currentHeadingAngleRadians, targetHeadingAngleRadians) *
                              config.maxAngularVelocity);
  }

  /**
   * Calculate the angular velocity given the current and target heading angle in radians.
   *
   * @param currentHeadingAngleRadians The current heading of the robot in radians.
   * @param targetHeadingAngleRadians  The target heading of the robot in radians.
   * @return Angular velocity in radians per second.
   */
  public double headingCalculate(double currentHeadingAngleRadians, double targetHeadingAngleRadians)
  {
    return thetaController.calculate(currentHeadingAngleRadians, targetHeadingAngleRadians) * config.maxAngularVelocity;
  }

  /**
   * Set a new maximum angular velocity that is different from the auto-generated one. Modified the
   * {@link SwerveControllerConfiguration#maxAngularVelocity} field which is used in the {@link SwerveController} class
   * for {@link ChassisSpeeds} generation.
   *
   * @param angularVelocity Angular velocity in radians per second.
   */
  public void setMaximumAngularVelocity(double angularVelocity)
  {
    config.maxAngularVelocity = angularVelocity;
  }
}

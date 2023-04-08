package swervelib.parser;

import static swervelib.math.SwerveMath.calculateMaxAngularVelocity;

/**
 * Swerve Controller configuration class which is used to configure {@link swervelib.SwerveController}.
 */
public class SwerveControllerConfiguration
{

  /**
   * PIDF for the heading of the robot.
   */
  public final PIDFConfig headingPIDF;
  /**
   * hypotenuse deadband for the robot angle control joystick.
   */
  public final double
                          angleJoyStickRadiusDeadband; // Deadband for the minimum hypot for the heading joystick.
  /**
   * Maximum robot speed in meters per second.
   */
  public       double     maxSpeed;
  /**
   * Maximum angular velocity in rad/s
   */
  public       double     maxAngularVelocity;

  /**
   * Construct the swerve controller configuration.
   *
   * @param driveCfg                    Drive configuration.
   * @param headingPIDF                 Heading PIDF configuration.
   * @param angleJoyStickRadiusDeadband Deadband on radius of angle joystick.
   */
  public SwerveControllerConfiguration(
      SwerveDriveConfiguration driveCfg,
      PIDFConfig headingPIDF,
      double angleJoyStickRadiusDeadband)
  {
    this.maxSpeed = driveCfg.maxSpeed;
    this.maxAngularVelocity =
        calculateMaxAngularVelocity(
            driveCfg.maxSpeed,
            Math.abs(driveCfg.moduleLocationsMeters[0].getX()),
            Math.abs(driveCfg.moduleLocationsMeters[0].getY()));
    this.headingPIDF = headingPIDF;
    this.angleJoyStickRadiusDeadband = angleJoyStickRadiusDeadband;
  }

  /**
   * Construct the swerve controller configuration. Assumes hypotenuse deadband of 0.5 (minimum radius for angle to be
   * set on angle joystick is .5 of the controller).
   *
   * @param driveCfg    Drive configuration.
   * @param headingPIDF Heading PIDF configuration.
   */
  public SwerveControllerConfiguration(SwerveDriveConfiguration driveCfg, PIDFConfig headingPIDF)
  {
    this(driveCfg, headingPIDF, 0.5);
  }
}

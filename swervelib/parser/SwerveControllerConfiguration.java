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
   * Maximum angular velocity in rad/s
   */
  public       double     maxAngularVelocity;

  /**
   * Construct the swerve controller configuration. Assumes robot is square to fetch maximum angular velocity.
   *
   * @param driveCfg                    {@link SwerveDriveConfiguration} to fetch the first module X and Y used to
   *                                    calculate the maximum angular velocity.
   * @param headingPIDF                 Heading PIDF configuration.
   * @param angleJoyStickRadiusDeadband Deadband on radius of angle joystick.
   * @param maxSpeedMPS                 Maximum speed in meters per second for angular velocity, remember if you have
   *                                    feet per second use {@link edu.wpi.first.math.util.Units#feetToMeters(double)}.
   */
  public SwerveControllerConfiguration(
      SwerveDriveConfiguration driveCfg,
      PIDFConfig headingPIDF,
      double angleJoyStickRadiusDeadband,
      double maxSpeedMPS)
  {
    this.maxAngularVelocity =
        calculateMaxAngularVelocity(
            maxSpeedMPS,
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
   * @param maxSpeedMPS Maximum speed in meters per second for angular velocity, remember if you have feet per second
   *                    use {@link edu.wpi.first.math.util.Units#feetToMeters(double)}.
   */
  public SwerveControllerConfiguration(SwerveDriveConfiguration driveCfg, PIDFConfig headingPIDF, double maxSpeedMPS)
  {
    this(driveCfg, headingPIDF, 0.5, maxSpeedMPS);
  }
}

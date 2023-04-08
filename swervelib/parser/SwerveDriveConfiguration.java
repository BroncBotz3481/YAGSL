package swervelib.parser;

import edu.wpi.first.math.geometry.Translation2d;
import swervelib.SwerveModule;
import swervelib.imu.SwerveIMU;

/**
 * Swerve drive configurations used during SwerveDrive construction.
 */
public class SwerveDriveConfiguration
{

  /**
   * Swerve Module locations.
   */
  public Translation2d[] moduleLocationsMeters;
  /**
   * Swerve IMU
   */
  public SwerveIMU       imu;
  /**
   * Invert the imu measurements.
   */
  public boolean         invertedIMU = false;
  /**
   * Max module speed in meters per second.
   */
  public double          maxSpeed, attainableMaxTranslationalSpeedMetersPerSecond,
      attainableMaxRotationalVelocityRadiansPerSecond;
  /**
   * Number of modules on the robot.
   */
  public int            moduleCount;
  /**
   * Swerve Modules.
   */
  public SwerveModule[] modules;

  /**
   * Create swerve drive configuration.
   *
   * @param moduleConfigs Module configuration.
   * @param swerveIMU     Swerve IMU.
   * @param maxSpeed      Max speed of the robot in meters per second.
   * @param invertedIMU   Invert the IMU.
   */
  public SwerveDriveConfiguration(
      SwerveModuleConfiguration[] moduleConfigs,
      SwerveIMU swerveIMU,
      double maxSpeed,
      boolean invertedIMU)
  {
    this.moduleCount = moduleConfigs.length;
    this.imu = swerveIMU;
    this.maxSpeed = maxSpeed;
    this.attainableMaxRotationalVelocityRadiansPerSecond = 0;
    this.attainableMaxTranslationalSpeedMetersPerSecond = 0;
    this.invertedIMU = invertedIMU;
    this.modules = createModules(moduleConfigs);
    this.moduleLocationsMeters = new Translation2d[moduleConfigs.length];
    for (SwerveModule module : modules)
    {
      this.moduleLocationsMeters[module.moduleNumber] = module.configuration.moduleLocation;
    }
  }

  /**
   * Create modules based off of the SwerveModuleConfiguration.
   *
   * @param swerves Swerve constants.
   * @return Swerve Modules.
   */
  public SwerveModule[] createModules(SwerveModuleConfiguration[] swerves)
  {
    SwerveModule[] modArr = new SwerveModule[swerves.length];
    for (int i = 0; i < swerves.length; i++)
    {
      modArr[i] = new SwerveModule(i, swerves[i]);
    }
    return modArr;
  }
}

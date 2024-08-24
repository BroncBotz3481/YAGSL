package swervelib.parser;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import swervelib.SwerveModule;
import swervelib.imu.SwerveIMU;

/**
 * Swerve drive configurations used during SwerveDrive construction.
 */
public class SwerveDriveConfiguration
{

  /**
   * Number of modules on the robot.
   */
  public final int                                 moduleCount;
  /**
   * Swerve Module locations.
   */
  public       Translation2d[]                     moduleLocationsMeters;
  /**
   * Swerve IMU
   */
  public       SwerveIMU                           imu;
  /**
   * Swerve Modules.
   */
  public       SwerveModule[]                      modules;
  /**
   * Physical characteristics of the swerve drive from physicalproperties.json.
   */
  public       SwerveModulePhysicalCharacteristics physicalCharacteristics;

  /**
   * Create swerve drive configuration.
   *
   * @param moduleConfigs           Module configuration.
   * @param swerveIMU               Swerve IMU.
   * @param invertedIMU             Invert the IMU.
   * @param driveFeedforward        The drive motor feedforward to use for the {@link SwerveModule}.
   * @param physicalCharacteristics {@link SwerveModulePhysicalCharacteristics} to store in association with self.
   */
  public SwerveDriveConfiguration(
      SwerveModuleConfiguration[] moduleConfigs,
      SwerveIMU swerveIMU,
      boolean invertedIMU,
      SimpleMotorFeedforward driveFeedforward,
      SwerveModulePhysicalCharacteristics physicalCharacteristics)
  {
    this.moduleCount = moduleConfigs.length;
    this.imu = swerveIMU;
    swerveIMU.setInverted(invertedIMU);
    this.modules = createModules(moduleConfigs, driveFeedforward);
    this.moduleLocationsMeters = new Translation2d[moduleConfigs.length];
    for (SwerveModule module : modules)
    {
      this.moduleLocationsMeters[module.moduleNumber] = module.configuration.moduleLocation;
    }
    this.physicalCharacteristics = physicalCharacteristics;
  }

  /**
   * Create modules based off of the SwerveModuleConfiguration.
   *
   * @param swerves          Swerve constants.
   * @param driveFeedforward Drive feedforward created using
   *                         {@link swervelib.math.SwerveMath#createDriveFeedforward(double, double, double)}.
   * @return Swerve Modules.
   */
  public SwerveModule[] createModules(SwerveModuleConfiguration[] swerves, SimpleMotorFeedforward driveFeedforward)
  {
    SwerveModule[] modArr = new SwerveModule[swerves.length];
    for (int i = 0; i < swerves.length; i++)
    {
      modArr[i] = new SwerveModule(i, swerves[i], driveFeedforward);
    }
    return modArr;
  }

  /**
   * Calculate the Drive Base Radius
   *
   * @return Drive base radius from center of robot to the farthest wheel in meters.
   */
  public double getDriveBaseRadiusMeters()
  {
    Translation2d centerOfModules = moduleLocationsMeters[0];

    //Calculate the Center by adding all module offsets together.
    for (int i = 1; i < moduleLocationsMeters.length; i++)
    {
      centerOfModules = centerOfModules.plus(moduleLocationsMeters[i]);
    }

    //Return Largest Radius
    return centerOfModules.getDistance(moduleLocationsMeters[0]);
  }
}

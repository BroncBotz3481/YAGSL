package swervelib.parser;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import swervelib.SwerveModule;
import swervelib.imu.NavXSwerve;
import swervelib.imu.Pigeon2Swerve;
import swervelib.imu.SwerveIMU;
import swervelib.math.SwerveMath;

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
   * @param physicalCharacteristics {@link SwerveModulePhysicalCharacteristics} to store in association with self.
   */
  public SwerveDriveConfiguration(
      SwerveModuleConfiguration[] moduleConfigs,
      SwerveIMU swerveIMU,
      boolean invertedIMU,
      SwerveModulePhysicalCharacteristics physicalCharacteristics)
  {
    this.moduleCount = moduleConfigs.length;
    this.imu = swerveIMU;
    swerveIMU.setInverted(invertedIMU);
    this.modules = createModules(moduleConfigs);
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

  /**
   * Get the trackwidth of the swerve modules.
   *
   * @return Effective trackwdtih in Meters
   */
  public double getTrackwidth()
  {
    SwerveModuleConfiguration fr = SwerveMath.getSwerveModule(modules, true, false);
    SwerveModuleConfiguration fl = SwerveMath.getSwerveModule(modules, true, true);
    return fr.moduleLocation.getDistance(fl.moduleLocation);
  }

  /**
   * Get the tracklength of the swerve modules.
   *
   * @return Effective tracklength in Meters
   */
  public double getTracklength()
  {
    SwerveModuleConfiguration br = SwerveMath.getSwerveModule(modules, false, false);
    SwerveModuleConfiguration bl = SwerveMath.getSwerveModule(modules, false, true);
    return br.moduleLocation.getDistance(bl.moduleLocation);
  }

  /**
   * Get the {@link DCMotor} corresponding to the first module's configuration.
   *
   * @return {@link DCMotor} of the drive motor.
   */
  public DCMotor getDriveMotorSim()
  {
    SwerveModuleConfiguration fl = SwerveMath.getSwerveModule(modules, true, true);
    return fl.driveMotor.getSimMotor();
  }

  /**
   * Get the {@link DCMotor} corresponding to the first module configuration.
   *
   * @return {@link DCMotor} of the angle motor.
   */
  public DCMotor getAngleMotorSim()
  {
    SwerveModuleConfiguration fl = SwerveMath.getSwerveModule(modules, true, true);
    return fl.angleMotor.getSimMotor();
  }

  /**
   * Get the gyro simulation for the robot.
   *
   * @return {@link GyroSimulation} gyro simulation.
   */
  public Supplier<GyroSimulation> getGyroSim()
  {
    if (imu instanceof Pigeon2Swerve)
    {
      return COTS.ofPigeon2();
    } else if (imu instanceof NavXSwerve)
    {
      return COTS.ofNav2X();
    }
    return COTS.ofGenericGyro();
  }

}

package swervelib.parser.json.modules;

import swervelib.math.SwerveMath;
import swervelib.telemetry.Alert;
import swervelib.telemetry.Alert.AlertType;

/**
 * Angle motor conversion factors composite JSON parse class.
 */
public class AngleConversionFactorsJson
{

  /**
   * Gear ratio for the angle/steering/azimuth motor on the Swerve Module. Motor rotations to 1 wheel rotation.
   */
  public double gearRatio = 0;
  /**
   * Calculated or given conversion factor.
   */
  public double factor    = 0;

  /**
   * Calculate the drive conversion factor.
   *
   * @return Drive conversion factor, if factor isn't set.
   */
  public double calculate()
  {
    if (factor != 0 && gearRatio != 0)
    {
      new Alert("Configuration",
                "The given angle conversion factor takes precedence over the composite conversion factor, please remove 'factor' if you want to use the composite factor instead.",
                AlertType.WARNING).set(true);
    }
    if (factor == 0)
    {
      factor = SwerveMath.calculateDegreesPerSteeringRotation(gearRatio);
    }
    return factor;
  }
}

package swervelib.parser.json.modules;

import swervelib.math.SwerveMath;

/**
 * Angle motor conversion factors composite JSON parse class.
 */
public class AngleConversionFactorsJson
{

  /**
   * Gear ratio for the angle/steering/azimuth motor on the Swerve Module. Motor rotations to 1 wheel rotation.
   */
  public double gearRatio;
  /**
   * Calculated or given conversion factor.
   */
  public double factor = 0;

  /**
   * Calculate the drive conversion factor.
   *
   * @return Drive conversion factor, if factor isn't set.
   */
  public double calculate()
  {
    if (factor == 0)
    {
      factor = SwerveMath.calculateDegreesPerSteeringRotation(gearRatio);
    }
    return factor;
  }
}

package swervelib.parser.json.modules;

import edu.wpi.first.math.util.Units;
import swervelib.math.SwerveMath;
import swervelib.telemetry.Alert;
import swervelib.telemetry.Alert.AlertType;

/**
 * Drive motor composite JSON parse class.
 */
public class DriveConversionFactorsJson
{

  /**
   * Gear ratio for the drive motor rotations to turn the wheel 1 complete rotation.
   */
  public double gearRatio = 0;
  /**
   * Diameter of the wheel in inches.
   */
  public double diameter  = 0;
  /**
   * Calculated conversion factor.
   */
  public double factor    = 0;

  /**
   * Calculate the drive conversion factor.
   *
   * @return Drive conversion factor, if factor isn't set.
   */
  public double calculate()
  {
    if (factor != 0 && (diameter != 0 || gearRatio != 0))
    {
      new Alert("Configuration",
                "The given drive conversion factor takes precedence over the composite conversion factor, please remove 'factor' if you want to use the composite factor instead.",
                AlertType.WARNING).set(true);
    }
    if (factor == 0)
    {
      factor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(this.diameter), this.gearRatio);
    }
    return factor;
  }
}

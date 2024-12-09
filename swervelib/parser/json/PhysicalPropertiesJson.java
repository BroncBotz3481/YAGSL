package swervelib.parser.json;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import swervelib.parser.SwerveModulePhysicalCharacteristics;
import swervelib.parser.json.modules.ConversionFactorsJson;

/**
 * {@link swervelib.parser.SwerveModulePhysicalCharacteristics} parsed data. Used to configure the SwerveModule.
 */
public class PhysicalPropertiesJson
{

  /**
   * DEPRECATED! Use {@link PhysicalPropertiesJson#conversionFactors} instead.
   */
  @Deprecated(since = "2025", forRemoval = true)
  public MotorConfigDouble     conversionFactor               = new MotorConfigDouble();
  /**
   * Minimum voltage to spin the module or wheel.
   */
  public MotorConfigDouble     friction                       = new MotorConfigDouble(0.3, 0.2);
  /**
   * Steer rotational inertia in KilogramMetersSquare.
   */
  public double                steerRotationalInertia         = 0.03;
  /**
   * Robot mass in lb (pounds)
   */
  public double                robotMass                      = 110.2311;
  /**
   * Conversion Factors composition. Auto-calculates the conversion factors.
   */
  public ConversionFactorsJson conversionFactors              = new ConversionFactorsJson();
  /**
   * The current limit in AMPs to apply to the motors.
   */
  public MotorConfigInt        currentLimit                   = new MotorConfigInt(40, 20);
  /**
   * The minimum number of seconds to take for the motor to go from 0 to full throttle.
   */
  public MotorConfigDouble     rampRate                       = new MotorConfigDouble(0.25, 0.25);
  /**
   * The grip tape coefficient of friction on carpet. Used to calculate the practical maximum acceleration.
   */
  public double                wheelGripCoefficientOfFriction = 1.19;
  /**
   * The voltage to use for the smart motor voltage compensation, default is 12.
   */
  public double                optimalVoltage                 = 12;

  /**
   * Create the physical characteristics based off the parsed data.
   *
   * @return {@link SwerveModulePhysicalCharacteristics} based on parsed data.
   */
  public SwerveModulePhysicalCharacteristics createPhysicalProperties()
  {
    // Setup deprecation notice.
    if (conversionFactor.drive != 0 && conversionFactor.angle != 0 && conversionFactors.isDriveEmpty() &&
        conversionFactors.isAngleEmpty())
    {
      new Alert("Configuration",
                "\n'conversionFactor': {'drive': " + conversionFactor.drive + ", 'angle': " + conversionFactor.angle +
                "} \nis deprecated, please use\n" +
                "'conversionFactors': {'drive': {'factor': " + conversionFactor.drive + "}, 'angle': {'factor': " +
                conversionFactor.angle + "} }",
                AlertType.kError).set(true);
    }

    return new SwerveModulePhysicalCharacteristics(
        conversionFactors,
        wheelGripCoefficientOfFriction,
        optimalVoltage,
        currentLimit.drive,
        currentLimit.angle,
        rampRate.drive,
        rampRate.angle,
        friction.drive,
        friction.angle,
        steerRotationalInertia,
        Units.Pounds.of(robotMass).in(Units.Kilogram));
  }
}


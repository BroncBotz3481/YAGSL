package swervelib.parser.json;

import swervelib.parser.SwerveModulePhysicalCharacteristics;

/**
 * {@link swervelib.parser.SwerveModulePhysicalCharacteristics} parsed data. Used to configure the SwerveModule.
 */
public class PhysicalPropertiesJson
{


  /**
   * Conversion factor applied to the motor controllers PID loops. Can be calculated with
   * {@link swervelib.math.SwerveMath#calculateDegreesPerSteeringRotation(double, double)} for angle motors or
   * {@link swervelib.math.SwerveMath#calculateMetersPerRotation(double, double, double)} for drive motors.
   */
  public MotorConfigDouble conversionFactor               = new MotorConfigDouble(0, 0);
  /**
   * The current limit in AMPs to apply to the motors.
   */
  public MotorConfigInt    currentLimit                   = new MotorConfigInt(40, 20);
  /**
   * The minimum number of seconds to take for the motor to go from 0 to full throttle.
   */
  public MotorConfigDouble rampRate                       = new MotorConfigDouble(0.25, 0.25);
  /**
   * The grip tape coefficient of friction on carpet. Used to calculate the practical maximum acceleration.
   */
  public double            wheelGripCoefficientOfFriction = 1.19;
  /**
   * The voltage to use for the smart motor voltage compensation, default is 12.
   */
  public double            optimalVoltage                 = 12;

  /**
   * Create the physical characteristics based off the parsed data.
   *
   * @return {@link SwerveModulePhysicalCharacteristics} based on parsed data.
   */
  public SwerveModulePhysicalCharacteristics createPhysicalProperties()
  {
    return new SwerveModulePhysicalCharacteristics(
        conversionFactor,
        wheelGripCoefficientOfFriction,
        optimalVoltage,
        currentLimit.drive,
        currentLimit.angle,
        rampRate.drive,
        rampRate.angle);
  }
}


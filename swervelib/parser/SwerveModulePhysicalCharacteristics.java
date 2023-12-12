package swervelib.parser;

import swervelib.parser.json.MotorConfigDouble;

/**
 * Configuration class which stores physical characteristics shared between every swerve module.
 */
public class SwerveModulePhysicalCharacteristics
{

  /**
   * Current limits for the Swerve Module.
   */
  public final int driveMotorCurrentLimit, angleMotorCurrentLimit;
  /**
   * The time it takes for the motor to go from 0 to full throttle in seconds.
   */
  public final double driveMotorRampRate, angleMotorRampRate;
  /**
   * Wheel grip tape coefficient of friction on carpet, as described by the vendor.
   */
  public final double            wheelGripCoefficientOfFriction;
  /**
   * The voltage to use for the smart motor voltage compensation.
   */
  public       double            optimalVoltage;
  /**
   * The conversion factors for the drive and angle motors, created by
   * {@link swervelib.math.SwerveMath#calculateMetersPerRotation(double, double, double)} and
   * {@link swervelib.math.SwerveMath#calculateDegreesPerSteeringRotation(double, double)}.
   */
  public       MotorConfigDouble conversionFactor;

  /**
   * Construct the swerve module physical characteristics.
   *
   * @param conversionFactors              The conversion factors for the drive and angle motors, created by
   *                                       {@link swervelib.math.SwerveMath#calculateMetersPerRotation(double, double,
   *                                       double)} and
   *                                       {@link swervelib.math.SwerveMath#calculateDegreesPerSteeringRotation(double,
   *                                       double)}.
   * @param wheelGripCoefficientOfFriction Wheel grip coefficient of friction on carpet given by manufacturer.
   * @param optimalVoltage                 Optimal robot voltage.
   * @param driveMotorCurrentLimit         Current limit for the drive motor.
   * @param angleMotorCurrentLimit         Current limit for the angle motor.
   * @param driveMotorRampRate             The time in seconds to go from 0 to full throttle on the motor. (Prevents
   *                                       over drawing power from battery)
   * @param angleMotorRampRate             The time in seconds to go from 0 to full throttle on the motor. (Prevents
   *                                       overdrawing power and power loss).
   */
  public SwerveModulePhysicalCharacteristics(
      MotorConfigDouble conversionFactors,
      double wheelGripCoefficientOfFriction,
      double optimalVoltage,
      int driveMotorCurrentLimit,
      int angleMotorCurrentLimit,
      double driveMotorRampRate,
      double angleMotorRampRate)
  {
    this.wheelGripCoefficientOfFriction = wheelGripCoefficientOfFriction;
    this.optimalVoltage = optimalVoltage;

    this.conversionFactor = conversionFactors;
    // Set the conversion factors to null if they are both 0.
    if (conversionFactors != null)
    {
      if (conversionFactors.angle == 0 && conversionFactors.drive == 0)
      {
        this.conversionFactor = null;
      }
    }

    this.driveMotorCurrentLimit = driveMotorCurrentLimit;
    this.angleMotorCurrentLimit = angleMotorCurrentLimit;
    this.driveMotorRampRate = driveMotorRampRate;
    this.angleMotorRampRate = angleMotorRampRate;
  }

  /**
   * Construct the swerve module physical characteristics. Assume coefficient of friction is 1.19 (taken from blue
   * nitrile on carpet from Studica) and optimal voltage is 12v. Assumes the drive motor current limit is 40A, and the
   * angle motor current limit is 20A.
   *
   * @param conversionFactors  The conversion factors for the drive and angle motors, created by
   *                           {@link swervelib.math.SwerveMath#calculateMetersPerRotation(double, double, double)} and
   *                           {@link swervelib.math.SwerveMath#calculateDegreesPerSteeringRotation(double, double)}.
   * @param driveMotorRampRate The time in seconds to go from 0 to full throttle on the motor. (Prevents over drawing
   *                           power from battery)
   * @param angleMotorRampRate The time in seconds to go from 0 to full throttle on the motor. (Prevents overdrawing
   *                           power and power loss).
   */
  public SwerveModulePhysicalCharacteristics(
      MotorConfigDouble conversionFactors,
      double driveMotorRampRate,
      double angleMotorRampRate)
  {
    this(
        conversionFactors,
        1.19,
        12,
        40,
        20,
        driveMotorRampRate,
        angleMotorRampRate);
  }
}

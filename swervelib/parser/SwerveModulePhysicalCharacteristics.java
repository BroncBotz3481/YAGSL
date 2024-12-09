package swervelib.parser;

import swervelib.parser.json.modules.ConversionFactorsJson;

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
   * The minimum voltage to spin the module or wheel.
   */
  public final double driveFrictionVoltage, angleFrictionVoltage;
  /**
   * Wheel grip tape coefficient of friction on carpet, as described by the vendor.
   */
  public final double                wheelGripCoefficientOfFriction;
  /**
   * Steer rotational inertia in (KilogramSquareMeters) kg/m_sq.
   */
  public final double                steerRotationalInertia;
  /**
   * Robot mass in Kilograms.
   */
  public final double                robotMassKg;
  /**
   * The voltage to use for the smart motor voltage compensation.
   */
  public       double                optimalVoltage;
  /**
   * The conversion factors for the drive and angle motors, created by
   * {@link swervelib.math.SwerveMath#calculateMetersPerRotation(double, double, double)} and
   * {@link swervelib.math.SwerveMath#calculateDegreesPerSteeringRotation(double, double)}.
   */
  public       ConversionFactorsJson conversionFactor;

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
   * @param angleFrictionVoltage           Angle motor minimum voltage.
   * @param driveFrictionVoltage           Drive motor minimum voltage.
   * @param steerRotationalInertia         Steering rotational inertia in KilogramSquareMeters.
   * @param robotMassKg                    Robot mass in kG.
   */
  public SwerveModulePhysicalCharacteristics(
      ConversionFactorsJson conversionFactors,
      double wheelGripCoefficientOfFriction,
      double optimalVoltage,
      int driveMotorCurrentLimit,
      int angleMotorCurrentLimit,
      double driveMotorRampRate,
      double angleMotorRampRate,
      double driveFrictionVoltage,
      double angleFrictionVoltage,
      double steerRotationalInertia,
      double robotMassKg)
  {
    this.wheelGripCoefficientOfFriction = wheelGripCoefficientOfFriction;
    this.optimalVoltage = optimalVoltage;

    this.conversionFactor = conversionFactors;
    // Set the conversion factors to null if they are both 0.
    if (conversionFactors != null)
    {
      if (conversionFactors.isAngleEmpty() && conversionFactors.isDriveEmpty())
      {
        this.conversionFactor = null;
      }
    }

    this.driveMotorCurrentLimit = driveMotorCurrentLimit;
    this.angleMotorCurrentLimit = angleMotorCurrentLimit;
    this.driveMotorRampRate = driveMotorRampRate;
    this.angleMotorRampRate = angleMotorRampRate;
    this.driveFrictionVoltage = driveFrictionVoltage;
    this.angleFrictionVoltage = angleFrictionVoltage;
    this.steerRotationalInertia = steerRotationalInertia;
    this.robotMassKg = robotMassKg;
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
      ConversionFactorsJson conversionFactors,
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
        angleMotorRampRate,
        0.2,
        0.3,
        0.03,
        50);
  }
}

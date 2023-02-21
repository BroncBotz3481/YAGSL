package swervelib.parser;

import static swervelib.math.SwerveMath.calculateAngleKV;
import static swervelib.math.SwerveMath.calculateDegreesPerSteeringRotation;
import static swervelib.math.SwerveMath.calculateMaxAcceleration;
import static swervelib.math.SwerveMath.calculateMetersPerRotation;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.motors.SwerveMotor;

/**
 * Swerve Module configuration class which is used to configure {@link swervelib.SwerveModule}.
 */
public class SwerveModuleConfiguration
{

  /**
   * Angle offset in degrees for the Swerve Module.
   */
  public final double                              angleOffset;
  /**
   * Whether the absolute encoder is inverted.
   */
  public final boolean                             absoluteEncoderInverted;
  /**
   * State of inversion of the drive motor.
   */
  public final boolean                             driveMotorInverted;
  /**
   * State of inversion of the angle motor.
   */
  public final boolean                             angleMotorInverted;
  /**
   * Maximum robot speed in meters per second.
   */
  public final double                              maxSpeed;
  /**
   * PIDF configuration options for the angle motor closed-loop PID controller.
   */
  public       PIDFConfig                          anglePIDF;
  /**
   * PIDF configuration options for the drive motor closed-loop PID controller.
   */
  public       PIDFConfig                          velocityPIDF;
  /**
   * Angle volt-meter-per-second.
   */
  public       double                              angleKV;
  /**
   * Swerve module location relative to the robot.
   */
  public       Translation2d                       moduleLocation;
  /**
   * Physical characteristics of the swerve module.
   */
  public       SwerveModulePhysicalCharacteristics physicalCharacteristics;
  /**
   * The drive motor and angle motor of this swerve module.
   */
  public       SwerveMotor                         driveMotor, angleMotor;
  /**
   * The Absolute Encoder for the swerve module.
   */
  public SwerveAbsoluteEncoder absoluteEncoder;

  /**
   * Construct a configuration object for swerve modules.
   *
   * @param driveMotor              Drive {@link SwerveMotor}.
   * @param angleMotor              Angle {@link SwerveMotor}
   * @param absoluteEncoder         Absolute encoder {@link SwerveAbsoluteEncoder}.
   * @param angleOffset             Absolute angle offset to 0.
   * @param absoluteEncoderInverted Absolute encoder inverted.
   * @param angleMotorInverted      State of inversion of the angle motor.
   * @param driveMotorInverted      Drive motor inverted.
   * @param xMeters                 Module location in meters from the center horizontally.
   * @param yMeters                 Module location in meters from center vertically.
   * @param anglePIDF               Angle PIDF configuration.
   * @param velocityPIDF            Velocity PIDF configuration.
   * @param maxSpeed                Maximum speed in meters per second.
   * @param physicalCharacteristics Physical characteristics of the swerve module.
   */
  public SwerveModuleConfiguration(
      SwerveMotor driveMotor,
      SwerveMotor angleMotor,
      SwerveAbsoluteEncoder absoluteEncoder,
      double angleOffset,
      double xMeters,
      double yMeters,
      PIDFConfig anglePIDF,
      PIDFConfig velocityPIDF,
      double maxSpeed,
      SwerveModulePhysicalCharacteristics physicalCharacteristics,
      boolean absoluteEncoderInverted,
      boolean driveMotorInverted,
      boolean angleMotorInverted)
  {
    this.driveMotor = driveMotor;
    this.angleMotor = angleMotor;
    this.absoluteEncoder = absoluteEncoder;
    this.angleOffset = angleOffset;
    this.absoluteEncoderInverted = absoluteEncoderInverted;
    this.driveMotorInverted = driveMotorInverted;
    this.angleMotorInverted = angleMotorInverted;
    this.moduleLocation = new Translation2d(xMeters, yMeters);
    this.anglePIDF = anglePIDF;
    this.velocityPIDF = velocityPIDF;
    this.maxSpeed = maxSpeed;
    this.angleKV =
        calculateAngleKV(
            physicalCharacteristics.optimalVoltage,
            physicalCharacteristics.angleMotorFreeSpeedRPM,
            physicalCharacteristics.angleGearRatio);
    this.physicalCharacteristics = physicalCharacteristics;
  }

  /**
   * Construct a configuration object for swerve modules. Assumes the absolute encoder and drive motor are not
   * inverted.
   *
   * @param driveMotor              Drive {@link SwerveMotor}.
   * @param angleMotor              Angle {@link SwerveMotor}
   * @param absoluteEncoder         Absolute encoder {@link SwerveAbsoluteEncoder}.
   * @param angleOffset             Absolute angle offset to 0.
   * @param xMeters                 Module location in meters from the center horizontally.
   * @param yMeters                 Module location in meters from center vertically.
   * @param anglePIDF               Angle PIDF configuration.
   * @param velocityPIDF            Velocity PIDF configuration.
   * @param maxSpeed                Maximum robot speed in meters per second.
   * @param physicalCharacteristics Physical characteristics of the swerve module.
   */
  public SwerveModuleConfiguration(
      SwerveMotor driveMotor,
      SwerveMotor angleMotor,
      SwerveAbsoluteEncoder absoluteEncoder,
      double angleOffset,
      double xMeters,
      double yMeters,
      PIDFConfig anglePIDF,
      PIDFConfig velocityPIDF,
      double maxSpeed,
      SwerveModulePhysicalCharacteristics physicalCharacteristics)
  {
    this(
        driveMotor,
        angleMotor,
        absoluteEncoder,
        angleOffset,
        xMeters,
        yMeters,
        anglePIDF,
        velocityPIDF,
        maxSpeed,
        physicalCharacteristics,
        false,
        false,
        false);
  }

  /**
   * Create the drive feedforward for swerve modules.
   *
   * @return Drive feedforward for drive motor on a swerve module.
   */
  public SimpleMotorFeedforward createDriveFeedforward()
  {
    double kv = physicalCharacteristics.optimalVoltage / maxSpeed;
    /// ^ Volt-seconds per meter (max voltage divided by max speed)
    double ka =
        physicalCharacteristics.optimalVoltage
        / calculateMaxAcceleration(physicalCharacteristics.wheelGripCoefficientOfFriction);
    /// ^ Volt-seconds^2 per meter (max voltage divided by max accel)
    return new SimpleMotorFeedforward(0, kv, ka);
  }

  /**
   * Get the encoder conversion for position encoders.
   *
   * @param isDriveMotor For the drive motor.
   * @return Position encoder conversion factor.
   */
  public double getPositionEncoderConversion(boolean isDriveMotor)
  {
    return isDriveMotor
           ? calculateMetersPerRotation(
        physicalCharacteristics.wheelDiameter,
        physicalCharacteristics.driveGearRatio,
        physicalCharacteristics.driveEncoderPulsePerRotation)
           : calculateDegreesPerSteeringRotation(
               physicalCharacteristics.angleGearRatio,
               physicalCharacteristics.angleEncoderPulsePerRotation);
  }
}

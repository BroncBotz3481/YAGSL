package swervelib.parser;

/**
 * Configuration class which stores physical characteristics shared between every swerve module.
 */
public class SwerveModulePhysicalCharacteristics
{

  /**
   * Wheel diameter in meters.
   */
  public final double wheelDiameter;
  /**
   * Drive gear ratio. How many times the motor has to spin before the wheel makes a complete rotation.
   */
  public final double driveGearRatio;
  /**
   * Angle gear ratio. How many times the motor has to spin before the wheel makes a full rotation.
   */
  public final double angleGearRatio;
  /**
   * Drive motor encoder pulse per rotation. 1 if integrated encoder.
   */
  public final int    driveEncoderPulsePerRotation;
  /**
   * Angle motor encoder pulse per rotation. 1 for Neo encoder. 2048 for Falcons.
   */
  public final int    angleEncoderPulsePerRotation;
  /**
   * Optimal voltage of the robot.
   */
  public final double optimalVoltage;
  /**
   * Current limits for the Swerve Module.
   */
  public final int    driveMotorCurrentLimit, angleMotorCurrentLimit;
  /**
   * The time it takes for the motor to go from 0 to full throttle in seconds.
   */
  public final double driveMotorRampRate, angleMotorRampRate;
  /**
   * Wheel grip tape coefficient of friction on carpet, as described by the vendor.
   */
  public final double wheelGripCoefficientOfFriction;
  /**
   * Angle motor kV used for second order kinematics to tune the feedforward, this variable should be adjusted so that
   * your drive train does not drift towards the direction you are rotating while you translate. When set to 0 the
   * calculated kV will be used.
   */
  public final double moduleSteerFFCL;

  /**
   * Construct the swerve module physical characteristics.
   *
   * @param driveGearRatio                 Gear ratio of the drive motor. Number of motor rotations to rotate the
   *                                       wheel.
   * @param angleGearRatio                 Gear ratio of the angle motor. Number of motor rotations to spin the wheel.
   * @param wheelDiameter                  Wheel diameter in meters.
   * @param wheelGripCoefficientOfFriction Wheel grip coefficient of friction on carpet given by manufacturer.
   * @param optimalVoltage                 Optimal robot voltage.
   * @param driveMotorCurrentLimit         Current limit for the drive motor.
   * @param angleMotorCurrentLimit         Current limit for the angle motor.
   * @param driveMotorRampRate             The time in seconds to go from 0 to full throttle on the motor. (Prevents
   *                                       over drawing power from battery)
   * @param angleMotorRampRate             The time in seconds to go from 0 to full throttle on the motor. (Prevents
   *                                       overdrawing power and power loss).
   * @param driveEncoderPulsePerRotation   The number of encoder pulses per motor rotation, 1 for integrated encoders.
   * @param angleEncoderPulsePerRotation   The number of encoder pulses per motor rotation, 1 for integrated encoders.
   * @param moduleSteerFFCL                The kV applied to the steering motor to ensure your drivetrain does not drift
   *                                       towards a direction when rotating while translating.
   */
  public SwerveModulePhysicalCharacteristics(
      double driveGearRatio,
      double angleGearRatio,
      double wheelDiameter,
      double wheelGripCoefficientOfFriction,
      double optimalVoltage,
      int driveMotorCurrentLimit,
      int angleMotorCurrentLimit,
      double driveMotorRampRate,
      double angleMotorRampRate,
      int driveEncoderPulsePerRotation,
      int angleEncoderPulsePerRotation,
      double moduleSteerFFCL)
  {
    this.wheelGripCoefficientOfFriction = wheelGripCoefficientOfFriction;
    this.optimalVoltage = optimalVoltage;

    this.angleGearRatio = angleGearRatio;
    this.driveGearRatio = driveGearRatio;
    this.angleEncoderPulsePerRotation = angleEncoderPulsePerRotation;
    this.driveEncoderPulsePerRotation = driveEncoderPulsePerRotation;
    this.wheelDiameter = wheelDiameter;

    this.driveMotorCurrentLimit = driveMotorCurrentLimit;
    this.angleMotorCurrentLimit = angleMotorCurrentLimit;
    this.driveMotorRampRate = driveMotorRampRate;
    this.angleMotorRampRate = angleMotorRampRate;
    this.moduleSteerFFCL = moduleSteerFFCL;
  }

  /**
   * Construct the swerve module physical characteristics. Assume coefficient of friction is 1.19 (taken from blue
   * nitrile on carpet from Studica) and optimal voltage is 12v. Assumes the drive motor current limit is 40A, and the
   * angle motor current limit is 20A.
   *
   * @param driveGearRatio               Gear ratio of the drive motor. Number of motor rotations to rotate the wheel.
   * @param angleGearRatio               Gear ratio of the angle motor. Number of motor rotations to spin the wheel.
   * @param wheelDiameter                Wheel diameter in meters.
   * @param driveMotorRampRate           The time in seconds to go from 0 to full throttle on the motor. (Prevents over
   *                                     drawing power from battery)
   * @param angleMotorRampRate           The time in seconds to go from 0 to full throttle on the motor. (Prevents
   *                                     overdrawing power and power loss).
   * @param driveEncoderPulsePerRotation The number of encoder pulses per motor rotation, 1 for integrated encoders.
   * @param angleEncoderPulsePerRotation The number of encoder pulses per motor rotation, 1 for integrated encoders.
   */
  public SwerveModulePhysicalCharacteristics(
      double driveGearRatio,
      double angleGearRatio,
      double wheelDiameter,
      double driveMotorRampRate,
      double angleMotorRampRate,
      int driveEncoderPulsePerRotation,
      int angleEncoderPulsePerRotation)
  {
    this(
        driveGearRatio,
        angleGearRatio,
        wheelDiameter,
        1.19,
        12,
        40,
        20,
        driveMotorRampRate,
        angleMotorRampRate,
        driveEncoderPulsePerRotation,
        angleEncoderPulsePerRotation, -0.30);
  }
}

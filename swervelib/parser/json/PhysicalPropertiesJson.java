package swervelib.parser.json;

import edu.wpi.first.math.util.Units;
import swervelib.parser.SwerveModulePhysicalCharacteristics;

/**
 * {@link swervelib.parser.SwerveModulePhysicalCharacteristics} parsed data. Used to configure the SwerveModule.
 */
public class PhysicalPropertiesJson
{

  /**
   * Wheel diameter in inches.
   */
  public double            wheelDiameter;
  /**
   * Gear ratio for the motors, number of times the motor has to spin before the wheel rotates a single time.
   */
  public MotorConfigDouble gearRatio;
  /**
   * Encoder pulse per rotation for non-integrated encoders. 1 for integrated encoders.
   */
  public MotorConfigInt    encoderPulsePerRotation        = new MotorConfigInt(1, 1);
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
   * Angle motor free speed rotations per minute.
   */
  public double            angleMotorFreeSpeedRPM;

  /**
   * Create the physical characteristics based off the parsed data.
   *
   * @param optimalVoltage Optimal voltage to compensate for and use to calculate drive motor feedforward.
   * @return {@link SwerveModulePhysicalCharacteristics} based on parsed data.
   */
  public SwerveModulePhysicalCharacteristics createPhysicalProperties(double optimalVoltage)
  {
    return new SwerveModulePhysicalCharacteristics(
        gearRatio.drive,
        gearRatio.angle,
        angleMotorFreeSpeedRPM,
        Units.inchesToMeters(wheelDiameter),
        wheelGripCoefficientOfFriction,
        optimalVoltage,
        currentLimit.drive,
        currentLimit.angle,
        rampRate.drive,
        rampRate.angle,
        encoderPulsePerRotation.drive,
        encoderPulsePerRotation.angle);
  }
}

/**
 * Used to store doubles for motor configuration.
 */
class MotorConfigDouble
{

  /**
   * Drive motor.
   */
  public double drive;
  /**
   * Angle motor.
   */
  public double angle;

  /**
   * Default constructor.
   */
  public MotorConfigDouble()
  {
  }

  /**
   * Default constructor.
   *
   * @param angle Angle data.
   * @param drive Drive data.
   */
  public MotorConfigDouble(double angle, double drive)
  {
    this.angle = angle;
    this.drive = drive;
  }
}

/**
 * Used to store ints for motor configuration.
 */
class MotorConfigInt
{

  /**
   * Drive motor.
   */
  public int drive;
  /**
   * Angle motor.
   */
  public int angle;

  /**
   * Default constructor.
   */
  public MotorConfigInt()
  {
  }

  /**
   * Default constructor with values.
   *
   * @param drive Drive data.
   * @param angle Angle data.
   */
  public MotorConfigInt(int drive, int angle)
  {
    this.angle = angle;
    this.drive = drive;
  }
}

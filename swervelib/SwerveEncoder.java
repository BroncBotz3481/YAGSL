package frc.robot.subsystems.swervedrive.swerve;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.MagnetFieldStrength;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.swervedrive.swerve.encoders.CTRECANCoder;
import frc.robot.subsystems.swervedrive.swerve.encoders.PWMAnalogEncoder;
import frc.robot.subsystems.swervedrive.swerve.encoders.PWMDutyCycleEncoder;
import frc.robot.subsystems.swervedrive.swerve.encoders.REVAbsoluteEncoder;

/**
 * Swerve Encoder class definition for common interfaces.
 */
public abstract class SwerveEncoder<AbsoluteEncoderType>
{

  /**
   * The encoder can be directly referenced for configuration purposes of onboard PIDs.
   */
  public AbsoluteEncoderType m_encoder;

  /**
   * Get the SwerveEncoder class from the given encoder types
   *
   * @param encoder               Encoder
   * @param <AbsoluteEncoderType> One of DutyCycleEncoder, AnalogEncoder, AbsoluteEncoder, and CANCoder
   * @return SwerveEncoder subclass
   */
  public static <AbsoluteEncoderType> SwerveEncoder<?> fromEncoder(AbsoluteEncoderType encoder)
  {
    if (encoder instanceof CANCoder)
    {
      return new CTRECANCoder((CANCoder) encoder);
    } else if (encoder instanceof DutyCycleEncoder)
    {
      return new PWMDutyCycleEncoder((DutyCycleEncoder) encoder);
    } else if (encoder instanceof AnalogEncoder)
    {
      return new PWMAnalogEncoder((AnalogEncoder) encoder);
    } else if (encoder instanceof AbsoluteEncoder)
    {
      return new REVAbsoluteEncoder((AbsoluteEncoder) encoder);
    }

    return null;
  }

  /**
   * Configure the absolute encoder if possible.
   */
  public abstract void configure();

  /**
   * Reset encoder to factory default settings, if possible.
   */
  public abstract void factoryDefault();

  /**
   * Get the magnetic field strength, if available.
   *
   * @return CTRE MagneticFieldStrength Enum.
   */
  public abstract MagnetFieldStrength getMagnetFieldStrength();

  /**
   * Get the absolute position in degrees.
   *
   * @return Absolute position (0, 360]
   */
  public abstract double getAbsolutePosition();

  /**
   * Get the velocity of the absolute encoder in degrees per second.
   *
   * @return Velocity in degrees per second.
   */
  public abstract double getVelocity();

  /**
   * Configure the magnetic offset for the AbsoluteEncoder.
   *
   * @param offset Offset in degrees.
   */
  public abstract void setOffset(double offset);

  /**
   * Is the encoder reachable?
   *
   * @return True if reachable, false otherwise.
   */
  public abstract boolean reachable();

  /**
   * Configure the sensor direction
   *
   * @param isInverted Inverted or not.
   */
  public abstract void setInverted(boolean isInverted);

}

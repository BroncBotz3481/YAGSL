package frc.robot.subsystems.swervedrive.swerve.encoders;

import com.ctre.phoenix.sensors.MagnetFieldStrength;
import com.revrobotics.AbsoluteEncoder;
import frc.robot.subsystems.swervedrive.swerve.SwerveEncoder;

public class REVAbsoluteEncoder extends SwerveEncoder<AbsoluteEncoder>
{

  /**
   * Constructor for AbsoluteEncoder class.
   *
   * @param encoder Encoder to wrap around.
   */
  public REVAbsoluteEncoder(AbsoluteEncoder encoder)
  {
    m_encoder = encoder;
    configure();
  }

  /**
   * Configure the absolute encoder if possible.
   */
  @Override
  public void configure()
  {
    // Redundant, but ensures everything is always correct.
    m_encoder.setPositionConversionFactor(360);
    m_encoder.setVelocityConversionFactor(360.0 / 60.0);
  }

  /**
   * Reset encoder to factory default settings, if possible.
   */
  @Override
  public void factoryDefault()
  {
    // Nothing to do here.
  }

  /**
   * Get the magnetic field strength, if available.
   *
   * @return CTRE MagneticFieldStrength Enum.
   */
  @Override
  public MagnetFieldStrength getMagnetFieldStrength()
  {
    // Unable to check, assume green.
    return MagnetFieldStrength.Good_GreenLED;
  }

  /**
   * Get the absolute position in degrees.
   *
   * @return Absolute position (0, 360]
   */
  @Override
  public double getAbsolutePosition()
  {
    return m_encoder.getPosition();
  }

  /**
   * Get the velocity of the absolute encoder in degrees per second.
   *
   * @return Velocity in degrees per second.
   */
  @Override
  public double getVelocity()
  {
    return m_encoder.getVelocity();
  }

  /**
   * Configure the magnetic offset for the AbsoluteEncoder.
   *
   * @param offset Offset in degrees.
   */
  @Override
  public void setOffset(double offset)
  {
    m_encoder.setZeroOffset(offset);
  }

  /**
   * Is the encoder reachable?
   *
   * @return True if reachable, false otherwise.
   */
  @Override
  public boolean reachable()
  {
    return true;
  }

  /**
   * Configure the sensor direction.
   *
   * @param isInverted Inverted or not.
   */
  @Override
  public void setInverted(boolean isInverted)
  {
    m_encoder.setInverted(isInverted);
  }
}

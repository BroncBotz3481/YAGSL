package frc.robot.subsystems.swervedrive.swerve.encoders;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.MagnetFieldStrength;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import frc.robot.subsystems.swervedrive.swerve.SwerveEncoder;

public class CTRECANCoder extends SwerveEncoder<CANCoder>
{

  /**
   * Create SwerveEncoder based off CANCoder.
   *
   * @param encoder CANCoder to use.
   */
  public CTRECANCoder(CANCoder encoder)
  {
    m_encoder = encoder;
  }

  /**
   * Configure the absolute encoder if possible.
   */
  public void configure()
  {
    CANCoderConfiguration sensorConfig = new CANCoderConfiguration();

    sensorConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    sensorConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    sensorConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    m_encoder.configAllSettings(sensorConfig);
  }

  /**
   * Get the magnetic field strength, if available.
   *
   * @return CTRE MagneticFieldStrength Enum.
   */
  @Override
  public MagnetFieldStrength getMagnetFieldStrength()
  {
    return m_encoder.getMagnetFieldStrength();
  }

  /**
   * Get the absolute position in degrees.
   *
   * @return Absolute position (0, 360]
   */
  @Override
  public double getAbsolutePosition()
  {
    return m_encoder.getAbsolutePosition();
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
    m_encoder.configMagnetOffset(offset);
  }

  /**
   * Is the encoder reachable?
   *
   * @return True if reachable, false otherwise.
   */
  @Override
  public boolean reachable()
  {
    return m_encoder.getFirmwareVersion() > 0;
  }

  /**
   * Configure the sensor direction
   *
   * @param isInverted Inverted or not.
   */
  @Override
  public void setInverted(boolean isInverted)
  {
    m_encoder.configSensorDirection(isInverted);
  }

  /**
   * Reset encoder to factory default settings, if possible.
   */
  @Override
  public void factoryDefault()
  {
    m_encoder.configFactoryDefault();
  }
}

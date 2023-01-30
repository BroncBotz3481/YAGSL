package frc.robot.subsystems.swervedrive.swerve.encoders;

import com.ctre.phoenix.sensors.MagnetFieldStrength;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive.swerve.SwerveEncoder;

public class PWMDutyCycleEncoder extends SwerveEncoder<DutyCycleEncoder>
{

  private final Timer   m_timer;
  private       boolean m_inverted = false;
  private       double  m_lastAngle, m_velocity, m_lastTime;

  /**
   * Construct PWM DutyCycleEncoder class.
   *
   * @param encoder Encoder class.
   */
  public PWMDutyCycleEncoder(DutyCycleEncoder encoder)
  {
    m_timer = new Timer();
    m_timer.start();

    m_encoder = encoder;
    configure();

    m_lastTime = m_timer.get();
    m_lastAngle = m_encoder.getDistance();
    Robot.getInstance().addPeriodic(this::updateVelocity, 1);
  }

  /**
   * Updates the velocity every second.
   */
  public void updateVelocity()
  {
    double currentTime = m_timer.get(), currentAngle = m_encoder.getDistance();
    m_velocity = (currentAngle - m_lastAngle) / (currentTime - m_lastTime);
    m_lastTime = currentTime;
    m_lastAngle = currentAngle;
  }

  /**
   * Configure the absolute encoder if possible.
   */
  @Override
  public void configure()
  {
    m_encoder.setDistancePerRotation(360);
  }

  /**
   * Reset encoder to factory default settings, if possible.
   */
  @Override
  public void factoryDefault()
  {
    m_encoder.reset();
  }

  /**
   * Get the magnetic field strength, if available.
   *
   * @return CTRE MagneticFieldStrength Enum.
   */
  @Override
  public MagnetFieldStrength getMagnetFieldStrength()
  {
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
    return m_encoder.getDistance() * (m_inverted ? -1 : 1);
  }

  /**
   * Get the velocity of the absolute encoder in degrees per second.
   *
   * @return Velocity in degrees per second.
   */
  @Override
  public double getVelocity()
  {
    return m_velocity;
  }

  /**
   * Configure the magnetic offset for the AbsoluteEncoder.
   *
   * @param offset Offset in degrees.
   */
  @Override
  public void setOffset(double offset)
  {
    m_encoder.setPositionOffset((Rotation2d.fromDegrees(offset).getDegrees() + 180) / 360);
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
   * Configure the sensor direction
   *
   * @param isInverted Inverted or not.
   */
  @Override
  public void setInverted(boolean isInverted)
  {
    m_inverted = isInverted;
  }
}

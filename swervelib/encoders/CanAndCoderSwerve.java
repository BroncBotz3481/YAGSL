package swervelib.encoders;

import com.reduxrobotics.sensors.canandcoder.CANandcoder;

/**
 * HELIUM {@link CANandcoder} from ReduxRobotics absolute encoder, attached through the CAN bus.
 */
public class CanAndCoderSwerve extends SwerveAbsoluteEncoder
{

  /**
   * The {@link CANandcoder} representing the CANandCoder on the CAN bus.
   */
  public  CANandcoder encoder;
  /**
   * Inversion state of the encoder.
   */
  private boolean     inverted = false;

  /**
   * Create the {@link CANandcoder}
   *
   * @param canid The CAN ID whenever the CANandCoder is operating on the CANBus.
   */
  public CanAndCoderSwerve(int canid)
  {
    encoder = new CANandcoder(canid);
  }

  /**
   * Reset the encoder to factory defaults.
   */
  @Override
  public void factoryDefault()
  {
    encoder.resetFactoryDefaults(false);
  }

  /**
   * Clear sticky faults on the encoder.
   */
  @Override
  public void clearStickyFaults()
  {
    encoder.clearStickyFaults();
  }

  /**
   * Configure the CANandCoder to read from [0, 360) per second.
   *
   * @param inverted Whether the encoder is inverted.
   */
  @Override
  public void configure(boolean inverted)
  {
    this.inverted = inverted;
  }

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in degrees from [0, 360).
   */
  @Override
  public double getAbsolutePosition()
  {
    return (inverted ? -1.0 : 1.0) * encoder.getPosition() * 360;
  }

  /**
   * Get the instantiated absolute encoder Object.
   *
   * @return Absolute encoder object.
   */
  @Override
  public Object getAbsoluteEncoder()
  {
    return encoder;
  }

  /**
   * Get the velocity in degrees/sec.
   *
   * @return velocity in degrees/sec.
   */
  @Override
  public double getVelocity()
  {
    return encoder.getVelocity();
  }
}

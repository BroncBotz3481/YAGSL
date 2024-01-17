package swervelib.encoders;

import com.reduxrobotics.sensors.canandcoder.Canandcoder;

/**
 * HELIUM {@link Canandcoder} from ReduxRobotics absolute encoder, attached through the CAN bus.
 */
public class CanAndCoderSwerve extends SwerveAbsoluteEncoder
{

  /**
   * The {@link Canandcoder} representing the CANandCoder on the CAN bus.
   */
  public Canandcoder encoder;

  /**
   * Create the {@link Canandcoder}
   *
   * @param canid The CAN ID whenever the CANandCoder is operating on the CANBus.
   */
  public CanAndCoderSwerve(int canid)
  {
    encoder = new Canandcoder(canid);
  }

  /**
   * Reset the encoder to factory defaults.
   * <p>
   * This will not clear the stored zero offset.
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
   * Configure the Canandcoder to read from [0, 360) per second.
   *
   * @param inverted Whether the encoder is inverted.
   */
  @Override
  public void configure(boolean inverted)
  {
    encoder.setSettings(new Canandcoder.Settings().setInvertDirection(inverted));
  }

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in degrees from [0, 360).
   */
  @Override
  public double getAbsolutePosition()
  {
    return encoder.getAbsPosition() * 360;
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
   * Cannot set the offset of the Canandcoder.
   *
   * @param offset the offset the Absolute Encoder uses as the zero point.
   * @return true if setting the zero point succeeded, false otherwise
   */
  @Override
  public boolean setAbsoluteEncoderOffset(double offset)
  {
    return encoder.setSettings(new Canandcoder.Settings().setZeroOffset(offset));
  }

  /**
   * Get the velocity in degrees/sec.
   *
   * @return velocity in degrees/sec.
   */
  @Override
  public double getVelocity()
  {
    return encoder.getVelocity() * 360;
  }
}

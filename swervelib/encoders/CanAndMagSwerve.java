package swervelib.encoders;

import com.reduxrobotics.sensors.canandmag.Canandmag;

/**
 * HELIUM {@link Canandmag} from ReduxRobotics absolute encoder, attached through the CAN bus.
 */
public class CanAndMagSwerve extends SwerveAbsoluteEncoder
{

  /**
   * The {@link Canandmag} representing the CANandMag on the CAN bus.
   */
  public Canandmag encoder;

  /**
   * Create the {@link Canandmag}
   *
   * @param canid The CAN ID whenever the CANandMag is operating on the CANBus.
   */
  public CanAndMagSwerve(int canid)
  {
    encoder = new Canandmag(canid);
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
   * Configure the CANandMag to read from [0, 360) per second.
   *
   * @param inverted Whether the encoder is inverted.
   */
  @Override
  public void configure(boolean inverted)
  {
    encoder.setSettings(new Canandmag.Settings().setInvertDirection(inverted));
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
   * Cannot set the offset of the CANandMag.
   *
   * @param offset the offset the Absolute Encoder uses as the zero point.
   * @return true if setting the zero point succeeded, false otherwise
   */
  @Override
  public boolean setAbsoluteEncoderOffset(double offset)
  {
    return encoder.setSettings(new Canandmag.Settings().setZeroOffset(offset));
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

package swervelib.encoders;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;

/**
 * DutyCycle encoders such as "US Digital MA3 with DIO Output, the CTRE Mag Encoder, the Rev Hex Encoder, and the AM Mag
 * Encoder." attached via a DIO lane.
 * <p>
 * Credits to
 * <a href="https://github.com/p2reneker25/2035-YAGSL/blob/main/swervelib/encoders/DIODutyCycleEncoderSwerve.java">
 * p2reneker25</a> for building this.
 */
public class DIODutyCycleEncoderSwerve extends SwerveAbsoluteEncoder
{

  /**
   * Duty Cycle Encoder.
   */
  private final DutyCycleEncoder encoder;
  /**
   * Inversion state.
   */
  private       boolean          isInverted;
  /**
   * An {@link Alert}  for if the encoder cannot report accurate velocities.
   */
  private       Alert            inaccurateVelocities;
  /**
   * The Offset in degrees of the DIO absolute encoder.
   */
  private       double           offset;

  /**
   * Constructor for the DIO duty cycle encoder.
   *
   * @param pin DIO lane for the encoder.
   */
  public DIODutyCycleEncoderSwerve(int pin)
  {
    encoder = new DutyCycleEncoder(pin);
    Timer.delay(2);
    inaccurateVelocities = new Alert(
        "Encoders",
        "The DIO Duty Cycle encoder may not report accurate velocities!",
        AlertType.kWarning);

  }

  @Override
  public void close()
  {
    encoder.close();
  }

  /**
   * Configure the inversion state of the encoder.
   *
   * @param inverted Whether the encoder is inverted.
   */
  @Override
  public void configure(boolean inverted)
  {
    isInverted = inverted;
  }

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in degrees from [0, 360).
   */
  @Override
  public double getAbsolutePosition()
  {
    return (isInverted ? -1.0 : 1.0) * ((encoder.get() * 360) - offset);
  }

  /**
   * Get the encoder object.
   *
   * @return {@link DutyCycleEncoder} from the class.
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
    inaccurateVelocities.set(true);
    return encoder.get();
  }

  /**
   * Reset the encoder to factory defaults.
   */
  @Override
  public void factoryDefault()
  {
    // Do nothing
  }

  /**
   * Clear sticky faults on the encoder.
   */
  @Override
  public void clearStickyFaults()
  {
    // Do nothing
  }


  @Override
  public boolean setAbsoluteEncoderOffset(double offset)
  {
    this.offset = offset;

    return true;
  }
}
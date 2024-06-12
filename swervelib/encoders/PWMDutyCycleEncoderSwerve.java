package swervelib.encoders;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import swervelib.telemetry.Alert;

/**
 * DutyCycle encoders such as "US Digital MA3 with PWM Output, the CTRE Mag Encoder, the Rev Hex Encoder, and the AM Mag
 * Encoder." attached via a PWM lane.
 * <p>
 * Credits to
 * <a href="https://github.com/p2reneker25/2035-YAGSL/blob/main/swervelib/encoders/PWMDutyCycleEncoderSwerve.java">
 * p2reneker25</a> for building this.
 */
public class PWMDutyCycleEncoderSwerve extends SwerveAbsoluteEncoder
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
   * Constructor for the PWM duty cycle encoder.
   *
   * @param pin PWM lane for the encoder.
   */
  public PWMDutyCycleEncoderSwerve(int pin)
  {
    encoder = new DutyCycleEncoder(pin);
    inaccurateVelocities = new Alert(
        "Encoders",
        "The PWM Duty Cycle encoder may not report accurate velocities!",
        Alert.AlertType.WARNING_TRACE);

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
    return (isInverted ? -1.0 : 1.0) * encoder.getAbsolutePosition() * 360;
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

  /**
   * Sets the offset of the Encoder in the WPILib Encoder Library.
   *
   * @param offset the offset the Absolute Encoder uses as the zero point.
   * @return Always true due to no external device commands.
   */
  @Override
  public boolean setAbsoluteEncoderOffset(double offset)
  {
    encoder.setPositionOffset(offset);

    return true;
  }
}
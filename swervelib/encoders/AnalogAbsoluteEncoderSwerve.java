package swervelib.encoders;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import swervelib.telemetry.Alert;

/**
 * Swerve Absolute Encoder for Thrifty Encoders and other analog encoders.
 */
public class AnalogAbsoluteEncoderSwerve extends SwerveAbsoluteEncoder
{
  // Entire class inspired by 5010
  // Source: https://github.com/FRC5010/FRCLibrary/blob/main/FRC5010Example2023/src/main/java/frc/robot/FRC5010/sensors/AnalogInput5010.java
  /**
   * Encoder as Analog Input.
   */
  public  AnalogInput encoder;
  /**
   * Inversion state of the encoder.
   */
  private boolean     inverted = false;
  /**
   * An {@link Alert} for if the absolute encoder offset cannot be set.
   */
  private Alert       cannotSetOffset;
  /**
   * An {@link Alert} detailing how the analog absolute encoder may not report accurate velocities.
   */
  private Alert       inaccurateVelocities;

  /**
   * Construct the Thrifty Encoder as a Swerve Absolute Encoder.
   *
   * @param encoder Encoder to construct.
   */
  public AnalogAbsoluteEncoderSwerve(AnalogInput encoder)
  {
    this.encoder = encoder;
    cannotSetOffset = new Alert(
        "Encoders",
        "Cannot Set Absolute Encoder Offset of Analog Encoders Channel #" + encoder.getChannel(),
        Alert.AlertType.WARNING);
    inaccurateVelocities = new Alert(
        "Encoders",
        "The Analog Absolute encoder may not report accurate velocities!",
        Alert.AlertType.WARNING_TRACE);
  }

  /**
   * Construct the Encoder given the analog input channel.
   *
   * @param channel Analog Input channel of which the encoder resides.
   */
  public AnalogAbsoluteEncoderSwerve(int channel)
  {
    this(new AnalogInput(channel));
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
   * Configure the absolute encoder to read from [0, 360) per second.
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
    return (inverted ? -1.0 : 1.0) * (encoder.getAverageVoltage() / RobotController.getVoltage5V()) * 360;
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
   * Cannot Set the offset of an Analog Absolute Encoder.
   *
   * @param offset the offset the Absolute Encoder uses as the zero point.
   * @return Will always be false as setting the offset is unsupported of an Analog absolute encoder.
   */
  @Override
  public boolean setAbsoluteEncoderOffset(double offset)
  {
    //Do Nothing
    cannotSetOffset.set(true);
    return false;
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
    return encoder.getValue();
  }
}

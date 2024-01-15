package swervelib.encoders;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkAnalogSensor.Mode;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;
import swervelib.motors.SwerveMotor;

/**
 * SparkMax absolute encoder, attached through the data port analog pin.
 */
public class SparkMaxAnalogEncoderSwerve extends SwerveAbsoluteEncoder
{

  /**
   * The {@link SparkAnalogSensor} representing the duty cycle encoder attached to the SparkMax analog port.
   */
  public SparkAnalogSensor encoder;

  /**
   * Create the {@link SparkMaxAnalogEncoderSwerve} object as a analog sensor from the {@link CANSparkMax} motor data
   * port analog pin.
   *
   * @param motor Motor to create the encoder from.
   */
  public SparkMaxAnalogEncoderSwerve(SwerveMotor motor)
  {
    if (motor.getMotor() instanceof CANSparkMax)
    {
      encoder = ((CANSparkMax) motor.getMotor()).getAnalog(Mode.kAbsolute);
    } else
    {
      throw new RuntimeException("Motor given to instantiate SparkMaxEncoder is not a CANSparkMax");
    }
  }

  /**
   * Run the configuration until it succeeds or times out.
   *
   * @param config Lambda supplier returning the error state.
   */
  private void configureSparkMax(Supplier<REVLibError> config)
  {
    for (int i = 0; i < maximumRetries; i++)
    {
      if (config.get() == REVLibError.kOk)
      {
        return;
      }
    }
    DriverStation.reportWarning("Failure configuring encoder", true);
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
    encoder.setInverted(inverted);
  }

  /**
   * Get the absolute position of the encoder.
   *
   * @return Absolute position in degrees from [0, 360).
   */
  @Override
  public double getAbsolutePosition()
  {
    return encoder.getPosition();
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
   * Sets the Absolute Encoder offset at the Encoder Level.
   *
   * @param offset the offset the Absolute Encoder uses as the zero point.
   * @return if setting Absolute Encoder Offset was successful or not.
   */
  @Override
  public boolean setAbsoluteEncoderOffset(double offset)
  {
    DriverStation.reportWarning("SparkMax Analog Sensor's do not support integrated offsets", true);
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
    return encoder.getVelocity();
  }
}

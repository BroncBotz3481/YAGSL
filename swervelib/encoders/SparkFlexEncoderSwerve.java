package swervelib.encoders;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import swervelib.motors.SparkFlexSwerve;
import swervelib.motors.SwerveMotor;

/**
 * SparkFlex absolute encoder, attached through the data port.
 */
public class SparkFlexEncoderSwerve extends SwerveAbsoluteEncoder
{

  /**
   * The {@link AbsoluteEncoder} representing the duty cycle encoder attached to the SparkFlex.
   */
  public  SparkAbsoluteEncoder encoder;
  /**
   * An {@link Alert} for if there is a failure configuring the encoder.
   */
  private Alert                failureConfiguring;
  /**
   * {@link SparkFlexSwerve} instance.
   */
  private SwerveMotor sparkFlex;

  /**
   * Create the {@link SparkFlexEncoderSwerve} object as a duty cycle from the {@link SparkFlex} motor.
   *
   * @param motor            Motor to create the encoder from.
   * @param conversionFactor The conversion factor to set if the output is not from 0 to 360.
   */
  public SparkFlexEncoderSwerve(SwerveMotor motor, int conversionFactor)
  {
    failureConfiguring = new Alert(
        "Encoders",
        "Failure configuring SparkFlex Absolute Encoder",
        AlertType.kWarning);
    if (motor.getMotor() instanceof SparkFlex)
    {
      sparkFlex = motor;
      encoder = ((SparkFlex) motor.getMotor()).getAbsoluteEncoder();
      setConversionFactor(conversionFactor);
    } else
    {
      throw new RuntimeException("Motor given to instantiate SparkFlexEncoder is not a CANSparkFlex");
    }
  }

  @Override
  public void close()
  {
    // SPARK Flex encoder gets closed with the motor
    // I don't think an encoder getting closed should 
    // close the entire motor so i will keep this empty
    // sparkFlex.close();
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
    if (sparkFlex instanceof SparkFlexSwerve)
    {
      SparkFlexConfig cfg = ((SparkFlexSwerve) sparkFlex).getConfig();
      cfg.absoluteEncoder.inverted(inverted);
      ((SparkFlexSwerve) sparkFlex).updateConfig(cfg);
    }
  }

  /**
   * Set the conversion factor of the {@link SparkFlexEncoderSwerve}.
   *
   * @param conversionFactor Position conversion factor from ticks to unit.
   */
  public void setConversionFactor(double conversionFactor)
  {
    SparkFlexConfig cfg = ((SparkFlexSwerve) sparkFlex).getConfig();
    cfg.signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(20);
    cfg.absoluteEncoder
        .positionConversionFactor(conversionFactor)
        .velocityConversionFactor(conversionFactor / 60);
    ((SparkFlexSwerve) sparkFlex).updateConfig(cfg);
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
   * Sets the Absolute Encoder Offset inside of the SparkFlex's Memory.
   *
   * @param offset the offset the Absolute Encoder uses as the zero point.
   * @return if setting Absolute Encoder Offset was successful or not.
   */
  @Override
  public boolean setAbsoluteEncoderOffset(double offset)
  {
    if (sparkFlex instanceof SparkFlexSwerve)
    {
      SparkFlexConfig cfg = ((SparkFlexSwerve) sparkFlex).getConfig();
      cfg.absoluteEncoder.zeroOffset(offset);
      ((SparkFlexSwerve) sparkFlex).updateConfig(cfg);
      return true;
    }
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

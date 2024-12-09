package swervelib.encoders;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.function.Supplier;
import swervelib.motors.SparkMaxBrushedMotorSwerve;
import swervelib.motors.SparkMaxSwerve;
import swervelib.motors.SwerveMotor;

/**
 * SparkMax absolute encoder, attached through the data port.
 */
public class SparkMaxEncoderSwerve extends SwerveAbsoluteEncoder
{

  /**
   * The {@link AbsoluteEncoder} representing the duty cycle encoder attached to the SparkMax.
   */
  public  SparkAbsoluteEncoder encoder;
  /**
   * An {@link Alert} for if there is a failure configuring the encoder.
   */
  private Alert                failureConfiguring;
  /**
   * An {@link Alert} for if there is a failure configuring the encoder offset.
   */
  private Alert                offsetFailure;
  /**
   * {@link SparkMaxBrushedMotorSwerve} or {@link SparkMaxSwerve} instance.
   */
  private SwerveMotor          sparkMax;

  /**
   * Create the {@link SparkMaxEncoderSwerve} object as a duty cycle from the {@link com.revrobotics.spark.SparkMax}
   * motor.
   *
   * @param motor            Motor to create the encoder from.
   * @param conversionFactor The conversion factor to set if the output is not from 0 to 360.
   */
  public SparkMaxEncoderSwerve(SwerveMotor motor, int conversionFactor)
  {
    failureConfiguring = new Alert(
        "Encoders",
        "Failure configuring SparkMax Analog Encoder",
        AlertType.kWarning);
    offsetFailure = new Alert(
        "Encoders",
        "Failure to set Absolute Encoder Offset",
        AlertType.kWarning);
    if (motor.getMotor() instanceof SparkMax)
    {
      sparkMax = motor;
      encoder = ((SparkMax) motor.getMotor()).getAbsoluteEncoder();
      motor.setAbsoluteEncoder(this);
      motor.configureIntegratedEncoder(conversionFactor);
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
    failureConfiguring.set(true);
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
    if (sparkMax instanceof SparkMaxSwerve)
    {
      SparkMaxConfig cfg = ((SparkMaxSwerve) sparkMax).getConfig();
      cfg.analogSensor.inverted(true);
      ((SparkMaxSwerve) sparkMax).updateConfig(cfg);
    } else if (sparkMax instanceof SparkMaxBrushedMotorSwerve)
    {
      SparkMaxConfig cfg = ((SparkMaxBrushedMotorSwerve) sparkMax).getConfig();
      cfg.analogSensor.inverted(true);
      ((SparkMaxBrushedMotorSwerve) sparkMax).updateConfig(cfg);
    }
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
   * Sets the Absolute Encoder Offset inside of the SparkMax's Memory.
   *
   * @param offset the offset the Absolute Encoder uses as the zero point.
   * @return if setting Absolute Encoder Offset was successful or not.
   */
  @Override
  public boolean setAbsoluteEncoderOffset(double offset)
  {
    if (sparkMax instanceof SparkMaxSwerve)
    {
      SparkMaxConfig cfg = ((SparkMaxSwerve) sparkMax).getConfig();
      cfg.absoluteEncoder.zeroOffset(offset);
      ((SparkMaxSwerve) sparkMax).updateConfig(cfg);
      return true;
    } else if (sparkMax instanceof SparkMaxBrushedMotorSwerve)
    {
      SparkMaxConfig cfg = ((SparkMaxBrushedMotorSwerve) sparkMax).getConfig();
      cfg.absoluteEncoder.zeroOffset(offset);
      ((SparkMaxBrushedMotorSwerve) sparkMax).updateConfig(cfg);
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

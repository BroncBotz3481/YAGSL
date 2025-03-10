package swervelib.encoders;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
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
        "Failure configuring SparkMax Absolute Encoder",
        AlertType.kWarning);
    offsetFailure = new Alert(
        "Encoders",
        "Failure to set Absolute Encoder Offset",
        AlertType.kWarning);
    if (motor.getMotor() instanceof SparkMax)
    {
      sparkMax = motor;
      encoder = ((SparkMax) motor.getMotor()).getAbsoluteEncoder();
      setConversionFactor(conversionFactor);
    } else
    {
      throw new RuntimeException("Motor given to instantiate SparkMaxEncoder is not a CANSparkMax");
    }
  }

  @Override
  public void close()
  {
    // SPARK MAX encoder gets closed with the motor
    // I don't think an encoder getting closed should 
    // close the entire motor so i will keep this empty
    // sparkFlex.close();
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
      cfg.absoluteEncoder.inverted(inverted);
      ((SparkMaxSwerve) sparkMax).updateConfig(cfg);
    } else if (sparkMax instanceof SparkMaxBrushedMotorSwerve)
    {
      SparkMaxConfig cfg = ((SparkMaxBrushedMotorSwerve) sparkMax).getConfig();
      cfg.absoluteEncoder.inverted(inverted);
      ((SparkMaxBrushedMotorSwerve) sparkMax).updateConfig(cfg);
    }
  }


  /**
   * Set the conversion factor of the {@link SparkMaxEncoderSwerve}.
   *
   * @param conversionFactor Position conversion factor from ticks to unit.
   */
  public void setConversionFactor(double conversionFactor)
  {
    // By default the SparkMax relays the info from the duty cycle encoder to the roborio every 200ms on CAN frame 5
    // This needs to be set to 20ms or under to properly update the swerve module position for odometry
    // Configuration taken from 3005, the team who helped develop the Max Swerve:
    // https://github.com/FRC3005/Charged-Up-2023-Public/blob/2b6a7c695e23edebafa27a76cf639a00f6e8a3a6/src/main/java/frc/robot/subsystems/drive/REVSwerveModule.java#L227-L244
    // Some of the frames can probably be adjusted to decrease CAN utilization, with 65535 being the max.
    // From testing, 20ms on frame 5 sometimes returns the same value while constantly powering the azimuth but 8ms may be overkill,
    // with limited testing 19ms did not return the same value while the module was constatntly rotating.

    SparkMaxConfig cfg = null;
    if (sparkMax instanceof SparkMaxSwerve)
    {
      cfg = ((SparkMaxSwerve) sparkMax).getConfig();

    } else if (sparkMax instanceof SparkMaxBrushedMotorSwerve)
    {
      cfg = ((SparkMaxBrushedMotorSwerve) sparkMax).getConfig();
    }
    if (cfg != null)
    {
      cfg.signals
          .absoluteEncoderPositionAlwaysOn(true)
          .absoluteEncoderPositionPeriodMs(20);

      cfg.absoluteEncoder
          .positionConversionFactor(conversionFactor)
          .velocityConversionFactor(conversionFactor / 60);
    }
    if (sparkMax instanceof SparkMaxSwerve)
    {
      ((SparkMaxSwerve) sparkMax).updateConfig(cfg);
    } else if (sparkMax instanceof SparkMaxBrushedMotorSwerve)
    {
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

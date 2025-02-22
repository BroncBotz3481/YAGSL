package swervelib.encoders;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Swerve Absolute Encoder for CTRE CANCoders.
 */
public class CANCoderSwerve extends SwerveAbsoluteEncoder
{

  /**
   * Wait time for status frames to show up.
   */
  public static double STATUS_TIMEOUT_SECONDS = Milliseconds.of(1).in(Seconds);
  /**
   * An {@link Alert} for if the CANCoder magnet field is less than ideal.
   */
  private final Alert                           magnetFieldLessThanIdeal;
  /**
   * An {@link Alert} for if the CANCoder reading is faulty.
   */
  private final Alert                           readingFaulty;
  /**
   * An {@link Alert} for if the CANCoder reading is faulty and the reading is ignored.
   */
  private final Alert                           readingIgnored;
  /**
   * An {@link Alert} for if the absolute encoder offset cannot be set.
   */
  private final Alert                           cannotSetOffset;
  /**
   * Magnet Health status signal for the CANCoder.
   */
  private final StatusSignal<MagnetHealthValue> magnetHealth;
  /**
   * CANCoder reading cache.
   */
  private final StatusSignal<Angle>             angle;
  /**
   * Angular velocity of the {@link CANcoder}.
   */
  private final StatusSignal<AngularVelocity>   velocity;
  /**
   * CANCoder with WPILib sendable and support.
   */
  public        CANcoder                        encoder;
  /**
   * {@link CANcoder} Configurator objet for this class.
   */
  private       CANcoderConfigurator            config;
  /**
   * {@link CANcoderConfiguration} object for the CANcoder.
   */
  private       CANcoderConfiguration           cfg                    = new CANcoderConfiguration();

  /**
   * Initialize the CANCoder on the standard CANBus.
   *
   * @param id CAN ID.
   */
  public CANCoderSwerve(int id)
  {
    // Empty string uses the default canbus for the system
    this(id, "");
  }

  /**
   * Initialize the CANCoder on the CANivore.
   *
   * @param id     CAN ID of the {@link CANcoder}.
   * @param canbus CAN bus to initialize it on. Should be "rio" or "" if the RIO CANbus, else is the CANivore name.
   */
  public CANCoderSwerve(int id, String canbus)
  {
    encoder = new CANcoder(id, canbus);
    config = encoder.getConfigurator();
    magnetHealth = encoder.getMagnetHealth();
    angle = encoder.getAbsolutePosition();
    velocity = encoder.getVelocity();
    magnetFieldLessThanIdeal = new Alert(
        "Encoders",
        "CANCoder " + encoder.getDeviceID() + " magnetic field is less than ideal.",
        AlertType.kWarning);
    readingFaulty = new Alert(
        "Encoders",
        "CANCoder " + encoder.getDeviceID() + " reading was faulty.",
        AlertType.kWarning);
    readingIgnored = new Alert(
        "Encoders",
        "CANCoder " + encoder.getDeviceID() + " reading was faulty, ignoring.",
        AlertType.kWarning);
    cannotSetOffset = new Alert(
        "Encoders",
        "Failure to set CANCoder "
        + encoder.getDeviceID()
        + " Absolute Encoder Offset",
        AlertType.kWarning);
  }

  @Override
  public void close()
  {
    encoder.close();
  }

  /**
   * Reset the encoder to factory defaults.
   */
  @Override
  public void factoryDefault()
  {
    cfg = new CANcoderConfiguration();
    config.apply(cfg);
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
   * Configure the absolute encoder to read from [0, 360) per second.
   *
   * @param inverted Whether the encoder is inverted.
   */
  @Override
  public void configure(boolean inverted)
  {
    config.refresh(cfg.MagnetSensor);
    config.apply(cfg.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(1))
                                 .withSensorDirection(inverted ? SensorDirectionValue.Clockwise_Positive
                                                               : SensorDirectionValue.CounterClockwise_Positive));
  }


  /**
   * Get the absolute position of the encoder. Sets {@link SwerveAbsoluteEncoder#readingError} on erroneous readings.
   *
   * @return Absolute position in degrees from [0, 360).
   */
  @Override
  public double getAbsolutePosition()
  {
    readingError = false;
    MagnetHealthValue strength = magnetHealth.refresh().getValue();
    angle.refresh();

    magnetFieldLessThanIdeal.set(strength != MagnetHealthValue.Magnet_Green);
    if (strength == MagnetHealthValue.Magnet_Invalid || strength == MagnetHealthValue.Magnet_Red)
    {
      readingError = true;
      readingFaulty.set(true);
      return 0;
    } else
    {
      readingFaulty.set(false);
    }

    // Taken from democat's library.
    // Source: https://github.com/democat3457/swerve-lib/blob/7c03126b8c22f23a501b2c2742f9d173a5bcbc40/src/main/java/com/swervedrivespecialties/swervelib/ctre/CanCoderFactoryBuilder.java#L51-L74
    for (int i = 0; i < maximumRetries; i++)
    {
      if (angle.getStatus() == StatusCode.OK)
      {
        break;
      }
      angle.waitForUpdate(STATUS_TIMEOUT_SECONDS);
    }
    if (angle.getStatus() != StatusCode.OK)
    {
      readingError = true;
      readingIgnored.set(true);
    } else
    {
      readingIgnored.set(false);
    }
    // Convert from Rotations to Degrees.
    return angle.getValueAsDouble() * 360;
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
   * Sets the Absolute Encoder Offset within the CANcoder's Memory.
   *
   * @param offset the offset the Absolute Encoder uses as the zero point in degrees.
   * @return if setting Absolute Encoder Offset was successful or not.
   */
  @Override
  public boolean setAbsoluteEncoderOffset(double offset)
  {
    StatusCode error = config.refresh(cfg.MagnetSensor);
    if (error != StatusCode.OK)
    {
      return false;
    }

    error = config.apply(cfg.MagnetSensor.withMagnetOffset(offset / 360));
    cannotSetOffset.setText(
        "Failure to set CANCoder "
        + encoder.getDeviceID()
        + " Absolute Encoder Offset Error: "
        + error);
    if (error == StatusCode.OK)
    {
      cannotSetOffset.set(false);
      return true;
    }
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
    return velocity.refresh().getValue().in(DegreesPerSecond);
  }
}

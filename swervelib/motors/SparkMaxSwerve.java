package swervelib.motors;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import swervelib.encoders.SparkMaxAnalogEncoderSwerve;
import swervelib.encoders.SparkMaxEncoderSwerve;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.parser.PIDFConfig;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * An implementation of {@link com.revrobotics.spark.SparkMax} as a {@link SwerveMotor}.
 */
public class SparkMaxSwerve extends SwerveMotor
{

  /**
   * {@link SparkMax} Instance.
   */
  private final SparkMax                  motor;
  /**
   * Integrated encoder.
   */
  public        RelativeEncoder           encoder;
  /**
   * Absolute encoder attached to the SparkMax (if exists)
   */
  public        SwerveAbsoluteEncoder     absoluteEncoder;
  /**
   * Closed-loop PID controller.
   */
  public        SparkClosedLoopController pid;
  /**
   * Factory default already occurred.
   */
  private       boolean                   factoryDefaultOccurred = false;
  /**
   * Supplier for the velocity of the motor controller.
   */
  private       Supplier<Double>          velocity;
  /**
   * Supplier for the position of the motor controller.
   */
  private       Supplier<Double>          position;
  /**
   * Configuration object for {@link SparkMax} motor.
   */
  private       SparkMaxConfig            cfg                    = new SparkMaxConfig();
  /**
   * Tracker for changes that need to be pushed.
   */
  private       boolean                   cfgUpdated             = false;
  /**
   * After the first post-module config update there will be an error thrown to alert to a possible issue.
   */
  private boolean startupInitialized = false;


  /**
   * Initialize the swerve motor.
   *
   * @param motor        The SwerveMotor as a SparkMax object.
   * @param isDriveMotor Is the motor being initialized a drive motor?
   * @param motorType    Motor type controlled by the {@link SparkMax} motor controller.
   */
  public SparkMaxSwerve(SparkMax motor, boolean isDriveMotor, DCMotor motorType)
  {
    this.motor = motor;
    this.isDriveMotor = isDriveMotor;
    this.simMotor = motorType;
    factoryDefaults();
    clearStickyFaults();

    encoder = motor.getEncoder();
    pid = motor.getClosedLoopController();

    cfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder); // Configure feedback of the PID controller as the integrated encoder.
    cfgUpdated = true;
    velocity = encoder::getVelocity;
    position = encoder::getPosition;

    // Spin off configurations in a different thread.
    // configureSparkMax(() -> motor.setCANTimeout(0)); // Commented out because it prevents feedback.
  }


  /**
   * Initialize the {@link SwerveMotor} as a {@link SparkMax} connected to a Brushless Motor.
   *
   * @param id           CAN ID of the SparkMax.
   * @param isDriveMotor Is the motor being initialized a drive motor?
   * @param motorType    Motor type controlled by the {@link SparkMax} motor controller.
   */
  public SparkMaxSwerve(int id, boolean isDriveMotor, DCMotor motorType)
  {
    this(new SparkMax(id, MotorType.kBrushless), isDriveMotor, motorType);
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
      Timer.delay(Units.Milliseconds.of(5).in(Seconds));
    }
    DriverStation.reportWarning("Failure configuring motor " + motor.getDeviceId(), true);
  }

  /**
   * Get the current configuration of the {@link SparkMax}
   *
   * @return {@link SparkMaxConfig}
   */
  public SparkMaxConfig getConfig()
  {
    return cfg;
  }

  /**
   * Update the config for the {@link SparkMax}
   *
   * @param cfgGiven Given {@link SparkMaxConfig} which should have minimal modifications.
   */
  public void updateConfig(SparkMaxConfig cfgGiven)
  {
    cfg.apply(cfgGiven);
    configureSparkMax(() -> motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
    cfgUpdated = false;
  }

  /**
   * Set the voltage compensation for the swerve module motor.
   *
   * @param nominalVoltage Nominal voltage for operation to output to.
   */
  @Override
  public void setVoltageCompensation(double nominalVoltage)
  {
    cfg.voltageCompensation(nominalVoltage);
    cfgUpdated = true;
  }

  /**
   * Set the current limit for the swerve drive motor, remember this may cause jumping if used in conjunction with
   * voltage compensation. This is useful to protect the motor from current spikes.
   *
   * @param currentLimit Current limit in AMPS at free speed.
   */
  @Override
  public void setCurrentLimit(int currentLimit)
  {
    cfg.smartCurrentLimit(currentLimit);
    cfgUpdated = true;

  }

  /**
   * Set the maximum rate the open/closed loop output can change by.
   *
   * @param rampRate Time in seconds to go from 0 to full throttle.
   */
  @Override
  public void setLoopRampRate(double rampRate)
  {
    cfg.closedLoopRampRate(rampRate)
       .openLoopRampRate(rampRate);
    cfgUpdated = true;

  }

  /**
   * Get the motor object from the module.
   *
   * @return Motor object.
   */
  @Override
  public Object getMotor()
  {
    return motor;
  }

  /**
   * Get the {@link DCMotor} of the motor class.
   *
   * @return {@link DCMotor} of this type.
   */
  @Override
  public DCMotor getSimMotor()
  {
    if (simMotor == null)
    {
      simMotor = DCMotor.getNEO(1);
    }
    return simMotor;
  }

  /**
   * Queries whether the absolute encoder is directly attached to the motor controller.
   *
   * @return connected absolute encoder state.
   */
  @Override
  public boolean isAttachedAbsoluteEncoder()
  {
    return absoluteEncoder != null;
  }

  /**
   * Configure the factory defaults.
   */
  @Override
  public void factoryDefaults()
  {
    // Do nothing
  }

  /**
   * Clear the sticky faults on the motor controller.
   */
  @Override
  public void clearStickyFaults()
  {
    configureSparkMax(motor::clearFaults);
  }

  /**
   * Set the absolute encoder to be a compatible absolute encoder.
   *
   * @param encoder The encoder to use.
   * @return The {@link SwerveMotor} for easy instantiation.
   */
  @Override
  public SwerveMotor setAbsoluteEncoder(SwerveAbsoluteEncoder encoder)
  {
    if (encoder == null)
    {
      absoluteEncoder = null;
      cfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
      cfgUpdated = true;

      velocity = this.encoder::getVelocity;
      position = this.encoder::getPosition;
      burnFlash();
    } else if (encoder instanceof SparkMaxAnalogEncoderSwerve || encoder instanceof SparkMaxEncoderSwerve)
    {
      cfg.closedLoop.feedbackSensor(encoder instanceof SparkMaxAnalogEncoderSwerve
                                    ? FeedbackSensor.kAnalogSensor : FeedbackSensor.kAbsoluteEncoder);
      cfgUpdated = true;

      DriverStation.reportWarning(
          "IF possible configure the encoder offset in the REV Hardware Client instead of using the" +
          " absoluteEncoderOffset in the Swerve Module JSON!",
          false);
      absoluteEncoder = encoder;
      velocity = absoluteEncoder::getVelocity;
      position = absoluteEncoder::getAbsolutePosition;
    }
    return this;
  }

  /**
   * Configure the integrated encoder for the swerve module. Sets the conversion factors for position and velocity.
   *
   * @param positionConversionFactor The conversion factor to apply.
   */
  @Override
  public void configureIntegratedEncoder(double positionConversionFactor)
  {
    cfg.signals
        .absoluteEncoderPositionAlwaysOn(false)
        .absoluteEncoderVelocityAlwaysOn(false)
        .analogPositionAlwaysOn(false)
        .analogVelocityAlwaysOn(false)
        .analogVoltageAlwaysOn(false)
        .externalOrAltEncoderPositionAlwaysOn(false)
        .externalOrAltEncoderVelocityAlwaysOn(false)
        .primaryEncoderPositionAlwaysOn(false)
        .primaryEncoderVelocityAlwaysOn(false)
        .iAccumulationAlwaysOn(false)
        .appliedOutputPeriodMs(10)
        .faultsPeriodMs(20);

    if (absoluteEncoder == null)
    {
      cfg.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
      cfg.encoder
          .positionConversionFactor(positionConversionFactor)
          .velocityConversionFactor(positionConversionFactor / 60);
      // Changes the measurement period and number of samples used to calculate the velocity for the intergrated motor controller
      // Notability this changes the returned velocity and the velocity used for the onboard velocity PID loop (TODO: triple check the PID portion of this statement)
      // Default settings of 32ms and 8 taps introduce ~100ms of measurement lag
      // https://www.chiefdelphi.com/t/shooter-encoder/400211/11
      // This value was taken from:
      // https://github.com/Mechanical-Advantage/RobotCode2023/blob/9884d13b2220b76d430e82248fd837adbc4a10bc/src/main/java/org/littletonrobotics/frc2023/subsystems/drive/ModuleIOSparkMax.java#L132-L133
      // and tested on 9176 for YAGSL, notably 3005 uses 16ms instead of 10 but 10 is more common based on github searches
      cfg.encoder
          .quadratureMeasurementPeriod(10)
          .quadratureAverageDepth(2);

      // Taken from
      // https://github.com/frc3512/SwerveBot-2022/blob/9d31afd05df6c630d5acb4ec2cf5d734c9093bf8/src/main/java/frc/lib/util/SparkMaxUtil.java#L67
      // Unused frames can be set to 65535 to decrease CAN ultilization.
      cfg.signals
          .primaryEncoderVelocityAlwaysOn(isDriveMotor) // Disable velocity reporting for angle motors.
          .primaryEncoderPositionAlwaysOn(true)
          .primaryEncoderPositionPeriodMs(20);

    } else
    {
      // By default the SparkMax relays the info from the duty cycle encoder to the roborio every 200ms on CAN frame 5
      // This needs to be set to 20ms or under to properly update the swerve module position for odometry
      // Configuration taken from 3005, the team who helped develop the Max Swerve:
      // https://github.com/FRC3005/Charged-Up-2023-Public/blob/2b6a7c695e23edebafa27a76cf639a00f6e8a3a6/src/main/java/frc/robot/subsystems/drive/REVSwerveModule.java#L227-L244
      // Some of the frames can probably be adjusted to decrease CAN utilization, with 65535 being the max.
      // From testing, 20ms on frame 5 sometimes returns the same value while constantly powering the azimuth but 8ms may be overkill,
      // with limited testing 19ms did not return the same value while the module was constatntly rotating.
      if (absoluteEncoder.getAbsoluteEncoder() instanceof AbsoluteEncoder)
      {
        cfg.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        cfg.signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs(20);

        cfg.absoluteEncoder
            .positionConversionFactor(positionConversionFactor)
            .velocityConversionFactor(positionConversionFactor / 60);
      } else
      {
        cfg.closedLoop.feedbackSensor(FeedbackSensor.kAnalogSensor);

        cfg.signals
            .analogVoltageAlwaysOn(true)
            .analogPositionAlwaysOn(true)
            .analogVoltagePeriodMs(20)
            .analogPositionPeriodMs(20);

        cfg.analogSensor
            .positionConversionFactor(positionConversionFactor)
            .velocityConversionFactor(positionConversionFactor / 60);
      }
    }
    cfgUpdated = true;

  }

  /**
   * Configure the PIDF values for the closed loop controller.
   *
   * @param config Configuration class holding the PIDF values.
   */
  @Override
  public void configurePIDF(PIDFConfig config)
  {
    cfg.closedLoop.pidf(config.p, config.i, config.d, config.f)
                  .iZone(config.iz)
                  .outputRange(config.output.min, config.output.max);
    cfgUpdated = true;

  }

  /**
   * Configure the PID wrapping for the position closed loop controller.
   *
   * @param minInput Minimum PID input.
   * @param maxInput Maximum PID input.
   */
  @Override
  public void configurePIDWrapping(double minInput, double maxInput)
  {
    cfg.closedLoop
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(minInput, maxInput);
    cfgUpdated = true;

  }

  /**
   * Set the idle mode.
   *
   * @param isBrakeMode Set the brake mode.
   */
  @Override
  public void setMotorBrake(boolean isBrakeMode)
  {
    cfg.idleMode(isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    cfgUpdated = true;

  }

  /**
   * Set the motor to be inverted.
   *
   * @param inverted State of inversion.
   */
  @Override
  public void setInverted(boolean inverted)
  {
    cfg.inverted(inverted);
    cfgUpdated = true;
  }

  /**
   * Save the configurations from flash to EEPROM.
   */
  @Override
  public void burnFlash()
  {
    configureSparkMax(() -> {
      return motor.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    });
    cfgUpdated = false;
  }

  /**
   * Set the percentage output.
   *
   * @param percentOutput percent out for the motor controller.
   */
  @Override
  public void set(double percentOutput)
  {
    motor.set(percentOutput);
  }

  /**
   * Set the closed loop PID controller reference point.
   *
   * @param setpoint    Setpoint in MPS or Angle in degrees.
   * @param feedforward Feedforward in volt-meter-per-second or kV.
   */
  @Override
  public void setReference(double setpoint, double feedforward)
  {
    int pidSlot = 0;

    if (cfgUpdated)
    {
      burnFlash();
      Timer.delay(0.01); // Give 10ms to apply changes
      if (startupInitialized)
      {
        DriverStation.reportWarning("Applying changes mid-execution not recommended.", true);
      } else
      {
        startupInitialized = true;
      }
    }

    if (isDriveMotor)
    {
      configureSparkMax(() ->
                            pid.setReference(
                                setpoint,
                                ControlType.kVelocity,
                                pidSlot,
                                feedforward));
    } else
    {
      configureSparkMax(() ->
                            pid.setReference(
                                setpoint,
                                ControlType.kPosition,
                                pidSlot,
                                feedforward));
      if (SwerveDriveTelemetry.isSimulation)
      {
        encoder.setPosition(setpoint);
      }
    }
  }

  /**
   * Set the closed loop PID controller reference point.
   *
   * @param setpoint    Setpoint in meters per second or angle in degrees.
   * @param feedforward Feedforward in volt-meter-per-second or kV.
   * @param position    Only used on the angle motor, the position of the motor in degrees.
   */
  @Override
  public void setReference(double setpoint, double feedforward, double position)
  {
    setReference(setpoint, feedforward);
  }

  /**
   * Get the voltage output of the motor controller.
   *
   * @return Voltage output.
   */
  @Override
  public double getVoltage()
  {
    return motor.getAppliedOutput() * motor.getBusVoltage();
  }

  /**
   * Set the voltage of the motor.
   *
   * @param voltage Voltage to set.
   */
  @Override
  public void setVoltage(double voltage)
  {
    motor.setVoltage(voltage);
  }

  /**
   * Get the applied dutycycle output.
   *
   * @return Applied dutycycle output to the motor.
   */
  @Override
  public double getAppliedOutput()
  {
    return motor.getAppliedOutput();
  }

  /**
   * Get the velocity of the integrated encoder.
   *
   * @return velocity
   */
  @Override
  public double getVelocity()
  {
    return velocity.get();
  }

  /**
   * Get the position of the integrated encoder.
   *
   * @return Position
   */
  @Override
  public double getPosition()
  {
    return position.get();
  }

  /**
   * Set the integrated encoder position.
   *
   * @param position Integrated encoder position.
   */
  @Override
  public void setPosition(double position)
  {
    if (absoluteEncoder == null)
    {
      configureSparkMax(() -> encoder.setPosition(position));
    }
  }
}

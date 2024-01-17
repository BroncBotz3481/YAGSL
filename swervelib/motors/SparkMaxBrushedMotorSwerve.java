package swervelib.motors;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder.Type;
import java.util.function.Supplier;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.parser.PIDFConfig;
import swervelib.telemetry.Alert;

/**
 * Brushed motor control with SparkMax.
 */
public class SparkMaxBrushedMotorSwerve extends SwerveMotor
{

  /**
   * SparkMAX Instance.
   */
  public CANSparkMax motor;

  /**
   * Absolute encoder attached to the SparkMax (if exists)
   */
  public  AbsoluteEncoder    absoluteEncoder;
  /**
   * Integrated encoder.
   */
  public  RelativeEncoder    encoder;
  /**
   * Closed-loop PID controller.
   */
  public  SparkPIDController pid;
  /**
   * Factory default already occurred.
   */
  private boolean            factoryDefaultOccurred = false;
  /**
   * An {@link Alert} for if the motor has no encoder.
   */
  private Alert              noEncoderAlert;
  /**
   * An {@link Alert} for if there is an error configuring the motor.
   */
  private Alert              failureConfiguringAlert;
  /**
   * An {@link Alert} for if the motor has no encoder defined.
   */
  private Alert              noEncoderDefinedAlert;

  /**
   * Initialize the swerve motor.
   *
   * @param motor                  The SwerveMotor as a SparkMax object.
   * @param isDriveMotor           Is the motor being initialized a drive motor?
   * @param encoderType            {@link Type} of encoder to use for the {@link CANSparkMax} device.
   * @param countsPerRevolution    The number of encoder pulses for the {@link Type} encoder per revolution.
   * @param useDataPortQuadEncoder Use the encoder attached to the data port of the spark max for a quadrature encoder.
   */
  public SparkMaxBrushedMotorSwerve(CANSparkMax motor, boolean isDriveMotor, Type encoderType, int countsPerRevolution,
                                    boolean useDataPortQuadEncoder)
  {
    // Drive motors **MUST** have an encoder attached.
    if (isDriveMotor && encoderType == Type.kNoSensor)
    {
      noEncoderAlert.set(true);
      throw new RuntimeException("Cannot use SparkMAX as a drive motor without an encoder attached.");
    }

    // Hall encoders can be used as quadrature encoders.
    if (encoderType == Type.kHallSensor)
    {
      encoderType = Type.kQuadrature;
    }

    this.motor = motor;
    this.isDriveMotor = isDriveMotor;

    factoryDefaults();
    clearStickyFaults();

    // Get the onboard PID controller.
    pid = motor.getPIDController();

    // If there is a sensor attached to the data port or encoder port set the relative encoder.
    if (isDriveMotor || (encoderType != Type.kNoSensor || useDataPortQuadEncoder))
    {
      this.encoder = useDataPortQuadEncoder ?
                     motor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, countsPerRevolution) :
                     motor.getEncoder(encoderType, countsPerRevolution);

      // Configure feedback of the PID controller as the integrated encoder.
      configureSparkMax(() -> pid.setFeedbackDevice(encoder));
    }
    // Spin off configurations in a different thread.
    // configureSparkMax(() -> motor.setCANTimeout(0)); // Commented it out because it prevents feedback.

    noEncoderAlert = new Alert("Motors",
                               "Cannot use motor without encoder.",
                               Alert.AlertType.ERROR_TRACE);
    failureConfiguringAlert = new Alert("Motors",
                                        "Failure configuring motor " + motor.getDeviceId(),
                                        Alert.AlertType.WARNING_TRACE);
    noEncoderDefinedAlert = new Alert("Motors",
                                      "An encoder MUST be defined to work with a SparkMAX",
                                      Alert.AlertType.ERROR_TRACE);
  }

  /**
   * Initialize the {@link SwerveMotor} as a {@link CANSparkMax} connected to a Brushless Motor.
   *
   * @param id                     CAN ID of the SparkMax.
   * @param isDriveMotor           Is the motor being initialized a drive motor?
   * @param encoderType            {@link Type} of encoder to use for the {@link CANSparkMax} device.
   * @param countsPerRevolution    The number of encoder pulses for the {@link Type} encoder per revolution.
   * @param useDataPortQuadEncoder Use the encoder attached to the data port of the spark max for a quadrature encoder.
   */
  public SparkMaxBrushedMotorSwerve(int id, boolean isDriveMotor, Type encoderType, int countsPerRevolution,
                                    boolean useDataPortQuadEncoder)
  {
    this(new CANSparkMax(id, MotorType.kBrushed), isDriveMotor, encoderType, countsPerRevolution,
         useDataPortQuadEncoder);
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
    failureConfiguringAlert.set(true);
  }

  /**
   * Set the voltage compensation for the swerve module motor.
   *
   * @param nominalVoltage Nominal voltage for operation to output to.
   */
  @Override
  public void setVoltageCompensation(double nominalVoltage)
  {
    configureSparkMax(() -> motor.enableVoltageCompensation(nominalVoltage));
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
    configureSparkMax(() -> motor.setSmartCurrentLimit(currentLimit));
  }

  /**
   * Set the maximum rate the open/closed loop output can change by.
   *
   * @param rampRate Time in seconds to go from 0 to full throttle.
   */
  @Override
  public void setLoopRampRate(double rampRate)
  {
    configureSparkMax(() -> motor.setOpenLoopRampRate(rampRate));
    configureSparkMax(() -> motor.setClosedLoopRampRate(rampRate));
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
    if (!factoryDefaultOccurred)
    {
      configureSparkMax(motor::restoreFactoryDefaults);
      factoryDefaultOccurred = true;
    }
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
    if (encoder.getAbsoluteEncoder() instanceof AbsoluteEncoder)
    {
      absoluteEncoder = (AbsoluteEncoder) encoder.getAbsoluteEncoder();
      configureSparkMax(() -> pid.setFeedbackDevice(absoluteEncoder));
    }
    if (absoluteEncoder == null && this.encoder == null)
    {
      noEncoderDefinedAlert.set(true);
      throw new RuntimeException("An encoder MUST be defined to work with a SparkMAX");
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
    if (absoluteEncoder == null)
    {
      configureSparkMax(() -> encoder.setPositionConversionFactor(positionConversionFactor));
      configureSparkMax(() -> encoder.setVelocityConversionFactor(positionConversionFactor / 60));

      // Taken from
      // https://github.com/frc3512/SwerveBot-2022/blob/9d31afd05df6c630d5acb4ec2cf5d734c9093bf8/src/main/java/frc/lib/util/CANSparkMaxUtil.java#L67
      configureCANStatusFrames(10, 20, 20, 500, 500);
    } else
    {
      configureSparkMax(() -> absoluteEncoder.setPositionConversionFactor(positionConversionFactor));
      configureSparkMax(() -> absoluteEncoder.setVelocityConversionFactor(positionConversionFactor / 60));
    }
  }

  /**
   * Configure the PIDF values for the closed loop controller.
   *
   * @param config Configuration class holding the PIDF values.
   */
  @Override
  public void configurePIDF(PIDFConfig config)
  {
//    int pidSlot =
//        isDriveMotor ? SparkMAX_slotIdx.Velocity.ordinal() : SparkMAX_slotIdx.Position.ordinal();
    int pidSlot = 0;

    configureSparkMax(() -> pid.setP(config.p, pidSlot));
    configureSparkMax(() -> pid.setI(config.i, pidSlot));
    configureSparkMax(() -> pid.setD(config.d, pidSlot));
    configureSparkMax(() -> pid.setFF(config.f, pidSlot));
    configureSparkMax(() -> pid.setIZone(config.iz, pidSlot));
    configureSparkMax(() -> pid.setOutputRange(config.output.min, config.output.max, pidSlot));
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
    configureSparkMax(() -> pid.setPositionPIDWrappingEnabled(true));
    configureSparkMax(() -> pid.setPositionPIDWrappingMinInput(minInput));
    configureSparkMax(() -> pid.setPositionPIDWrappingMaxInput(maxInput));
  }

  /**
   * Set the CAN status frames.
   *
   * @param CANStatus0 Applied Output, Faults, Sticky Faults, Is Follower
   * @param CANStatus1 Motor Velocity, Motor Temperature, Motor Voltage, Motor Current
   * @param CANStatus2 Motor Position
   * @param CANStatus3 Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor Position
   * @param CANStatus4 Alternate Encoder Velocity, Alternate Encoder Position
   */
  public void configureCANStatusFrames(
      int CANStatus0, int CANStatus1, int CANStatus2, int CANStatus3, int CANStatus4)
  {
    configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, CANStatus0));
    configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, CANStatus1));
    configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, CANStatus2));
    configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, CANStatus3));
    configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, CANStatus4));
    // TODO: Configure Status Frame 5 and 6 if necessary
    //  https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
  }

  /**
   * Set the idle mode.
   *
   * @param isBrakeMode Set the brake mode.
   */
  @Override
  public void setMotorBrake(boolean isBrakeMode)
  {
    configureSparkMax(() -> motor.setIdleMode(isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast));
  }

  /**
   * Set the motor to be inverted.
   *
   * @param inverted State of inversion.
   */
  @Override
  public void setInverted(boolean inverted)
  {
    motor.setInverted(inverted);
  }

  /**
   * Save the configurations from flash to EEPROM.
   */
  @Override
  public void burnFlash()
  {
    configureSparkMax(() -> motor.burnFlash());
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
//    int pidSlot =
//        isDriveMotor ? SparkMAX_slotIdx.Velocity.ordinal() : SparkMAX_slotIdx.Position.ordinal();
    int pidSlot = 0;
    configureSparkMax(() ->
                          pid.setReference(
                              setpoint,
                              isDriveMotor ? ControlType.kVelocity : ControlType.kPosition,
                              pidSlot,
                              feedforward)
                     );
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
   * Get the velocity of the integrated encoder.
   *
   * @return velocity
   */
  @Override
  public double getVelocity()
  {
    return absoluteEncoder == null ? encoder.getVelocity() : absoluteEncoder.getVelocity();
  }

  /**
   * Get the position of the integrated encoder.
   *
   * @return Position
   */
  @Override
  public double getPosition()
  {
    return absoluteEncoder == null ? encoder.getPosition() : absoluteEncoder.getPosition();
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

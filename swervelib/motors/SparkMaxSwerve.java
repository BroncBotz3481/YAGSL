package swervelib.motors;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.parser.PIDFConfig;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * An implementation of {@link CANSparkMax} as a {@link SwerveMotor}.
 */
public class SparkMaxSwerve extends SwerveMotor
{

  /**
   * SparkMAX Instance.
   */
  public  CANSparkMax           motor;
  /**
   * Integrated encoder.
   */
  public  RelativeEncoder       encoder;
  /**
   * Absolute encoder attached to the SparkMax (if exists)
   */
  public  SwerveAbsoluteEncoder absoluteEncoder;
  /**
   * Closed-loop PID controller.
   */
  public  SparkPIDController    pid;
  /**
   * Factory default already occurred.
   */
  private boolean               factoryDefaultOccurred = false;

  /**
   * Initialize the swerve motor.
   *
   * @param motor        The SwerveMotor as a SparkMax object.
   * @param isDriveMotor Is the motor being initialized a drive motor?
   */
  public SparkMaxSwerve(CANSparkMax motor, boolean isDriveMotor)
  {
    this.motor = motor;
    this.isDriveMotor = isDriveMotor;
    factoryDefaults();
    clearStickyFaults();

    encoder = motor.getEncoder();
    pid = motor.getPIDController();
    pid.setFeedbackDevice(
        encoder); // Configure feedback of the PID controller as the integrated encoder.

    // Spin off configurations in a different thread.
    // configureSparkMax(() -> motor.setCANTimeout(0)); // Commented out because it prevents feedback.
  }

  /**
   * Initialize the {@link SwerveMotor} as a {@link CANSparkMax} connected to a Brushless Motor.
   *
   * @param id           CAN ID of the SparkMax.
   * @param isDriveMotor Is the motor being initialized a drive motor?
   */
  public SparkMaxSwerve(int id, boolean isDriveMotor)
  {
    this(new CANSparkMax(id, MotorType.kBrushless), isDriveMotor);
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
    DriverStation.reportWarning("Failure configuring motor " + motor.getDeviceId(), true);
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
    if (encoder.getAbsoluteEncoder() instanceof MotorFeedbackSensor)
    {
      DriverStation.reportWarning(
          "IF possible configure the duty cycle encoder offset in the REV Hardware Client instead of using the" +
          " absoluteEncoderOffset in the Swerve Module JSON!",
          false);
      absoluteEncoder = encoder;
      configureSparkMax(() -> pid.setFeedbackDevice((MotorFeedbackSensor) absoluteEncoder.getAbsoluteEncoder()));
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
      configureCANStatusFrames(10, 20, 20, 500, 500, 200, 200);
    } else
    {
      configureSparkMax(() -> {
        if (absoluteEncoder.getAbsoluteEncoder() instanceof AbsoluteEncoder)
        {
          return ((AbsoluteEncoder) absoluteEncoder.getAbsoluteEncoder()).setPositionConversionFactor(
              positionConversionFactor);
        } else
        {
          return ((SparkAnalogSensor) absoluteEncoder.getAbsoluteEncoder()).setPositionConversionFactor(
              positionConversionFactor);
        }
      });
      configureSparkMax(() -> {
        if (absoluteEncoder.getAbsoluteEncoder() instanceof AbsoluteEncoder)
        {
          return ((AbsoluteEncoder) absoluteEncoder.getAbsoluteEncoder()).setVelocityConversionFactor(
              positionConversionFactor / 60);
        } else
        {
          return ((SparkAnalogSensor) absoluteEncoder.getAbsoluteEncoder()).setVelocityConversionFactor(
              positionConversionFactor / 60);
        }
      });
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
    configureSparkMax(() -> pid.setP(config.p));
    configureSparkMax(() -> pid.setI(config.i));
    configureSparkMax(() -> pid.setD(config.d));
    configureSparkMax(() -> pid.setFF(config.f));
    configureSparkMax(() -> pid.setIZone(config.iz));
    configureSparkMax(() -> pid.setOutputRange(config.output.min, config.output.max));
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
   * @param CANStatus5 Duty Cycle Absolute Encoder Position, Duty Cycle Absolute Encoder Absolute Angle
   * @param CANStatus6 Duty Cycle Absolute Encoder Velocity, Duty Cycle Absolute Encoder Frequency
   */
  public void configureCANStatusFrames(
      int CANStatus0, int CANStatus1, int CANStatus2, int CANStatus3, int CANStatus4, int CANStatus5, int CANStatus6)
  {
    configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, CANStatus0));
    configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, CANStatus1));
    configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, CANStatus2));
    configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, CANStatus3));
    configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, CANStatus4));
    configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, CANStatus5));
    configureSparkMax(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, CANStatus6));
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
    try
    {
      Thread.sleep(200);
    } catch (Exception e)
    {
    }
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
    boolean possibleBurnOutIssue = true;
//    int pidSlot =
//        isDriveMotor ? SparkMAX_slotIdx.Velocity.ordinal() : SparkMAX_slotIdx.Position.ordinal();
    int pidSlot = 0;

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
    return absoluteEncoder == null ? encoder.getPosition() : absoluteEncoder.getAbsolutePosition();
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

  /**
   * REV Slots for PID configuration.
   */
  enum SparkMAX_slotIdx
  {
    /**
     * Slot 1, used for position PID's.
     */
    Position,
    /**
     * Slot 2, used for velocity PID's.
     */
    Velocity,
    /**
     * Slot 3, used arbitrarily. (Documentation show simulations).
     */
    Simulation
  }
}

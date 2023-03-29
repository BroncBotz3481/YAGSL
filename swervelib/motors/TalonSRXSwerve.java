package swervelib.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.math.SwerveMath;
import swervelib.parser.PIDFConfig;
import swervelib.simulation.ctre.PhysicsSim;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * {@link com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX} Swerve Motor.
 */
public class TalonSRXSwerve extends SwerveMotor
{

  /**
   * Factory default already occurred.
   */
  private final boolean               factoryDefaultOccurred = false;
  /**
   * Current TalonFX configuration.
   */
  private final TalonSRXConfiguration configuration          = new TalonSRXConfiguration();
  /**
   * Whether the absolute encoder is integrated.
   */
  private final boolean               absoluteEncoder        = false;
  /**
   * TalonSRX motor controller.
   */
  WPI_TalonSRX motor;
  /**
   * The position conversion factor to convert raw sensor units to Meters Per 100ms, or Ticks to Degrees.
   */
  private double  positionConversionFactor = 1;
  /**
   * If the TalonFX configuration has changed.
   */
  private boolean configChanged            = true;
  /**
   * Nominal voltage default to use with feedforward.
   */
  private double  nominalVoltage           = 12.0;

  /**
   * Constructor for TalonSRX swerve motor.
   *
   * @param motor        Motor to use.
   * @param isDriveMotor Whether this motor is a drive motor.
   */
  public TalonSRXSwerve(WPI_TalonSRX motor, boolean isDriveMotor)
  {
    this.isDriveMotor = isDriveMotor;
    this.motor = motor;
    motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    factoryDefaults();
    clearStickyFaults();

    if (SwerveDriveTelemetry.isSimulation)
    {
      PhysicsSim.getInstance().addTalonSRX(motor, .25, 6800);
    }
  }

  /**
   * Construct the TalonSRX swerve motor given the ID.
   *
   * @param id           ID of the TalonSRX on the canbus.
   * @param isDriveMotor Whether the motor is a drive or steering motor.
   */
  public TalonSRXSwerve(int id, boolean isDriveMotor)
  {
    this(new WPI_TalonSRX(id), isDriveMotor);
  }

  /**
   * Configure the factory defaults.
   */
  @Override
  public void factoryDefaults()
  {
    if (!factoryDefaultOccurred)
    {
      motor.configFactoryDefault();
      motor.setSensorPhase(true);
    }
  }

  /**
   * Clear the sticky faults on the motor controller.
   */
  @Override
  public void clearStickyFaults()
  {
    motor.clearStickyFaults();
  }

  /**
   * Set the absolute encoder to be a compatible absolute encoder.
   *
   * @param encoder The encoder to use.
   */
  @Override
  public SwerveMotor setAbsoluteEncoder(SwerveAbsoluteEncoder encoder)
  {
    // Do not support.
    return this;
  }

  /**
   * Configure the integrated encoder for the swerve module. Sets the conversion factors for position and velocity.
   *
   * @param positionConversionFactor The conversion factor to apply for position.
   *                                 <p><br>
   *                                 Degrees: <br>
   *                                 <code>
   *                                 360 / (angleGearRatio * encoderTicksPerRotation)
   *                                 </code><br>
   *                                 <p><br>
   *                                 Meters:<br>
   *                                 <code>
   *                                 (Math.PI * wheelDiameter) / (driveGearRatio * encoderTicksPerRotation)
   *                                 </code>
   */
  @Override
  public void configureIntegratedEncoder(double positionConversionFactor)
  {
    this.positionConversionFactor = positionConversionFactor;
    // Taken from democat's library.
    // https://github.com/democat3457/swerve-lib/blob/7c03126b8c22f23a501b2c2742f9d173a5bcbc40/src/main/java/com/swervedrivespecialties/swervelib/ctre/Falcon500DriveControllerFactoryBuilder.java#L16
    configureCANStatusFrames(250);
  }

  /**
   * Set the CAN status frames.
   *
   * @param CANStatus1 Applied Motor Output, Fault Information, Limit Switch Information
   */
  public void configureCANStatusFrames(int CANStatus1)
  {
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, CANStatus1);
  }

  /**
   * Set the CAN status frames.
   *
   * @param CANStatus1       Applied Motor Output, Fault Information, Limit Switch Information
   * @param CANStatus2       Selected Sensor Position (PID 0), Selected Sensor Velocity (PID 0), Brushed Supply Current
   *                         Measurement, Sticky Fault Information
   * @param CANStatus3       Quadrature Information
   * @param CANStatus4       Analog Input, Supply Battery Voltage, Controller Temperature
   * @param CANStatus8       Pulse Width Information
   * @param CANStatus10      Motion Profiling/Motion Magic Information
   * @param CANStatus12      Selected Sensor Position (Aux PID 1), Selected Sensor Velocity (Aux PID 1)
   * @param CANStatus13      PID0 (Primary PID) Information
   * @param CANStatus14      PID1 (Auxiliary PID) Information
   * @param CANStatus21      Integrated Sensor Position (Talon FX), Integrated Sensor Velocity (Talon FX)
   * @param CANStatusCurrent Brushless Supply Current Measurement, Brushless Stator Current Measurement
   */
  public void configureCANStatusFrames(int CANStatus1, int CANStatus2, int CANStatus3, int CANStatus4, int CANStatus8,
                                       int CANStatus10, int CANStatus12, int CANStatus13, int CANStatus14,
                                       int CANStatus21, int CANStatusCurrent)
  {
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, CANStatus1);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, CANStatus2);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, CANStatus3);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, CANStatus4);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, CANStatus8);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, CANStatus10);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, CANStatus12);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, CANStatus13);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, CANStatus14);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, CANStatus21);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, CANStatusCurrent);

    // TODO: Configure Status Frame 2 thru 21 if necessary
    // https://v5.docs.ctr-electronics.com/en/stable/ch18_CommonAPI.html#setting-status-frame-periods
  }

  /**
   * Configure the PIDF values for the closed loop controller. 0 is disabled or off.
   *
   * @param config Configuration class holding the PIDF values.
   */
  @Override
  public void configurePIDF(PIDFConfig config)
  {
    configuration.slot0.kP = config.p;
    configuration.slot0.kI = config.i;
    configuration.slot0.kD = config.d;
    configuration.slot0.kF = config.f;
    configuration.slot0.integralZone = config.iz;
    configuration.slot0.closedLoopPeakOutput = config.output.max;
    configChanged = true;
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
    // Do nothing
  }

  /**
   * Set the idle mode.
   *
   * @param isBrakeMode Set the brake mode.
   */
  @Override
  public void setMotorBrake(boolean isBrakeMode)
  {
    motor.setNeutralMode(isBrakeMode ? NeutralMode.Brake : NeutralMode.Coast);
  }

  /**
   * Set the motor to be inverted.
   *
   * @param inverted State of inversion.
   */
  @Override
  public void setInverted(boolean inverted)
  {
    Timer.delay(1);
    motor.setInverted(inverted);
  }

  /**
   * Save the configurations from flash to EEPROM.
   */
  @Override
  public void burnFlash()
  {
    if (configChanged)
    {
      motor.configAllSettings(configuration, 250);
      configChanged = false;
    }
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
   * Convert the setpoint into native sensor units.
   *
   * @param setpoint Setpoint to mutate. In meters per second or degrees.
   * @param position Position in degrees, only used on angle motors.
   * @return Setpoint as native sensor units. Encoder ticks per 100ms, or Encoder tick.
   */
  public double convertToNativeSensorUnits(double setpoint, double position)
  {
    setpoint =
        isDriveMotor ? setpoint * .1 : SwerveMath.placeInAppropriate0To360Scope(position, setpoint);
    return setpoint / positionConversionFactor;
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
    setReference(setpoint, feedforward, getPosition());
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
    if (SwerveDriveTelemetry.isSimulation)
    {
      PhysicsSim.getInstance().run();
    }

    burnFlash();

    motor.set(
        isDriveMotor ? ControlMode.Velocity : ControlMode.Position,
        convertToNativeSensorUnits(setpoint, position),
        DemandType.ArbitraryFeedForward,
        feedforward / nominalVoltage);
  }

  /**
   * Get the velocity of the integrated encoder.
   *
   * @return velocity in Meters Per Second, or Degrees per Second.
   */
  @Override
  public double getVelocity()
  {
    return (motor.getSelectedSensorVelocity() * 10) * positionConversionFactor;
  }

  /**
   * Get the position of the integrated encoder.
   *
   * @return Position in Meters or Degrees.
   */
  @Override
  public double getPosition()
  {
    if (isDriveMotor)
    {
      return motor.getSelectedSensorPosition() * positionConversionFactor;
    } else
    {
      var pos = motor.getSelectedSensorPosition() * positionConversionFactor;
      pos %= 360;
      if (pos < 360)
      {
        pos += 360;
      }
      return pos;
    }
  }

  /**
   * Set the integrated encoder position.
   *
   * @param position Integrated encoder position. Should be angle in degrees or meters.
   */
  @Override
  public void setPosition(double position)
  {
    if (!absoluteEncoder && !SwerveDriveTelemetry.isSimulation)
    {
      motor.setSelectedSensorPosition(position / positionConversionFactor, 0, 250);
    }
  }

  /**
   * Set the voltage compensation for the swerve module motor.
   *
   * @param nominalVoltage Nominal voltage for operation to output to.
   */
  @Override
  public void setVoltageCompensation(double nominalVoltage)
  {
    configuration.voltageCompSaturation = nominalVoltage;
    configChanged = true;
    this.nominalVoltage = nominalVoltage;
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
    configuration.continuousCurrentLimit = currentLimit;
    configuration.peakCurrentLimit = currentLimit;
    configChanged = true;
  }

  /**
   * Set the maximum rate the open/closed loop output can change by.
   *
   * @param rampRate Time in seconds to go from 0 to full throttle.
   */
  @Override
  public void setLoopRampRate(double rampRate)
  {
    configuration.closedloopRamp = rampRate;
    configuration.openloopRamp = rampRate;
    configChanged = true;
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
    return absoluteEncoder;
  }
}

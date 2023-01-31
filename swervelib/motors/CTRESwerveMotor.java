package frc.robot.subsystems.swervedrive.swerve.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swervedrive.swerve.SwerveEncoder;
import frc.robot.subsystems.swervedrive.swerve.SwerveMotor;
import java.util.function.Supplier;

public class CTRESwerveMotor extends SwerveMotor
{

  private final SwerveEncoder m_angleEncoder;
  private final boolean       m_integratedAbsEncoder;
  private final double        m_allowableClosedLoopError;

  private final ControlMode      m_controlMode;
  private final int              m_mainPIDSlotId;
  private final int              m_mainPidId;
  private final Supplier<Double> m_encoderRet;
  private final WPI_TalonFX      m_motor;

  /**
   * kV feed forward for PID
   */
  private final double m_moduleRadkV;

  // TODO: Finish this based off of BaseFalconSwerve
  public CTRESwerveMotor(WPI_TalonFX motor, SwerveEncoder<?> encoder, ModuleMotorType type, double gearRatio,
                         double wheelDiameter,
                         double freeSpeedRPM, double powerLimit)
  {
    TalonFXConfiguration config = new TalonFXConfiguration();

    m_integratedAbsEncoder = encoder.m_encoder instanceof CANCoder;
    // Inspired by the following sources
    // https://github.com/SwerveDriveSpecialties/swerve-lib-2022-unmaintained/blob/55f3f1ad9e6bd81e56779d022a40917aacf8d3b3/src/main/java/com/swervedrivespecialties/swervelib/rev/NeoDriveControllerFactoryBuilder.java#L38
    // https://github.com/first95/FRC2022/blob/1f57d6837e04d8c8a89f4d83d71b5d2172f41a0e/SwervyBot/src/main/java/frc/robot/SwerveModule.java#L68
    // https://github.com/first95/FRC2022/blob/1f57d6837e04d8c8a89f4d83d71b5d2172f41a0e/SwervyBot/src/main/java/frc/robot/Constants.java#L89
    // https://github.com/AusTINCANsProgrammingTeam/2022Swerve/blob/main/2022Swerve/src/main/java/frc/robot/Constants.java
    motor.clearStickyFaults();
    motor.configFactoryDefault();

    m_angleEncoder = encoder;
    m_motorType = type;
    m_motor = motor;

    m_motor.setNeutralMode(NeutralMode.Brake);
    m_motor.setSensorPhase(true);

    m_motor.enableVoltageCompensation(true);

    m_mainPidId = CTRE_pidIdx.PRIMARY_PID.ordinal();
    if (type == ModuleMotorType.DRIVE)
    {
      m_moduleRadkV = 1;
      m_allowableClosedLoopError = Units.inchesToMeters(1) * 60;

      m_mainPIDSlotId = CTRE_slotIdx.Velocity.ordinal();
      m_controlMode = ControlMode.Velocity;
      m_encoderRet = m_motor::getSelectedSensorVelocity;

      setCurrentLimit(40);

      setConversionFactor(((Math.PI * wheelDiameter) / gearRatio) * 10); // Convert units to MPS.
    } else
    {
      m_moduleRadkV = (12 * 60) / (freeSpeedRPM * Math.toRadians(360 / (m_integratedAbsEncoder ? 1 : gearRatio)));
      m_allowableClosedLoopError = 5;

      // Configure the CANCoder as the remote sensor.
      if (m_integratedAbsEncoder)
      {
        m_motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        m_motor.configRemoteFeedbackFilter((CANCoder) m_angleEncoder.m_encoder,
                                           CTRE_remoteSensor.REMOTE_SENSOR_0.ordinal());
        ((CANCoder) m_angleEncoder.m_encoder).configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
      } else
      {
        m_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        m_motor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        m_motor.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Signed_PlusMinus180);
        setEncoder(0);
      }

      m_mainPIDSlotId = CTRE_slotIdx.Distance.ordinal();
      m_controlMode = ControlMode.Position;
      m_encoderRet = m_motor::getSelectedSensorPosition;

      setCurrentLimit(20);

      m_motor.configFeedbackNotContinuous(false, 100);
    }

    setPIDOutputRange(-powerLimit, powerLimit);
    setVoltageCompensation(12);

    optimizeCANFrames();

  }

  /**
   * Configure the maximum power (-1 to 1) the PID can output. This helps manage voltage pull for the drive base.
   *
   * @param min Minimum output.
   * @param max Maximum output.
   */
  @Override
  public void setPIDOutputRange(double min, double max)
  {
    m_motor.configPeakOutputReverse(min);
    m_motor.configPeakOutputForward(max);
  }

  /**
   * Set the PIDF coefficients for the closed loop PID onboard the SparkMax.
   *
   * @param P            Proportional gain for closed loop. This is multiplied by closed loop error in sensor units.
   *                     Default is 1.0
   * @param I            Integral gain for closed loop. This is multiplied by closed loop error in sensor units every
   *                     PID Loop.
   * @param D            Derivative gain for closed loop. This is multiplied by derivative error (sensor units per PID
   *                     loop). Default is 0.1
   * @param F            Feed Fwd gain for Closed loop.
   * @param integralZone Integral Zone can be used to auto clear the integral accumulator if the sensor pos is too far
   *                     from the target. This prevents unstable oscillation if the kI is too large. Value is in sensor
   *                     units.
   */
  @Override
  public void setPIDF(double P, double I, double D, double F, double integralZone)
  {
    m_motor.selectProfileSlot(m_mainPIDSlotId, m_mainPidId);
    // More Closed-Loop Configs at
    // https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#closed-loop-configs-per-slot-four-slots-available
    // Example at
    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20General/VelocityClosedLoop_ArbFeedForward/src/main/java/frc/robot/Robot.java
    m_motor.config_kP(m_mainPIDSlotId, P);
    m_motor.config_kI(m_mainPIDSlotId, I);
    m_motor.config_kD(m_mainPIDSlotId, D);
    m_motor.config_kF(m_mainPIDSlotId, F);
    m_motor.config_IntegralZone(m_mainPIDSlotId, integralZone);
    m_motor.configAllowableClosedloopError(m_mainPIDSlotId, m_allowableClosedLoopError);

    // If the closed loop error is within this threshold, the motor output will be neutral. Set to 0 to disable.
    // Value is in sensor units.
  }

  /**
   * Configures the conversion factor based upon which motor.
   *
   * @param conversionFactor Conversion from RPM to MPS for drive motor, and rotations to degrees for the turning
   *                         motor.
   */
  @Override
  public void setConversionFactor(double conversionFactor)
  {
    m_motor.configSelectedFeedbackCoefficient(conversionFactor);
  }

  /**
   * Set the target for the PID loop.
   *
   * @param target      The PID target to aim for.
   * @param feedforward The feedforward for this target.
   */
  @Override
  public void setTarget(double target, double feedforward)
  {
    m_motor.set(m_controlMode, target, DemandType.ArbitraryFeedForward, feedforward);
  }

  /**
   * Stop the motor by sending 0 volts to it.
   */
  @Override
  public void stop()
  {
    m_motor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Set the speed of the drive motor from -1 to 1.
   *
   * @param speed Speed from -1 to 1.
   */
  @Override
  public void set(double speed)
  {
    m_motor.set(ControlMode.PercentOutput, speed);
  }

  /**
   * Set the voltage of the motor.
   *
   * @param voltage Voltage to output.
   */
  @Override
  public void setVoltage(double voltage)
  {
    m_motor.setVoltage(voltage);
  }

  /**
   * Get the current value of the encoder corresponding to the PID.
   *
   * @return Current value of the encoder.
   */
  @Override
  public double get()
  {
    return m_encoderRet.get();
  }

  /**
   * Get the current value of the encoder.
   *
   * @return Current value of the encoder.
   */
  @Override
  public double getPosition()
  {
    return m_motor.getSelectedSensorPosition();
  }

  /**
   * Set the voltage compensation for the swerve module motor.
   *
   * @param nominalVoltage Nominal voltage for operation to output to.
   */
  @Override
  public void setVoltageCompensation(double nominalVoltage)
  {
    m_motor.configVoltageCompSaturation(nominalVoltage);
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
    m_motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, currentLimit, 1));
  }

  /**
   * Set the encoder value.
   *
   * @param value Value to set the encoder to.
   */
  @Override
  public void setEncoder(double value)
  {
    m_motor.setSelectedSensorPosition(value, m_mainPidId, 100);
  }

  /**
   * Check that the link is good on the swerve module and CAN bus is able to retrieve data.
   *
   * @return true on all devices are accessible over CAN.
   */
  @Override
  public boolean reachable()
  {
    return m_motor.getFirmwareVersion() != 0;
  }

  /**
   * Check if the absolute encoder is used inplace of the integrated encoder.
   *
   * @return true, if the absolute encoder is being used as the integrated controller.
   */
  @Override
  public boolean remoteIntegratedEncoder()
  {
    return m_integratedAbsEncoder;
  }

  /**
   * Optimize the CAN status frames to reduce utilization.
   */
  @Override
  public void optimizeCANFrames()
  {

  }

  /**
   * Save configuration data to the motor controller so it is persistent on reboot.
   */
  @Override
  public void saveConfig()
  {
    // Config is on the fly for falcons.
  }

  /**
   * Invert the motor.
   *
   * @param inverted Set the motor as inverted.
   */
  @Override
  public void setInverted(boolean inverted)
  {
    m_motor.setInverted(inverted);
  }

  @Override
  public double getAmps()
  {
    return m_motor.getStatorCurrent();
  }

  /**
   * The Talon SRX Slot profile used to configure the motor to use for the PID.
   */
  enum CTRE_slotIdx
  {
    Distance, Turning, Velocity, MotionProfile
  }

  /**
   * The TalonSRX PID to use onboard.
   */
  enum CTRE_pidIdx
  {
    PRIMARY_PID, AUXILIARY_PID, THIRD_PID, FOURTH_PID
  }

  enum CTRE_remoteSensor
  {
    REMOTE_SENSOR_0, REMOTE_SENSOR_1
  }

//  /**
//   * Set the PIDF coefficients for the closed loop PID onboard the TalonSRX.
//   *
//   * @param profile         The {@link CTRE_slotIdx} to use.
//   * @param P               Proportional gain for closed loop. This is multiplied by closed loop error in sensor
//   units.
//   *                        Note the closed loop output interprets a final value of 1023 as full output. So use a gain
//   *                        of '0.25' to get full output if err is 4096u (Mag Encoder 1 rotation)
//   * @param I               Integral gain for closed loop. This is multiplied by closed loop error in sensor units
//   every
//   *                        PID Loop. Note the closed loop output interprets a final value of 1023 as full output. So
//   *                        use a gain of '0.00025' to get full output if err is 4096u (Mag Encoder 1 rotation) after
//   *                        1000 loops
//   * @param D               Derivative gain for closed loop. This is multiplied by derivative error (sensor units per
//   *                        PID loop). Note the closed loop output interprets a final value of 1023 as full output. So
//   *                        use a gain of '250' to get full output if derr is 4096u per (Mag Encoder 1 rotation) per
//   *                        1000 loops (typ 1 sec)
//   * @param F               Feed Fwd gain for Closed loop. See documentation for calculation details. If using
//   velocity,
//   *                        motion magic, or motion profile, use (1023 * duty-cycle /
//   *                        sensor-velocity-sensor-units-per-100ms)
//   * @param integralZone    Integral Zone can be used to auto clear the integral accumulator if the sensor pos is too
//   *                        far from the target. This prevents unstable oscillation if the kI is too large. Value
//   is in
//   *                        sensor units. (ticks per 100ms)
//   * @param moduleMotorType Motor Type for swerve module.
//   */
//  private void setCTREPIDF(CTRE_slotIdx profile, double P, double I, double D, double F, double integralZone,
//                           ModuleMotorType moduleMotorType)
//  {
//    ((BaseTalon) (moduleMotorType == ModuleMotorType.DRIVE ? m_driveMotor
//                                                           : m_turningMotor)).selectProfileSlot(
//        profile.ordinal(), CTRE_pidIdx.PRIMARY_PID.ordinal());
//    // More Closed-Loop Configs at
//    // https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#closed-loop-configs-per-slot-four-slots-available
//    // Example at
//    // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20General/VelocityClosedLoop_ArbFeedForward/src/main/java/frc/robot/Robot.java
//    ((BaseTalon) (moduleMotorType == ModuleMotorType.DRIVE ? m_driveMotor : m_turningMotor)).config_kP(
//        profile.ordinal(), P);
//    ((BaseTalon) (moduleMotorType == ModuleMotorType.DRIVE ? m_driveMotor : m_turningMotor)).config_kI(
//        profile.ordinal(), I);
//    ((BaseTalon) (moduleMotorType == ModuleMotorType.DRIVE ? m_driveMotor : m_turningMotor)).config_kD(
//        profile.ordinal(), D);
//    ((BaseTalon) (moduleMotorType == ModuleMotorType.DRIVE ? m_driveMotor : m_turningMotor)).config_kF(
//        profile.ordinal(), F);
//
//    ((BaseTalon) (moduleMotorType == ModuleMotorType.DRIVE ? m_driveMotor
//                                                           : m_turningMotor)).config_IntegralZone(
//        profile.ordinal(), integralZone);
//
//    // If the closed loop error is within this threshold, the motor output will be neutral. Set to 0 to disable.
//    // Value is in sensor units.
//
//    ((BaseTalon) (moduleMotorType == ModuleMotorType.DRIVE ? m_driveMotor
//                                                           : m_turningMotor)).configAllowableClosedloopError(
//        profile.ordinal(), 0);
//
//  }
//
//  /**
//   * Set the angle using the onboard controller when working with CTRE Talons
//   *
//   * @param angle Angle in degrees
//   */
//  private void setCTREAngle(double angle)
//  {
//    ((BaseTalon) m_turningMotor).set(ControlMode.Position, angle);
//    // TODO: Pass feedforward down.
//  }
//
//  /**
//   * Set the velocity of the drive motor.
//   *
//   * @param velocity Velocity in meters per second.
//   */
//  private void setCTREDrive(double velocity)
//  {
//    ((BaseTalon) m_driveMotor).set(ControlMode.Velocity, driveFeedforward.calculate(velocity));
//  }
//
//  /**
//   * Set up the CTRE motors and configure class attributes correspondingly
//   *
//   * @param motor           Motor controller to configure.
//   * @param moduleMotorType Motor type to configure
//   * @param gearRatio       Gear ratio of the motor for one revolution.
//   */
//  private void setupCTREMotor(BaseTalon motor, ModuleMotorType moduleMotorType, double gearRatio)
//  {
//
//    // Purposely did not configure status frames since CTRE motors should be on a CANivore
//
//    motor.setSensorPhase(true);
//    motor.setNeutralMode(NeutralMode.Brake);
//    // Unable to use TalonFX configs since this should support both TalonSRX's and TalonFX's
//    setVoltageCompensation(12, moduleMotorType);
//    // Code is based off of.
//    // https://github.com/SwerveDriveSpecialties/swerve-lib-2022-unmaintained/blob/55f3f1ad9e6bd81e56779d022a40917aacf8d3b3/src/main/java/com/swervedrivespecialties/swervelib/ctre/Falcon500SteerControllerFactoryBuilder.java#L91
//    // and here
//    // https://github.com/SwerveDriveSpecialties/swerve-lib-2022-unmaintained/blob/55f3f1ad9e6bd81e56779d022a40917aacf8d3b3/src/main/java/com/swervedrivespecialties/swervelib/ctre/Falcon500DriveControllerFactoryBuilder.java#L52
//
//    if (moduleMotorType == ModuleMotorType.DRIVE)
//    {
//      setCurrentLimit(80, moduleMotorType);
//      // Math set's the coefficient to the OUTPUT of the ENCODER (ticks/100ms) which is the INPUT to the PID.
//      // We want to set the PID to use MPS == meters/second :)
//      // Dimensional analysis, solve for K
//      // ticks/100ms * K = meters/second
//      // ticks/100ms * 100ms/(1s=1000ms) * (pi*diameter)meters/(ticks[4096]*gearRatio)ticks = meters/second
//      // ticks/100ms * 1/10 * (pi*diameter)/(ticks[4096]*gearRatio)ticks = meters/second
//      // ticks/100ms * (pi*diameter)/((ticks[4096]*gearRatio)*10) = meters/second
//      // K = (pi*diameter)/((ticks[4096]*gearRatio)*10)
//      // Set the feedback sensor up earlier in setCANRemoteFeedbackSensor()
//      motor.configSelectedFeedbackCoefficient(((Math.PI * wheelDiameter) / ((4096 / gearRatio)) * 10));
//    } else
//    {
//      setCurrentLimit(20, moduleMotorType);
//      setPIDF(0.2, 0, 0.1, 0, 100, moduleMotorType);
//    }
//  }
//
//  /**
//   * Set the CANCoder to be the primary PID on the motor controller and configure the PID to accept inputs in degrees.
//   * The talon will communicate independently of the roboRIO to fetch the current CANCoder position (which will result
//   * in PID adjustments when using a CANivore).
//   *
//   * @param motor   Talon Motor controller to configure.
//   * @param encoder CANCoder to use as the remote sensor.
//   */
//  private void setupCTRECANCoderRemoteSensor(BaseTalon motor, CANCoder encoder)
//  {
//    motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
//    motor.configRemoteFeedbackFilter(encoder, CTRE_remoteSensor.REMOTE_SENSOR_0.ordinal());
//    motor.configSelectedFeedbackCoefficient((double) 360 / 4096); // Degrees/Ticks
//    // The CANCoder has 4096 ticks per 1 revolution.
//  }
//
//  /**
//   * Returns whether the turning motor is a CTRE motor.
//   *
//   * @return is the turning motor a CTRE motor?
//   */
//  private boolean isCTRETurningMotor()
//  {
//    return m_turningMotor instanceof BaseMotorController;
//  }
//
//  /**
//   * Returns whether the drive motor is a CTRE motor. All CTRE motors implement the {@link BaseMotorController} class.
//   * We will only support the TalonSRX and TalonFX.
//   *
//   * @return is the drive motor a CTRE motor?
//   */
//  private boolean isCTREDriveMotor()
//  {
//    return m_driveMotor instanceof TalonFX || m_driveMotor instanceof TalonSRX;
//  }

}

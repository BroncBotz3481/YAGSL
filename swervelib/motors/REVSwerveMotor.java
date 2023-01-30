package frc.robot.subsystems.swervedrive.swerve.motors;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive.swerve.SwerveEncoder;
import frc.robot.subsystems.swervedrive.swerve.SwerveMotor;
import java.util.function.Supplier;

public class REVSwerveMotor extends SwerveMotor
{

  private final int                   m_mainPidSlot;
  private final int                   m_secondaryPidSlot;
  private final ControlType           m_pidControlType;
  private final CANSparkMax           m_motor;
  private final MotorFeedbackSensor   m_encoder;
  private final boolean               m_integratedAbsEncoder;
  private final SparkMaxPIDController m_pid;
  private final ArbFFUnits            m_feedforwardUnits;
  private final Supplier<Double>      m_encoderRet, m_encoderPosRet;
  /**
   * kV feed forward for PID
   */
  private final double m_moduleRadkV;

  /**
   * Constructor for REV Swerve Motor, expecting CANSparkMax. Clears sticky faults and restores factory defaults.
   *
   * @param motor           SparkMAX motor controller.
   * @param absoluteEncoder The absolute encoder used for the module, if the motor is a turning motor and the encoder is
   *                        compatible it will set the encoder as the remote integrated encoder and does not need
   *                        periodic synchronization.
   * @param type            Swerve module motor type.
   * @param gearRatio       Gear ratio.
   * @param wheelDiameter   Wheel diameter in meters.
   * @param freeSpeedRPM    Free speed RPM of the motor.
   * @param powerLimit      Power limit for the motor.
   */
  public REVSwerveMotor(CANSparkMax motor, SwerveEncoder<?> absoluteEncoder, ModuleMotorType type, double gearRatio,
                        double wheelDiameter, double freeSpeedRPM, double powerLimit)
  {
    m_integratedAbsEncoder = absoluteEncoder.m_encoder instanceof AbsoluteEncoder && type == ModuleMotorType.TURNING;

    // Inspired by the following sources
    // https://github.com/SwerveDriveSpecialties/swerve-lib-2022-unmaintained/blob/55f3f1ad9e6bd81e56779d022a40917aacf8d3b3/src/main/java/com/swervedrivespecialties/swervelib/rev/NeoDriveControllerFactoryBuilder.java#L38
    // https://github.com/first95/FRC2022/blob/1f57d6837e04d8c8a89f4d83d71b5d2172f41a0e/SwervyBot/src/main/java/frc/robot/SwerveModule.java#L68
    // https://github.com/first95/FRC2022/blob/1f57d6837e04d8c8a89f4d83d71b5d2172f41a0e/SwervyBot/src/main/java/frc/robot/Constants.java#L89
    // https://github.com/AusTINCANsProgrammingTeam/2022Swerve/blob/main/2022Swerve/src/main/java/frc/robot/Constants.java
    motor.restoreFactoryDefaults();

    m_motor = motor;
    m_motorType = type;
    m_encoder = m_integratedAbsEncoder ? (AbsoluteEncoder) absoluteEncoder.m_encoder : motor.getEncoder();
    m_pid = motor.getPIDController();

    m_pid.setFeedbackDevice(m_encoder);

    motor.clearFaults();
    motor.setIdleMode(IdleMode.kBrake);

//    double powerDriving = .6, powerTurning = .4; // TODO: Change from magic numbers.

    if (type == ModuleMotorType.DRIVE)
    {
      m_moduleRadkV = 1;

      m_encoderRet = ((RelativeEncoder) m_encoder)::getVelocity;
      m_encoderPosRet = ((RelativeEncoder) m_encoder)::getPosition;
      m_feedforwardUnits = ArbFFUnits.kPercentOut;
      m_pidControlType = ControlType.kVelocity;
      m_mainPidSlot = REV_slotIdx.Velocity.ordinal();
      m_secondaryPidSlot = REV_slotIdx.Position.ordinal();

//      setPIDF(0.1, 0, 0, 0, 1);
      setPIDF(0.01, 0, 0.005, 0, 0);

      setConversionFactor(((Math.PI * wheelDiameter) / gearRatio) / 60);

      setCurrentLimit(50);
    } else
    {
      m_moduleRadkV = (12 * 60) / (freeSpeedRPM * Math.toRadians(360 / (m_integratedAbsEncoder ? 1 : gearRatio)));

      if (m_integratedAbsEncoder)
      {
        m_encoderPosRet = m_encoderRet = ((AbsoluteEncoder) m_encoder)::getPosition;
      } else
      {
        m_encoderPosRet = m_encoderRet = ((RelativeEncoder) m_encoder)::getPosition;
      }

      m_feedforwardUnits = ArbFFUnits.kVoltage;
      m_pidControlType = ControlType.kPosition;
      m_mainPidSlot = REV_slotIdx.Position.ordinal();
      m_secondaryPidSlot = REV_slotIdx.Velocity.ordinal();

      setPIDF(0.01, 0, 0.005, 0, 0);

      setConversionFactor(m_integratedAbsEncoder ? 360 : 360 / gearRatio);

      m_pid.setPositionPIDWrappingEnabled(true);
      m_pid.setPositionPIDWrappingMinInput(-180);
      m_pid.setPositionPIDWrappingMaxInput(180);

      setCurrentLimit(20);
    }

    setPIDOutputRange(-powerLimit, powerLimit);
    setVoltageCompensation(12);

    setEncoder(0);

    optimizeCANFrames();

    m_motor.setCANTimeout(0); // Spin off configurations in a different thread.
    if (!Robot.isReal())
    {
      REVPhysicsSim.getInstance().addSparkMax(m_motor, 2.6f, 5676);
    }
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
    m_pid.setOutputRange(min, max, m_mainPidSlot);
    m_pid.setOutputRange(min, max, m_secondaryPidSlot);
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
    // Example at
    // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java#L65-L71
    kP = P;
    kI = I;
    kD = D;
    kF = F;
    kIZ = integralZone;
    m_pid.setP(P, m_mainPidSlot);
    m_pid.setI(I, m_mainPidSlot);
    m_pid.setD(D, m_mainPidSlot);
    m_pid.setFF(F, m_mainPidSlot);
    m_pid.setIZone(integralZone, m_mainPidSlot);
    // m_pid.setOutputRange(-1, 1, m_mainPidSlot);
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
    if (m_motorType == ModuleMotorType.TURNING)
    {
      if (m_encoder instanceof AbsoluteEncoder)
      {
        ((AbsoluteEncoder) m_encoder).setPositionConversionFactor(conversionFactor);
        ((AbsoluteEncoder) m_encoder).setVelocityConversionFactor(conversionFactor / 60);
      } else
      {
        ((RelativeEncoder) m_encoder).setPositionConversionFactor(conversionFactor);
        ((RelativeEncoder) m_encoder).setVelocityConversionFactor(conversionFactor / 60);
      }
    } else
    {
      ((RelativeEncoder) m_encoder).setVelocityConversionFactor(conversionFactor);
      ((RelativeEncoder) m_encoder).setPositionConversionFactor(
          conversionFactor * 60 * (Robot.isReal() ? 1 : 42 * 60)); // RPS -> RPM sim
      // SparkMaxSimProfile assumes the velocity is in RPM and multiplies it by 60, in our use case velocity is in RPS
      // The Sim profile also neglects to take into account that there are 42 ticks per rotation.
    }
  }

  /**
   * Set the target for the PID loop.
   *
   * @param target      The PID target to aim for.
   * @param feedforward feedForward value.
   */
  @Override
  public void setTarget(double target, double feedforward)
  {
    this.target = target;
    m_pid.setReference(target, m_pidControlType, m_mainPidSlot, feedforward * m_moduleRadkV, m_feedforwardUnits);
  }

  /**
   * Stop the motor by sending 0 volts to it.
   */
  @Override
  public void stop()
  {
    m_motor.set(0);
  }

  /**
   * Set the speed of the drive motor from -1 to 1.
   *
   * @param speed Speed from -1 to 1.
   */
  @Override
  public void set(double speed)
  {
    m_motor.set(speed);
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
   * Get the current output.
   *
   * @return Output amps.
   */
  @Override
  public double getAmps()
  {
    return m_motor.getOutputCurrent();
  }

  /**
   * Get the current value of the encoder.
   *
   * @return Current value of the encoder.
   */
  @Override
  public double getPosition()
  {
    return m_encoderPosRet.get();
  }

  /**
   * Set the voltage compensation for the swerve module motor.
   *
   * @param nominalVoltage Nominal voltage for operation to output to.
   */
  @Override
  public void setVoltageCompensation(double nominalVoltage)
  {
    m_motor.enableVoltageCompensation(nominalVoltage);
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
    m_motor.setSmartCurrentLimit(currentLimit);
  }

  /**
   * Set the encoder value.
   *
   * @param value Value to set the encoder to.
   */
  @Override
  public void setEncoder(double value)
  {
    if (m_encoder instanceof RelativeEncoder)
    {
      ((RelativeEncoder) m_encoder).setPosition(value);
    }
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
  public void setCANStatusFrames(int CANStatus0, int CANStatus1, int CANStatus2, int CANStatus3, int CANStatus4)
  {
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, CANStatus0);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, CANStatus1);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, CANStatus2);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, CANStatus3);
    m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, CANStatus4);
    // TODO: Configure Status Frame 5 and 6 if necessary
    //  https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
  }

  /**
   * Check that the link is good on the swerve module and CAN bus is able to retrieve data.
   *
   * @return true on all devices are accessible over CAN.
   */
  @Override
  public boolean reachable()
  {
    // Based off of https://github.com/DigitalDislocators/SDS-MK4i-NEO-Swerve-Template/blob/main/src/main/java/frc/robot/subsystems/SwerveModuleSparkMax.java#L490
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
    // Taken from https://github.com/frc3512/SwerveBot-2022/blob/9d31afd05df6c630d5acb4ec2cf5d734c9093bf8/src/main/java/frc/lib/util/CANSparkMaxUtil.java#L67
    if (m_motorType == ModuleMotorType.DRIVE)
    {
      setCANStatusFrames(10, 20, 500, 500, 500);
    } else
    {
      setCANStatusFrames(10, 500, 20, 500, 500);
    }
  }

  /**
   * Save configuration data to the motor controller so it is persistent on reboot.
   */
  @Override
  public void saveConfig()
  {
    m_motor.burnFlash();
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

  /**
   * REV Slots for PID configuration.
   */
  enum REV_slotIdx
  {
    Position, Velocity, Simulation
  }
}

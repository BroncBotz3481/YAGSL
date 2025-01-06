// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package swervelib.motors;

import com.thethriftybot.Conversion;
import com.thethriftybot.Conversion.PositionUnit;
import com.thethriftybot.Conversion.VelocityUnit;
import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.thethriftybot.ThriftyNova.EncoderType;
import com.thethriftybot.ThriftyNova.PIDSlot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import java.util.List;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.parser.PIDFConfig;

/**
 * An implementation of {@link ThriftyNova} as a {@link SwerveMotor}.
 */
public class ThriftyNovaSwerve extends SwerveMotor
{

  /**
   * ThriftyNova Instance.
   */
  private       ThriftyNova   motor;
  /**
   * The Encoder type being used
   */
  private       EncoderType   encoderType;
  /**
   * Closed-loop PID controller.
   */
  public        PIDController pid;
  /**
   * Factory default already occurred.
   */
  private       boolean       factoryDefaultOccurred   = false;
  /**
   * Position conversion object for the motor encoder
   */
  private       Conversion    positionConversion;
  /**
   * Velocity conversion object for the motor encoder
   */
  private       Conversion    velocityConversion;
  /**
   * The position conversion factor for the encoder
   */
  private       double        positionConversionFactor = 1.0;
  /**
   * The position conversion factor for the encoder
   */
  private       double        velocityConversionFactor = 1.0 / 60.0;
  /**
   * {@link DCMotor} for simulation and calculations.
   */
  private final DCMotor       simMotor;

  /**
   * Initialize the swerve motor.
   *
   * @param motor        The SwerveMotor as a ThriftyNova object.
   * @param isDriveMotor Is the motor being initialized a drive motor?
   * @param motorType    {@link DCMotor} controlled by the {@link ThriftyNova}
   */
  public ThriftyNovaSwerve(ThriftyNova motor, boolean isDriveMotor, DCMotor motorType)
  {
    this.motor = motor;
    this.isDriveMotor = isDriveMotor;
    this.simMotor = motorType;
    factoryDefaults();
    clearStickyFaults();

    motor.usePIDSlot(PIDSlot.SLOT0);
    pid = new PIDController(0, 0, 0);
    motor.pid0.setPID(pid);

    if (isDriveMotor)
    {
      positionConversion = new Conversion(PositionUnit.ROTATIONS, EncoderType.INTERNAL);
      velocityConversion = new Conversion(VelocityUnit.ROTATIONS_PER_SEC, EncoderType.INTERNAL);
    } else
    {
      positionConversion = new Conversion(PositionUnit.DEGREES, EncoderType.INTERNAL);
      velocityConversion = new Conversion(VelocityUnit.DEGREES_PER_SEC, EncoderType.INTERNAL);
    }
  }

  /**
   * Initialize the {@link SwerveMotor} as a {@link ThriftyNova} connected to a Brushless Motor.
   *
   * @param id           CAN ID of the ThriftyNova.
   * @param isDriveMotor Is the motor being initialized a drive motor?
   * @param motor        {@link DCMotor} controlled by the {@link ThriftyNova}
   */
  public ThriftyNovaSwerve(int id, boolean isDriveMotor, DCMotor motor)
  {
    this(new ThriftyNova(id), isDriveMotor, motor);
  }

  /**
   * Set factory defaults on the motor controller.
   */
  @Override
  public void factoryDefaults()
  {
    // Factory defaults from https://docs.thethriftybot.com/thrifty-nova/gqCPUYXcVoOZ4KW3DqIr/software-resources/configure-controller-settings/factory-default
    if (!factoryDefaultOccurred)
    {
      motor.setInverted(false);
      motor.setBrakeMode(false);
      setCurrentLimit(40);
      motor.setEncoderPosition(0);
      motor.setMaxOutput(1.0);
      motor.setRampDown(100);
      motor.setRampUp(100);
      configureCANStatusFrames(0.25, 0.1, 0.25, 0.5, 0.50);
      motor.setSoftLimits(0, 0);
      configurePIDF(new PIDFConfig());
      motor.pid1.setP(0)
                .setI(0)
                .setD(0)
                .setFF(0.0);
      DriverStation.reportWarning("Factory defaults not implemented for ThriftyNovaSwerve", true);
      factoryDefaultOccurred = true;
    }
  }

  /**
   * Clear the sticky faults on the motor controller.
   */
  @Override
  public void clearStickyFaults()
  {
    motor.clearErrors();
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
    if (isDriveMotor)
    {
      positionConversion = new Conversion(PositionUnit.ROTATIONS, EncoderType.ABS);
      velocityConversion = new Conversion(VelocityUnit.ROTATIONS_PER_SEC, EncoderType.ABS);
    } else
    {
      positionConversion = new Conversion(PositionUnit.DEGREES, EncoderType.ABS);
      velocityConversion = new Conversion(VelocityUnit.DEGREES_PER_SEC, EncoderType.ABS);
    }
    motor.useEncoderType(EncoderType.ABS);
    return this;
  }

  @Override
  public void configureIntegratedEncoder(double positionConversionFactor)
  {
    motor.useEncoderType(EncoderType.INTERNAL);
    if (isDriveMotor)
    {
      positionConversion = new Conversion(PositionUnit.ROTATIONS, EncoderType.INTERNAL);
      velocityConversion = new Conversion(VelocityUnit.ROTATIONS_PER_SEC, EncoderType.INTERNAL);
    } else
    {
      positionConversion = new Conversion(PositionUnit.DEGREES, EncoderType.INTERNAL);
      velocityConversion = new Conversion(VelocityUnit.DEGREES_PER_SEC, EncoderType.INTERNAL);
    }
    this.positionConversionFactor = positionConversionFactor;
    this.velocityConversionFactor = positionConversionFactor / 60.0;
    configureCANStatusFrames(0.25, 0.01, 0.01, 0.02, 0.20);
  }

  /**
   * Set the CAN status frames.
   *
   * @param fault      Fault transmission rate
   * @param sensor     Sensor transmission rate
   * @param quadSensor External quad encoder transmission rate
   * @param control    Control frame transmission rate
   * @param current    Current feedback transmission rate
   */
  public void configureCANStatusFrames(
      double fault, double sensor, double quadSensor, double control, double current)
  {
    motor.canFreq.setFault(fault);
    motor.canFreq.setSensor(sensor);
    motor.canFreq.setQuadSensor(quadSensor);
    motor.canFreq.setControl(control);
    motor.canFreq.setCurrent(current);
    checkErrors("Configuring CAN status frames failed: ");
  }

  /**
   * Configure the PIDF values for the closed loop controller. 0 is disabled or off.
   *
   * @param config Configuration class holding the PIDF values.
   */
  @Override
  public void configurePIDF(PIDFConfig config)
  {
    motor.pid0.setP(config.p).setI(config.i).setD(config.d);
    checkErrors("Configuring PIDF failed: ");
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
    motor.setBrakeMode(isBrakeMode);
    checkErrors("Setting motor brake mode failed: ");
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
    checkErrors("Setting motor inversion failed: ");
  }

  /**
   * Save the configurations from flash to EEPROM.
   */
  @Override
  public void burnFlash()
  {
    // Do nothing
  }

  /**
   * Set the percentage output.
   *
   * @param percentOutput percent out for the motor controller.
   */
  @Override
  public void set(double percentOutput)
  {
    motor.setPercent(percentOutput);
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
    if (isDriveMotor)
    {
      motor.pid0.setFF(feedforward);
      motor.setVelocity(setpoint);
    } else
    {
      motor.setPosition(setpoint / 360.0);
    }
  }

  /**
   * Get the voltage output of the motor controller.
   *
   * @return Voltage output.
   */
  @Override
  public double getVoltage()
  {
    throw new UnsupportedOperationException("Unimplemented method 'getVoltage'");
  }

  /**
   * Set the voltage of the motor.
   *
   * @param voltage Voltage to set.
   */
  @Override
  public void setVoltage(double voltage)
  {
    motor.setPercent(voltage / RobotController.getBatteryVoltage());
  }

  /**
   * Get the voltage output of the motor controller.
   *
   * @return Voltage output.
   */
  @Override
  public double getAppliedOutput()
  {
    return motor.getStatorCurrent();
  }

  /**
   * Get the velocity of the integrated encoder.
   *
   * @return velocity in Meters Per Second, or Degrees per Second.
   */
  @Override
  public double getVelocity()
  {
    return velocityConversion.fromMotor(motor.getVelocity()) * velocityConversionFactor;
  }

  /**
   * Get the position of the integrated encoder.
   *
   * @return Position in Meters or Degrees.
   */
  @Override
  public double getPosition()
  {
    return positionConversion.fromMotor(motor.getPosition()) * positionConversionFactor;
  }

  /**
   * Set the integrated encoder position.
   *
   * @param position Integrated encoder position. Should be angle in degrees or meters.
   */
  @Override
  public void setPosition(double position)
  {
    motor.setEncoderPosition(positionConversion.toMotor(position));
  }

  @Override
  public void setVoltageCompensation(double nominalVoltage)
  {
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
    motor.setMaxCurrent(CurrentType.STATOR, currentLimit);
    checkErrors("Setting current limit failed: ");
  }

  /**
   * Set the maximum rate the open/closed loop output can change by.
   *
   * @param rampRate Time in seconds to go from 0 to full throttle.
   */
  @Override
  public void setLoopRampRate(double rampRate)
  {
    motor.setRampUp(rampRate);
    motor.setRampDown(rampRate);
    checkErrors("Setting loop ramp rate failed: ");
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
    return EncoderType.ABS == encoderType;
  }

  /**
   * Checks for errors in the motor and logs them if any are found.
   *
   * @param message the message to prepend to the log and print statement
   */
  private void checkErrors(String message)
  {
    List<ThriftyNova.Error> errors = motor.getErrors();
    if (errors.size() > 0)
    {
      for (ThriftyNova.Error error : errors)
      {
        DataLogManager.log(this.getClass().getSimpleName() + ": " + message + error.toString());
        System.out.println(this.getClass().getSimpleName() + ": " + message + error.toString());
      }
    }

  }

  @Override
  public DCMotor getSimMotor()
  {
    return simMotor;
  }
}

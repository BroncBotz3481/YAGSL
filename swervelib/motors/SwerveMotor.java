package swervelib.motors;

import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.parser.PIDFConfig;

/**
 * Swerve motor abstraction which defines a standard interface for motors within a swerve module.
 */
public abstract class SwerveMotor
{

  /**
   * The maximum amount of times the swerve motor will attempt to configure a motor if failures occur.
   */
  public final int     maximumRetries = 5;
  /**
   * Whether the swerve motor is a drive motor.
   */
  protected    boolean isDriveMotor;

  /**
   * Configure the factory defaults.
   */
  public abstract void factoryDefaults();

  /**
   * Clear the sticky faults on the motor controller.
   */
  public abstract void clearStickyFaults();

  /**
   * Set the absolute encoder to be a compatible absolute encoder.
   *
   * @param encoder The encoder to use.
   * @return The {@link SwerveMotor} for single line configuration.
   */
  public abstract SwerveMotor setAbsoluteEncoder(SwerveAbsoluteEncoder encoder);

  /**
   * Configure the integrated encoder for the swerve module. Sets the conversion factors for position and velocity.
   *
   * @param positionConversionFactor The conversion factor to apply for position.
   */
  public abstract void configureIntegratedEncoder(double positionConversionFactor);

  /**
   * Configure the PIDF values for the closed loop controller. 0 is disabled or off.
   *
   * @param config Configuration class holding the PIDF values.
   */
  public abstract void configurePIDF(PIDFConfig config);

  /**
   * Configure the PID wrapping for the position closed loop controller.
   *
   * @param minInput Minimum PID input.
   * @param maxInput Maximum PID input.
   */
  public abstract void configurePIDWrapping(double minInput, double maxInput);

  /**
   * Set the idle mode.
   *
   * @param isBrakeMode Set the brake mode.
   */
  public abstract void setMotorBrake(boolean isBrakeMode);

  /**
   * Set the motor to be inverted.
   *
   * @param inverted State of inversion.
   */
  public abstract void setInverted(boolean inverted);

  /**
   * Save the configurations from flash to EEPROM.
   */
  public abstract void burnFlash();

  /**
   * Set the percentage output.
   *
   * @param percentOutput percent out for the motor controller.
   */
  public abstract void set(double percentOutput);

  /**
   * Set the closed loop PID controller reference point.
   *
   * @param setpoint    Setpoint in meters per second or angle in degrees.
   * @param feedforward Feedforward in volt-meter-per-second or kV.
   */
  public abstract void setReference(double setpoint, double feedforward);

  /**
   * Set the closed loop PID controller reference point.
   *
   * @param setpoint    Setpoint in meters per second or angle in degrees.
   * @param feedforward Feedforward in volt-meter-per-second or kV.
   * @param position    Only used on the angle motor, the position of the motor in degrees.
   */
  public abstract void setReference(double setpoint, double feedforward, double position);

  /**
   * Get the velocity of the integrated encoder.
   *
   * @return velocity in meters per second or degrees per second.
   */
  public abstract double getVelocity();

  /**
   * Get the position of the integrated encoder.
   *
   * @return Position in meters or degrees.
   */
  public abstract double getPosition();

  /**
   * Set the integrated encoder position.
   *
   * @param position Integrated encoder position. Should be angle in degrees or meters per second.
   */
  public abstract void setPosition(double position);

  /**
   * Set the voltage compensation for the swerve module motor.
   *
   * @param nominalVoltage Nominal voltage for operation to output to.
   */
  public abstract void setVoltageCompensation(double nominalVoltage);

  /**
   * Set the current limit for the swerve drive motor, remember this may cause jumping if used in conjunction with
   * voltage compensation. This is useful to protect the motor from current spikes.
   *
   * @param currentLimit Current limit in AMPS at free speed.
   */
  public abstract void setCurrentLimit(int currentLimit);

  /**
   * Set the maximum rate the open/closed loop output can change by.
   *
   * @param rampRate Time in seconds to go from 0 to full throttle.
   */
  public abstract void setLoopRampRate(double rampRate);

  /**
   * Get the motor object from the module.
   *
   * @return Motor object.
   */
  public abstract Object getMotor();

  /**
   * Queries whether the absolute encoder is directly attached to the motor controller.
   *
   * @return connected absolute encoder state.
   */
  public abstract boolean isAttachedAbsoluteEncoder();
}

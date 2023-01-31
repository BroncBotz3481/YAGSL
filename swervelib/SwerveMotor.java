package frc.robot.subsystems.swervedrive.swerve;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.subsystems.swervedrive.swerve.motors.CTRESwerveMotor;
import frc.robot.subsystems.swervedrive.swerve.motors.REVSwerveMotor;

public abstract class SwerveMotor
{

  /**
   * Module motor type.
   */
  public ModuleMotorType m_motorType;
  /**
   * PIDF Values.
   */
  public double          kP, kI, kD, kF, kIZ;
  /**
   * Target value for PID.
   */
  public double target;

  /**
   * Constructor for Swerve Motor, expecting CANSparkMax or TalonFX. Clears sticky faults and restores factory
   * defaults.
   *
   * @param motor           Motor controller.
   * @param absoluteEncoder The absolute encoder used for the module, if the motor is a turning motor and the encoder is
   *                        compatible it will set the encoder as the remote integrated encoder and does not need
   *                        periodic synchronization.
   * @param type            Swerve module motor type.
   * @param gearRatio       Gear ratio.
   * @param wheelDiameter   Wheel diameter in meters.
   * @param freeSpeedRPM    Free speed RPM of the motor.
   * @param powerLimit      Power limit for the motor.
   * @param <MotorType>     Motor type to use.
   */
  public static <MotorType extends MotorController> SwerveMotor fromMotor(MotorType motor,
                                                                          SwerveEncoder<?> absoluteEncoder,
                                                                          ModuleMotorType type,
                                                                          double gearRatio,
                                                                          double wheelDiameter,
                                                                          double freeSpeedRPM, double powerLimit)
  {
    if (motor instanceof CANSparkMax)
    {
      return new REVSwerveMotor(((CANSparkMax) motor),
                                absoluteEncoder,
                                type,
                                gearRatio,
                                wheelDiameter,
                                freeSpeedRPM,
                                powerLimit);
    }
    if (motor instanceof WPI_TalonFX)
    {
      return new CTRESwerveMotor((WPI_TalonFX) motor,
                                 absoluteEncoder,
                                 type,
                                 gearRatio,
                                 wheelDiameter,
                                 freeSpeedRPM,
                                 powerLimit);
    }
    return null;
  }


  /**
   * Configure the maximum power (-1 to 1) the PID can output. This helps manage voltage pull for the drive base.
   *
   * @param min Minimum output.
   * @param max Maximum output.
   */
  public abstract void setPIDOutputRange(double min, double max);

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
  public abstract void setPIDF(double P, double I, double D, double F, double integralZone);

  /**
   * Configures the conversion factor based upon which motor.
   *
   * @param conversionFactor Conversion from RPM to MPS for drive motor, and rotations to degrees for the turning
   *                         motor.
   */
  public abstract void setConversionFactor(double conversionFactor);

  /**
   * Set the target for the PID loop.
   *
   * @param target      The PID target to aim for.
   * @param feedforward The feedforward for this target.
   */
  public abstract void setTarget(double target, double feedforward);

  /**
   * Stop the motor by sending 0 volts to it.
   */
  public abstract void stop();

  /**
   * Set the speed of the drive motor from -1 to 1.
   *
   * @param speed Speed from -1 to 1.
   */
  public abstract void set(double speed);

  /**
   * Set the voltage of the motor.
   *
   * @param voltage Voltage to output.
   */
  public abstract void setVoltage(double voltage);

  /**
   * Get the current value of the encoder corresponding to the PID.
   *
   * @return Current value of the encoder either in velocity or position..
   */
  public abstract double get();

  /**
   * Get the current output.
   *
   * @return Output amps.
   */
  public abstract double getAmps();

  /**
   * Get the current value of the encoder.
   *
   * @return Current value of the encoder.
   */
  public abstract double getPosition();

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
   * Set the encoder value.
   *
   * @param value Value to set the encoder to.
   */
  public abstract void setEncoder(double value);

  /**
   * Check that the link is good on the swerve module and CAN bus is able to retrieve data.
   *
   * @return true on all devices are accessible over CAN.
   */
  public abstract boolean reachable();

  /**
   * Check if the absolute encoder is used inplace of the integrated encoder.
   *
   * @return true, if the absolute encoder is being used as the integrated controller.
   */
  public abstract boolean remoteIntegratedEncoder();

  /**
   * Optimize the CAN status frames to reduce utilization.
   */
  public abstract void optimizeCANFrames();

  /**
   * Save configuration data to the motor controller so it is persistent on reboot.
   */
  public abstract void saveConfig();

  /**
   * Invert the motor.
   *
   * @param inverted Set the motor as inverted.
   */
  public abstract void setInverted(boolean inverted);

  /**
   * Motor type for the swerve drive module
   */
  public enum ModuleMotorType
  {
    DRIVE, TURNING
  }
}

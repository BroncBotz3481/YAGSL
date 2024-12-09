package swervelib;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import java.util.function.Supplier;
import swervelib.encoders.SwerveAbsoluteEncoder;

/**
 * Class to perform tests on the swerve drive.
 */
public class SwerveDriveTest
{

  /**
   * Tracks the voltage being applied to a motor
   */
  private static final MutVoltage         m_appliedVoltage = new MutVoltage(0, 0, Volts);
  /**
   * Tracks the distance travelled of a position motor
   */
  private static final MutDistance        m_distance       = new MutDistance(0, 0, Meter);
  /**
   * Tracks the velocity of a positional motor
   */
  private static final MutLinearVelocity  m_velocity       = new MutLinearVelocity(0, 9, MetersPerSecond);
  /**
   * Tracks the rotations of an angular motor
   */
  private static final MutAngle           m_anglePosition  = new MutAngle(0, 0, Degrees);
  /**
   * Tracks the velocity of an angular motor
   */
  private static final MutAngularVelocity m_angVelocity    = new MutAngularVelocity(0, 0, DegreesPerSecond);

  /**
   * Set the angle of the modules to a given {@link Rotation2d}
   *
   * @param swerveDrive {@link SwerveDrive} to use.
   * @param moduleAngle {@link Rotation2d} to set every module to.
   */
  public static void angleModules(SwerveDrive swerveDrive, Rotation2d moduleAngle)
  {
    for (SwerveModule swerveModule : swerveDrive.getModules())
    {
      swerveModule.getAngleMotor().setReference(moduleAngle.getDegrees(), 0);
    }
  }

  /**
   * Power the drive motors for the swerve drive to a set duty cycle percentage.
   *
   * @param swerveDrive {@link SwerveDrive} to control.
   * @param percentage  Duty cycle percentage of voltage to send to drive motors.
   */
  public static void powerDriveMotorsDutyCycle(SwerveDrive swerveDrive, double percentage)
  {
    for (SwerveModule swerveModule : swerveDrive.getModules())
    {
      swerveModule.getDriveMotor().set(percentage);
    }
  }

  /**
   * Power the angle motors for the swerve drive to a set percentage.
   *
   * @param swerveDrive {@link SwerveDrive} to control.
   * @param percentage  DutyCycle percentage to send to angle motors.
   */
  public static void powerAngleMotorsDutyCycle(SwerveDrive swerveDrive, double percentage)
  {
    for (SwerveModule swerveModule : swerveDrive.getModules())
    {
      swerveModule.getAngleMotor().set(percentage);
    }
  }

  /**
   * Power the drive motors for the swerve drive to a set voltage.
   *
   * @param swerveDrive {@link SwerveDrive} to control.
   * @param volts       DutyCycle percentage of voltage to send to drive motors.
   */
  public static void powerDriveMotorsVoltage(SwerveDrive swerveDrive, double volts)
  {
    for (SwerveModule swerveModule : swerveDrive.getModules())
    {
      swerveModule.getDriveMotor().setVoltage(volts);
    }
  }

  /**
   * Power the angle motors for the swerve drive to a set voltage.
   *
   * @param swerveDrive {@link SwerveDrive} to control.
   * @param volts       Voltage to send to angle motors.
   */
  public static void powerAngleMotorsVoltage(SwerveDrive swerveDrive, double volts)
  {
    for (SwerveModule swerveModule : swerveDrive.getModules())
    {
      swerveModule.getAngleMotor().setVoltage(volts);
    }
  }

  /**
   * Set the modules to center to 0.
   *
   * @param swerveDrive Swerve Drive to control.
   */
  public static void centerModules(SwerveDrive swerveDrive)
  {
    angleModules(swerveDrive, Rotation2d.fromDegrees(0));
  }

  /**
   * Find the minimum amount of power required to move the swerve drive motors.
   *
   * @param swerveDrive      {@link SwerveDrive} to control.
   * @param minMovement      Minimum amount of movement to drive motors.
   * @param testDelaySeconds Time in seconds for the motor to move.
   * @param maxVolts         The maximum voltage to send to drive motors.
   * @return minimum voltage required.
   */
  public static double findDriveMotorKV(SwerveDrive swerveDrive, double minMovement, double testDelaySeconds,
                                        double maxVolts)
  {
    double[] startingEncoders = new double[4];
    double   kV               = 0;

    SwerveDriveTest.powerDriveMotorsVoltage(swerveDrive, 0);
    SwerveModule[] modules = swerveDrive.getModules();
    for (int i = 0; i < modules.length; i++)
    {
      startingEncoders[i] = Math.abs(modules[i].getDriveMotor().getPosition());
    }

    for (double kV_new = 0; kV_new < maxVolts; kV_new += 0.0001)
    {

      SwerveDriveTest.powerDriveMotorsVoltage(swerveDrive, kV);
      boolean foundkV          = false;
      double  startTimeSeconds = Timer.getFPGATimestamp();
      while ((Timer.getFPGATimestamp() - startTimeSeconds) < testDelaySeconds && !foundkV)
      {
        for (int i = 0; i < modules.length; i++)
        {
          if ((modules[i].getDriveMotor().getPosition() - startingEncoders[i]) > minMovement)
          {
            foundkV = true;
            break;
          }
        }
      }
      if (foundkV)
      {
        SwerveDriveTest.powerDriveMotorsVoltage(swerveDrive, 0);
        kV = kV_new;
      }
    }
    return kV;
  }

  /**
   * Find the coupling ratio for all modules.
   *
   * @param swerveDrive {@link SwerveDrive} to operate with.
   * @param volts       Voltage to send to angle motors to spin.
   * @param automatic   Attempt to automatically spin the modules.
   * @return Average coupling ratio.
   */
  public static double findCouplingRatio(SwerveDrive swerveDrive, double volts, boolean automatic)
  {
    System.out.println("Stopping the Swerve Drive.");
    SwerveDriveTest.powerDriveMotorsVoltage(swerveDrive, 0);
    SwerveDriveTest.powerAngleMotorsVoltage(swerveDrive, 0);
    Timer.delay(1);
    double couplingRatioSum = 0;
    for (SwerveModule module : swerveDrive.getModules())
    {
      if (module.getAbsoluteEncoder() == null)
      {
        throw new RuntimeException("Absolute encoders are required to find the coupling ratio.");
      }
      SwerveAbsoluteEncoder absoluteEncoder = module.getAbsoluteEncoder();
      if (absoluteEncoder.readingError)
      {
        throw new RuntimeException("Absolute encoder encountered a reading error please debug.");
      }
      System.out.println("Fetching the current absolute encoder and drive encoder position.");
      module.getAngleMotor().setVoltage(0);
      Timer.delay(1);
      Rotation2d startingAbsoluteEncoderPosition = Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition());
      double driveEncoderPositionRotations = module.getDriveMotor().getPosition() /
                                             module.configuration.conversionFactors.drive.factor;
      if (automatic)
      {
        module.getAngleMotor().setVoltage(volts);
        Timer.delay(0.01);
        System.out.println("Rotating the module 360 degrees");
        while (!Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition()).equals(startingAbsoluteEncoderPosition))
          ;
        module.getAngleMotor().setVoltage(0);
      } else
      {
        DriverStation.reportWarning(
            "Spin the " + module.configuration.name + " module 360 degrees now, you have 1 minute.\n",
            false);
        Timer.delay(60);
      }
      double couplingRatio =
          (module.getDriveMotor().getPosition() / module.configuration.conversionFactors.drive.factor) -
          driveEncoderPositionRotations;
      DriverStation.reportWarning(module.configuration.name + " Coupling Ratio: " + couplingRatio, false);
      couplingRatioSum += couplingRatio;
    }
    DriverStation.reportWarning("Average Coupling Ratio: " + (couplingRatioSum / 4.0), false);
    return (couplingRatioSum / 4.0);
  }

  /**
   * Creates a SysIdRoutine.Config with a custom final timeout
   *
   * @param timeout - the most a SysIdRoutine should run
   * @return A custom SysIdRoutine.Config
   */
  public static Config createConfigCustomTimeout(double timeout)
  {
    return new Config(null, null, Seconds.of(timeout));
  }

  /**
   * Logs output, position and velocuty info form the drive motor to the SysIdRoutineLog <br /> Although SysIdRoutine
   * expects to be logging Voltage, this function logs in Duty-Cycle (percent output) because it results in correctly
   * adjusted values in the analysis for use in this library.
   *
   * @param module - the swerve module being logged
   * @param log    - the logger
   */
  public static void logDriveMotorDutyCycle(SwerveModule module, SysIdRoutineLog log)
  {
    logDriveMotorActivity(module, log, () -> module.getDriveMotor().getVoltage() / RobotController.getBatteryVoltage());
  }

  /**
   * Logs voltage, position and velocuty info form the drive motor to the SysIdRoutineLog
   *
   * @param module - the swerve module being logged
   * @param log    - the logger
   */
  public static void logDriveMotorVoltage(SwerveModule module, SysIdRoutineLog log)
  {
    logDriveMotorActivity(module, log, () -> module.getDriveMotor().getVoltage());
  }

  /**
   * Logs power, position and velocuty info form the drive motor to the SysIdRoutineLog
   *
   * @param module        - the swerve module being logged
   * @param log           - the logger
   * @param powerSupplied - a functional supplier of the power to be logged
   */
  public static void logDriveMotorActivity(SwerveModule module, SysIdRoutineLog log, Supplier<Double> powerSupplied)
  {
    double power    = powerSupplied.get();
    double distance = module.getPosition().distanceMeters;
    double velocity = module.getDriveMotor().getVelocity();
    SmartDashboard.putNumber("swerve/modules/" + module.configuration.name + "/SysId Drive Power", power);
    SmartDashboard.putNumber("swerve/modules/" + module.configuration.name + "/SysId Drive Position", distance);
    SmartDashboard.putNumber("swerve/modules/" + module.configuration.name + "/SysId Drive Velocity", velocity);
    log.motor("drive-" + module.configuration.name)
       .voltage(m_appliedVoltage.mut_replace(power, Volts))
       .linearPosition(m_distance.mut_replace(distance, Meters))
       .linearVelocity(m_velocity.mut_replace(velocity, MetersPerSecond));
  }

  /**
   * Sets up the SysId runner and logger for the drive motors
   *
   * @param config          - The SysIdRoutine.Config to use
   * @param swerveSubsystem - the subsystem to add to requirements
   * @param swerveDrive     - the SwerveDrive from which to access motor info
   * @param maxVolts        - The maximum voltage that should be applied to the drive motors.
   * @return A SysIdRoutine runner
   */
  public static SysIdRoutine setDriveSysIdRoutine(Config config, SubsystemBase swerveSubsystem,
                                                  SwerveDrive swerveDrive, double maxVolts)
  {
    return new SysIdRoutine(config, new SysIdRoutine.Mechanism(
        (Voltage voltage) -> {
          SwerveDriveTest.centerModules(swerveDrive);
          SwerveDriveTest.powerDriveMotorsVoltage(swerveDrive, Math.min(voltage.in(Volts), maxVolts));
        },
        log -> {
          for (SwerveModule module : swerveDrive.getModules())
          {
            logDriveMotorVoltage(module, log);
          }
        }, swerveSubsystem));
  }

  /**
   * Logs info about the angle motor to the SysIdRoutineLog. <br /> Although SysIdRoutine expects to be logging Voltage,
   * this function logs in Duty-Cycle (percent output) because it results in correctly adjusted values in the analysis
   * for use in this library.
   *
   * @param module - the swerve module being logged
   * @param log    - the logger
   */
  public static void logAngularMotorDutyCycle(SwerveModule module, SysIdRoutineLog log)
  {
    logAngularMotorActivity(module,
                            log,
                            () -> module.getAngleMotor().getVoltage() / RobotController.getBatteryVoltage());
  }

  /**
   * Logs info about the angle motor to the SysIdRoutineLog
   *
   * @param module - the swerve module being logged
   * @param log    - the logger
   */
  public static void logAngularMotorVoltage(SwerveModule module, SysIdRoutineLog log)
  {
    logAngularMotorActivity(module, log, () -> module.getAngleMotor().getVoltage());
  }

  /**
   * Logs info about the angle motor to the SysIdRoutineLog
   *
   * @param module        - the swerve module being logged
   * @param log           - the logger
   * @param powerSupplied - a functional supplier of the power to be logged
   */
  public static void logAngularMotorActivity(SwerveModule module, SysIdRoutineLog log, Supplier<Double> powerSupplied)
  {
    double power    = powerSupplied.get();
    double angle    = module.getAngleMotor().getPosition();
    double velocity = module.getAngleMotor().getVelocity();
    SmartDashboard.putNumber("swerve/modules/" + module.configuration.name + "/SysId Angle Power", power);
    SmartDashboard.putNumber("swerve/modules/" + module.configuration.name + "/SysId Angle Position", angle);
    SmartDashboard.putNumber("swerve/modules/" + module.configuration.name + "/SysId Absolute Encoder Velocity",
                             velocity);
    log.motor("angle-" + module.configuration.name)
       .voltage(m_appliedVoltage.mut_replace(power, Volts))
       .angularPosition(m_anglePosition.mut_replace(angle, Degrees))
       .angularVelocity(m_angVelocity.mut_replace(velocity, DegreesPerSecond));
  }

  /**
   * Sets up the SysId runner and logger for the angle motors
   *
   * @param config          - The SysIdRoutine.Config to use
   * @param swerveSubsystem - the subsystem to add to requirements
   * @param swerveDrive     - the SwerveDrive from which to access motor info
   * @return A SysIdRoutineRunner
   */
  public static SysIdRoutine setAngleSysIdRoutine(Config config, SubsystemBase swerveSubsystem,
                                                  SwerveDrive swerveDrive)
  {
    return new SysIdRoutine(config, new SysIdRoutine.Mechanism(
        (Voltage voltage) -> {
          SwerveDriveTest.powerAngleMotorsVoltage(swerveDrive, voltage.in(Volts));
          SwerveDriveTest.powerDriveMotorsVoltage(swerveDrive, 0);
        },
        log -> {
          for (SwerveModule module : swerveDrive.getModules())
          {
            logAngularMotorVoltage(module, log);
          }
        }, swerveSubsystem));
  }

  /**
   * Creates a command that can be mapped to a button or other trigger. Delays can be set to customize the length of
   * each part of the SysId Routine
   *
   * @param sysIdRoutine   - The Sys Id routine runner
   * @param delay          - seconds between each portion to allow motors to spin down, etc...
   * @param quasiTimeout   - seconds to run the Quasistatic routines, so robot doesn't get too far
   * @param dynamicTimeout - seconds to run the Dynamic routines, 2-3 secs should be enough
   * @return A command that can be mapped to a button or other trigger
   */
  public static Command generateSysIdCommand(SysIdRoutine sysIdRoutine, double delay, double quasiTimeout,
                                             double dynamicTimeout)
  {
    return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(quasiTimeout)
                       .andThen(Commands.waitSeconds(delay))
                       .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(quasiTimeout))
                       .andThen(Commands.waitSeconds(delay))
                       .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(dynamicTimeout))
                       .andThen(Commands.waitSeconds(delay))
                       .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(dynamicTimeout));
  }
}

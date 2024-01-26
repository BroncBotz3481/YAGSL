package swervelib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import swervelib.encoders.SwerveAbsoluteEncoder;

/**
 * Class to perform tests on the swerve drive.
 */
public class SwerveDriveTest
{

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
      swerveModule.setDesiredState(new SwerveModuleState(0, moduleAngle), false, true);
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
  public static void powerAngleMotors(SwerveDrive swerveDrive, double percentage)
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
                                             module.configuration.conversionFactors.drive;
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
      double couplingRatio = (module.getDriveMotor().getPosition() / module.configuration.conversionFactors.drive) -
                             driveEncoderPositionRotations;
      DriverStation.reportWarning(module.configuration.name + " Coupling Ratio: " + couplingRatio, false);
      couplingRatioSum += couplingRatio;
    }
    DriverStation.reportWarning("Average Coupling Ratio: " + (couplingRatioSum / 4.0), false);
    return (couplingRatioSum / 4.0);
  }
}

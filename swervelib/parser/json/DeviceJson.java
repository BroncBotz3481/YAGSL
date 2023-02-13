package frc.robot.subsystems.swervedrive2.swervelib.parser.json;

import frc.robot.subsystems.swervedrive2.swervelib.encoders.CANCoderSwerve;
import frc.robot.subsystems.swervedrive2.swervelib.encoders.SparkMaxEncoderSwerve;
import frc.robot.subsystems.swervedrive2.swervelib.encoders.SwerveAbsoluteEncoder;
import frc.robot.subsystems.swervedrive2.swervelib.imu.NavXSwerve;
import frc.robot.subsystems.swervedrive2.swervelib.imu.Pigeon2Swerve;
import frc.robot.subsystems.swervedrive2.swervelib.imu.PigeonSwerve;
import frc.robot.subsystems.swervedrive2.swervelib.imu.SwerveIMU;
import frc.robot.subsystems.swervedrive2.swervelib.motors.SparkMaxSwerve;
import frc.robot.subsystems.swervedrive2.swervelib.motors.SwerveMotor;

/**
 * Device JSON parsed class. Used to access the JSON data.
 */
public class DeviceJson
{

  /**
   * The device type, e.g. pigeon/pigeon2/sparkmax/talonfx/navx
   */
  public String type;
  /**
   * The CAN ID or pin ID of the device.
   */
  public int    id;
  /**
   * The CAN bus name which the device resides on if using CAN.
   */
  public String canbus = "";

  /**
   * Create a {@link SwerveAbsoluteEncoder} from the current configuration.
   *
   * @return {@link SwerveAbsoluteEncoder} given.
   */
  public SwerveAbsoluteEncoder createEncoder()
  {
    switch (type)
    {
      case "integrated":
      case "attached":
        return null;
      case "cancoder":
        return new CANCoderSwerve(id, canbus != null ? canbus : "");
      default:
        throw new RuntimeException(type + " is not a recognized absolute encoder type.");
    }
  }

  /**
   * Create a {@link SwerveIMU} from the given configuration.
   *
   * @return {@link SwerveIMU} given.
   */
  public SwerveIMU createIMU()
  {
    switch (type)
    {
      case "navx":
        return new NavXSwerve();
      case "pigeon":
        return new PigeonSwerve(id);
      case "pigeon2":
        return new Pigeon2Swerve(id, canbus != null ? canbus : "");
      default:
        throw new RuntimeException(type + " is not a recognized absolute encoder type.");
    }
  }

  /**
   * Create a {@link SwerveMotor} from the given configuration.
   *
   * @param isDriveMotor If the motor being generated is a drive motor.
   * @return {@link SwerveMotor} given.
   */
  public SwerveMotor createMotor(boolean isDriveMotor)
  {
    if (type.equals("sparkmax"))
    {
      return new SparkMaxSwerve(id, isDriveMotor);
    }
    throw new RuntimeException(type + " is not a recognized absolute encoder type.");
  }

  /**
   * Create a {@link SwerveAbsoluteEncoder} from the data port on the motor controller.
   *
   * @param motor The motor to create the absolute encoder from.
   * @return {@link SwerveAbsoluteEncoder} from the motor controller.
   */
  public SwerveAbsoluteEncoder createIntegratedEncoder(SwerveMotor motor)
  {
    if (type.equals("sparkmax"))
    {
      return new SparkMaxEncoderSwerve(motor);
    }
    throw new RuntimeException("Could not create absolute encoder from data port of " + type + " id " + id);
  }
}

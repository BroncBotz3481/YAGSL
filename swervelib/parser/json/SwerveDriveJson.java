package swervelib.parser.json;

/**
 * {@link swervelib.SwerveDrive} JSON parsed class. Used to access parsed data from the swervedrive.json file.
 */
public class SwerveDriveJson
{

  /**
   * Maximum robot speed in feet per second.
   */
  public double     maxSpeed;
  /**
   * Optimal voltage to compensate to and base feedforward calculations off of.
   */
  public double     optimalVoltage;
  /**
   * Robot IMU used to determine heading of the robot.
   */
  public DeviceJson imu;
  /**
   * Invert the IMU of the robot.
   */
  public boolean    invertedIMU;
  /**
   * Module JSONs in order clockwise order starting from front left.
   */
  public String[]   modules;
}

package swervelib.parser.json;

/**
 * Used to store ints for motor configuration.
 */
public class MotorConfigInt
{

  /**
   * Drive motor.
   */
  public int drive;
  /**
   * Angle motor.
   */
  public int angle;

  /**
   * Default constructor.
   */
  public MotorConfigInt()
  {
  }

  /**
   * Default constructor with values.
   *
   * @param drive Drive data.
   * @param angle Angle data.
   */
  public MotorConfigInt(int drive, int angle)
  {
    this.angle = angle;
    this.drive = drive;
  }
}

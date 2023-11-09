package swervelib.parser.json;

/**
 * Used to store doubles for motor configuration.
 */
public class MotorConfigDouble
{

  /**
   * Drive motor.
   */
  public double drive;
  /**
   * Angle motor.
   */
  public double angle;

  /**
   * Default constructor.
   */
  public MotorConfigDouble()
  {
  }

  /**
   * Default constructor.
   *
   * @param angle Angle data.
   * @param drive Drive data.
   */
  public MotorConfigDouble(double angle, double drive)
  {
    this.angle = angle;
    this.drive = drive;
  }
}

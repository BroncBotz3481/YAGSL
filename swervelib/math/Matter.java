package swervelib.math;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * Object with significant mass that needs to be taken into account.
 */
public class Matter
{

  /**
   * Position in meters from robot center in 3d space.
   */
  public Translation3d position;
  /**
   * Mass in kg of object.
   */
  public double        mass;

  /**
   * Construct an object representing some significant matter on the robot.
   *
   * @param position Position of the matter in meters.
   * @param mass     Mass in kg.
   */
  public Matter(Translation3d position, double mass)
  {
    this.mass = mass;
    this.position = position;
  }

  /**
   * Get the center mass of the object.
   *
   * @return center mass = position * mass
   */
  public Translation3d massMoment()
  {
    return position.times(mass);
  }
}

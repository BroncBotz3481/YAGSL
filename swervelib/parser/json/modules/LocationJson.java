package swervelib.parser.json.modules;

/**
 * Location JSON parsed class. Used to access the JSON data. Module locations, in inches, as distances to the center of
 * the robot. +x is towards the robot front, and +y is towards robot left.
 */
public class LocationJson
{

  /**
   * Location of the swerve module in inches from the center of the robot horizontally.
   */
  public double front = 0, x = 0;
  /**
   * Location of the swerve module in inches from the center of the robot vertically.
   */
  public double left = 0, y = 0;
}

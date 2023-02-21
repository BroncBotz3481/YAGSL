package swervelib.parser.json.modules;

/**
 * Location JSON parsed class. Used to access the JSON data. Module locations, in inches, as distances to the center of
 * the robot. Positive x is torwards the robot front, and * +y is torwards robot left.
 */
public class LocationJson
{

  /**
   * Location of the swerve module in inches from the center of the robot horizontally.
   */
  public double x;
  /**
   * Location of the swerve module in inches from the center of the robot vertically.
   */
  public double y;
}

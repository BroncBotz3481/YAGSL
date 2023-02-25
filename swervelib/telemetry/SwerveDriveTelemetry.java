package swervelib.telemetry;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Telemetry to describe the {@link swervelib.SwerveDrive} following frc-web-components. (Which follows AdvantageKit)
 */
public class SwerveDriveTelemetry
{

  /**
   * The current telemetry verbosity level.
   */
  public static TelemetryVerbosity verbosity             = TelemetryVerbosity.MACHINE;
  /**
   * State of simulation of the Robot, used to optimize retrieval.
   */
  public static boolean            isSimulation          = RobotBase.isSimulation();
  /**
   * The number of swerve modules
   */
  public static int                moduleCount;
  /**
   * The number of swerve modules
   */
  public static double[]           wheelLocations;
  /**
   * An array of rotation and velocity values describing the measured state of each swerve module
   */
  public static double[]           measuredStates;
  /**
   * An array of rotation and velocity values describing the desired state of each swerve module
   */
  public static double[]           desiredStates;
  /**
   * The robot's current rotation based on odometry or gyro readings
   */
  public static double             robotRotation         = 0;
  /**
   * The maximum achievable speed of the modules, used to adjust the size of the vectors.
   */
  public static double             maxSpeed;
  /**
   * The units of the module rotations and robot rotation
   */
  public static String             rotationUnit          = "degrees";
  /**
   * The distance between the left and right modules.
   */
  public static double             sizeLeftRight;
  /**
   * The distance between the front and back modules.
   */
  public static double             sizeFrontBack;
  /**
   * The direction the robot should be facing when the "Robot Rotation" is zero or blank. This option is often useful to
   * align with odometry data or match videos. 'up', 'right', 'down' or 'left'
   */
  public static String             forwardDirection      = "up";
  /**
   * The maximum achievable angular velocity of the robot. This is used to visualize the angular velocity from the
   * chassis speeds properties.
   */
  public static double             maxAngularVelocity;
  /**
   * The maximum achievable angular velocity of the robot. This is used to visualize the angular velocity from the
   * chassis speeds properties.
   */
  public static double[]           measuredChassisSpeeds = new double[3];
  /**
   * Describes the desired forward, sideways and angular velocity of the robot.
   */
  public static double[]           desiredChassisSpeeds  = new double[3];

  /**
   * Upload data to smartdashboard
   */
  public static void updateData()
  {
    SmartDashboard.putNumber("swerve/moduleCount", moduleCount);
    SmartDashboard.putNumberArray("swerve/wheelLocations", wheelLocations);
    SmartDashboard.putNumberArray("swerve/measuredStates", measuredStates);
    SmartDashboard.putNumberArray("swerve/desiredStates", desiredStates);
    SmartDashboard.putNumber("swerve/robotRotation", robotRotation);
    SmartDashboard.putNumber("swerve/maxSpeed", maxSpeed);
    SmartDashboard.putString("swerve/rotationUnit", rotationUnit);
    SmartDashboard.putNumber("swerve/sizeLeftRight", sizeLeftRight);
    SmartDashboard.putNumber("swerve/sizeFrontBack", sizeFrontBack);
    SmartDashboard.putString("swerve/forwardDirection", forwardDirection);
    SmartDashboard.putNumber("swerve/maxAngularVelocity", maxAngularVelocity);
    SmartDashboard.putNumberArray("swerve/measuredChassisSpeeds", measuredChassisSpeeds);
    SmartDashboard.putNumberArray("swerve/desiredChassisSpeeds", desiredChassisSpeeds);
  }

  /**
   * Verbosity of telemetry data sent back.
   */
  public enum TelemetryVerbosity
  {
    /**
     * No telemetry data is sent back.
     */
    NONE,
    /**
     * Low telemetry data, only post the robot position on the field.
     */
    LOW,
    /**
     * Full swerve drive data is sent back in both human and machine readable forms.
     */
    HIGH,
    /**
     * Only send the machine readable data related to swerve drive.
     */
    MACHINE
  }
}

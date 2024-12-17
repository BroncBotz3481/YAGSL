package swervelib.telemetry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import swervelib.SwerveDrive;

/**
 * Telemetry to describe the {@link swervelib.SwerveDrive} following frc-web-components. (Which follows AdvantageKit)
 */
public class SwerveDriveTelemetry
{

  /**
   * An {@link Alert} for if the CAN ID is greater than 40.
   */
  public static final  Alert                                   canIdWarning             = new Alert("JSON",
                                                                                                    "CAN IDs greater than 40 can cause undefined behaviour, please use a CAN ID below 40!",
                                                                                                    AlertType.kWarning);
  /**
   * An {@link Alert} for if there is an I2C lockup issue on the roboRIO.
   */
  public static final  Alert                                   i2cLockupWarning         = new Alert("IMU",
                                                                                                    "I2C lockup issue detected on roboRIO. Check console for more information.",
                                                                                                    AlertType.kWarning);
  /**
   * NavX serial comm issue.
   */
  public static final  Alert                                   serialCommsIssueWarning  = new Alert("IMU",
                                                                                                    "Serial comms is interrupted with USB and other serial traffic and causes intermittent connected/disconnection issues. Please consider another protocol or be mindful of this.",
                                                                                                    AlertType.kWarning);
  /**
   * Module counter publisher for NT4
   */
  private static final DoublePublisher                         moduleCountPublisher
                                                                                        = NetworkTableInstance.getDefault()
                                                                                                              .getDoubleTopic(
                                                                                                                  "swerve/moduleCount")
                                                                                                              .publish();
  /**
   * Module measured states for Nt4
   */
  private static final DoubleArrayPublisher                    measuredStatesArrayPublisher
                                                                                        = NetworkTableInstance.getDefault()
                                                                                                              .getDoubleArrayTopic(
                                                                                                                  "swerve/measuredStates")
                                                                                                              .publish();
  /**
   * Desired states for NT4
   */
  private static final DoubleArrayPublisher                    desiredStatesArrayPublisher
                                                                                        = NetworkTableInstance.getDefault()
                                                                                                              .getDoubleArrayTopic(
                                                                                                                  "swerve/desiredStates")
                                                                                                              .publish();
  /**
   * Measured chassis speeds array publisher.
   */
  private static final DoubleArrayPublisher                    measuredChassisSpeedsArrayPublisher
                                                                                        = NetworkTableInstance.getDefault()
                                                                                                              .getDoubleArrayTopic(
                                                                                                                  "swerve/measuredChassisSpeeds")
                                                                                                              .publish();
  /**
   * Desired chassis speeds array publisher.
   */
  private static final DoubleArrayPublisher                    desiredChassisSpeedsArrayPublisher
                                                                                        = NetworkTableInstance.getDefault()
                                                                                                              .getDoubleArrayTopic(
                                                                                                                  "swerve/desiredChassisSpeeds")
                                                                                                              .publish();
  /**
   * Robot rotation publisher.
   */
  private static final DoublePublisher                         robotRotationPublisher
                                                                                        = NetworkTableInstance.getDefault()
                                                                                                              .getDoubleTopic(
                                                                                                                  "swerve/robotRotation")
                                                                                                              .publish();
  /**
   * Max angular velocity publisher.
   */
  private static final DoublePublisher                         maxAngularVelocityPublisher
                                                                                        = NetworkTableInstance.getDefault()
                                                                                                              .getDoubleTopic(
                                                                                                                  "swerve/maxAngularVelocity")
                                                                                                              .publish();
  /**
   * Struct publisher for AdvantageScope swerve widgets.
   */
  private static final StructArrayPublisher<SwerveModuleState> measuredStatesStruct
                                                                                        = NetworkTableInstance.getDefault()
                                                                                                              .getStructArrayTopic(
                                                                                                                  "swerve/advantagescope/currentStates",
                                                                                                                  SwerveModuleState.struct)
                                                                                                              .publish();
  /**
   * Struct publisher for AdvantageScope swerve widgets.
   */
  private static final StructArrayPublisher<SwerveModuleState> desiredStatesStruct
                                                                                        = NetworkTableInstance.getDefault()
                                                                                                              .getStructArrayTopic(
                                                                                                                  "swerve/advantagescope/desiredStates",
                                                                                                                  SwerveModuleState.struct)
                                                                                                              .publish();
  /**
   * Measured {@link ChassisSpeeds} for NT4 AdvantageScope swerve widgets.
   */
  private static final StructPublisher<ChassisSpeeds>          measuredChassisSpeedsStruct
                                                                                        = NetworkTableInstance.getDefault()
                                                                                                              .getStructTopic(
                                                                                                                  "swerve/advantagescope/measuredChassisSpeeds",
                                                                                                                  ChassisSpeeds.struct)
                                                                                                              .publish();
  /**
   * Desired {@link ChassisSpeeds} for NT4 AdvantageScope swerve widgets.
   */
  private static final StructPublisher<ChassisSpeeds>          desiredChassisSpeedsStruct
                                                                                        = NetworkTableInstance.getDefault()
                                                                                                              .getStructTopic(
                                                                                                                  "swerve/advantagescope/desiredChassisSpeeds",
                                                                                                                  ChassisSpeeds.struct)
                                                                                                              .publish();
  /**
   * Robot {@link Rotation2d} for AdvantageScope swerve widgets.
   */
  private static final StructPublisher<Rotation2d>             robotRotationStruct
                                                                                        = NetworkTableInstance.getDefault()
                                                                                                              .getStructTopic(
                                                                                                                  "swerve/advantagescope/robotRotation",
                                                                                                                  Rotation2d.struct)
                                                                                                              .publish();
  /**
   * Wheel locations array publisher for NT4.
   */
  private static final DoubleArrayPublisher wheelLocationsArrayPublisher = NetworkTableInstance.getDefault()
                                                                                               .getDoubleArrayTopic(
                                                                                                   "swerve/wheelLocation")
                                                                                               .publish();
  /**
   * Max speed publisher for NT4.
   */
  private static final DoublePublisher      maxSpeedPublisher            = NetworkTableInstance.getDefault()
                                                                                               .getDoubleTopic(
                                                                                                   "swerve/maxSpeed")
                                                                                               .publish();
  /**
   * Rotation unit for NT4.
   */
  private static final StringPublisher      rotationUnitPublisher        = NetworkTableInstance.getDefault()
                                                                                               .getStringTopic(
                                                                                                   "swerve/rotationUnit")
                                                                                               .publish();
  /**
   * Chassis width publisher
   */
  private static final DoublePublisher      sizeLeftRightPublisher       = NetworkTableInstance.getDefault()
                                                                                               .getDoubleTopic(
                                                                                                   "swerve/sizeLeftRight")
                                                                                               .publish();
  /**
   * Chassis Length publisher.
   */
  private static final DoublePublisher      sizeFrontBackPublisher       = NetworkTableInstance.getDefault()
                                                                                               .getDoubleTopic(
                                                                                                   "swerve/sizeFrontBack")
                                                                                               .publish();
  /**
   * Chassis direction widget publisher.
   */
  private static final StringPublisher      forwardDirectionPublisher    = NetworkTableInstance.getDefault()
                                                                                               .getStringTopic(
                                                                                                   "swerve/forwardDirection")
                                                                                               .publish();
  /**
   * Odometry cycle time, updated whenever {@link SwerveDrive#updateOdometry()} is called.
   */
  private static final DoublePublisher      odomCycleTime
                                                                         = NetworkTableInstance.getDefault()
                                                                                               .getDoubleTopic(
                                                                                                   "swerve/odomCycleMS")
                                                                                               .publish();
  /**
   * Control cycle time, updated whenever
   * {@link swervelib.SwerveModule#setDesiredState(SwerveModuleState, boolean, double)} is called for the last module.
   */
  private static final DoublePublisher      ctrlCycleTime
                                                                         = NetworkTableInstance.getDefault()
                                                                                               .getDoubleTopic(
                                                                                                   "swerve/controlCycleMS")
                                                                                               .publish();
  /**
   * Odometry timer to track cycle times.
   */
  private static final Timer                odomTimer                    = new Timer();
  /**
   * Control timer to track cycle times.
   */
  private static final Timer                ctrlTimer                    = new Timer();
  /**
   * Measured swerve module states object.
   */
  public static        SwerveModuleState[]  measuredStatesObj
                                                                         = new SwerveModuleState[4];
  /**
   * Desired swerve module states object
   */
  public static        SwerveModuleState[]  desiredStatesObj
                                                                         = new SwerveModuleState[4];
  /**
   * The maximum achievable angular velocity of the robot. This is used to visualize the angular velocity from the
   * chassis speeds properties.
   */
  public static        ChassisSpeeds        measuredChassisSpeedsObj     = new ChassisSpeeds();
  /**
   * Describes the desired forward, sideways and angular velocity of the robot.
   */
  public static        ChassisSpeeds        desiredChassisSpeedsObj      = new ChassisSpeeds();
  /**
   * The robot's current rotation based on odometry or gyro readings
   */
  public static        Rotation2d           robotRotationObj             = new Rotation2d();
  /**
   * The current telemetry verbosity level.
   */
  public static        TelemetryVerbosity   verbosity
                                                                         = TelemetryVerbosity.MACHINE;
  /**
   * State of simulation of the Robot, used to optimize retrieval.
   */
  public static        boolean              isSimulation
                                                                         = RobotBase.isSimulation();
  /**
   * The number of swerve modules
   */
  public static        int                  moduleCount;
  /**
   * The Locations of the swerve drive wheels.
   */
  public static        double[]             wheelLocations;
  /**
   * An array of rotation and velocity values describing the measured state of each swerve module
   */
  public static        double[]             measuredStates;
  /**
   * An array of rotation and velocity values describing the desired state of each swerve module
   */
  public static        double[]             desiredStates;
  /**
   * The robot's current rotation based on odometry or gyro readings
   */
  public static        double               robotRotation                = 0;
  /**
   * The maximum achievable speed of the modules, used to adjust the size of the vectors.
   */
  public static        double               maxSpeed;
  /**
   * The units of the module rotations and robot rotation
   */
  public static        String               rotationUnit                 = "degrees";
  /**
   * The distance between the left and right modules.
   */
  public static        double               sizeLeftRight;
  /**
   * The distance between the front and back modules.
   */
  public static        double               sizeFrontBack;
  /**
   * The direction the robot should be facing when the "Robot Rotation" is zero or blank. This option is often useful to
   * align with odometry data or match videos. 'up', 'right', 'down' or 'left'
   */
  public static        String               forwardDirection             = "up";
  /**
   * The maximum achievable angular velocity of the robot. This is used to visualize the angular velocity from the
   * chassis speeds properties.
   */
  public static        double               maxAngularVelocity;
  /**
   * The maximum achievable angular velocity of the robot. This is used to visualize the angular velocity from the
   * chassis speeds properties.
   */
  public static        double[]             measuredChassisSpeeds        = new double[3];
  /**
   * Describes the desired forward, sideways and angular velocity of the robot.
   */
  public static        double[]             desiredChassisSpeeds         = new double[3];
  /**
   * Update the telemetry settings that infrequently change.
   */
  public static        boolean              updateSettings               = true;

  /**
   * Start the ctrl timer to measure cycle time, independent of periodic loops.
   */
  public static void startCtrlCycle()
  {
    if (ctrlTimer.isRunning())
    {
      ctrlTimer.reset();
    } else
    {
      ctrlTimer.start();
    }
  }

  /**
   * Update the Control cycle time.
   */
  public static void endCtrlCycle()
  {
    if (DriverStation.isTeleopEnabled() || DriverStation.isAutonomousEnabled() || DriverStation.isTestEnabled())
    {
      // 100ms per module on initialization is normal
      ctrlCycleTime.set(ctrlTimer.get() * 1000);
    }
    ctrlTimer.reset();
  }

  /**
   * Start the odom cycle timer to calculate how long each odom took. Independent of periodic loops.
   */
  public static void startOdomCycle()
  {
    if (odomTimer.isRunning())
    {

      odomTimer.reset();
    } else
    {
      odomTimer.start();

    }
  }

  /**
   * Update the odom cycle time.
   */
  public static void endOdomCycle()
  {
    if (DriverStation.isTeleopEnabled() || DriverStation.isAutonomousEnabled() || DriverStation.isTestEnabled())
    {
      odomCycleTime.set(odomTimer.get() * 1000);
    }
    odomTimer.reset();
  }

  /**
   * Update only the settings that infrequently or never change.
   */
  public static void updateSwerveTelemetrySettings()
  {
    if (updateSettings)
    {
      updateSettings = false;
      wheelLocationsArrayPublisher.set(wheelLocations);
      maxSpeedPublisher.set(maxSpeed);
      rotationUnitPublisher.set(rotationUnit);
      sizeLeftRightPublisher.set(sizeLeftRight);
      sizeFrontBackPublisher.set(sizeFrontBack);
      forwardDirectionPublisher.set(forwardDirection);
    }
  }

  /**
   * Upload data to smartdashboard
   */
  public static void updateData()
  {
    if (updateSettings)
    {
      updateSwerveTelemetrySettings();
    }
    measuredChassisSpeeds[0] = measuredChassisSpeedsObj.vxMetersPerSecond;
    measuredChassisSpeeds[1] = measuredChassisSpeedsObj.vxMetersPerSecond;
    measuredChassisSpeeds[2] = Math.toDegrees(measuredChassisSpeedsObj.omegaRadiansPerSecond);

    desiredChassisSpeeds[0] = desiredChassisSpeedsObj.vxMetersPerSecond;
    desiredChassisSpeeds[1] = desiredChassisSpeedsObj.vyMetersPerSecond;
    desiredChassisSpeeds[2] = Math.toDegrees(desiredChassisSpeedsObj.omegaRadiansPerSecond);

    robotRotation = robotRotationObj.getDegrees();

    for (int i = 0; i < measuredStatesObj.length; i++)
    {
      SwerveModuleState state = measuredStatesObj[i];
      if (state != null)
      {
        measuredStates[i * 2] = state.angle.getDegrees();
        measuredStates[i * 2 + 1] = state.speedMetersPerSecond;
      }
    }

    for (int i = 0; i < desiredStatesObj.length; i++)
    {
      SwerveModuleState state = desiredStatesObj[i];
      if (state != null)
      {
        desiredStates[i * 2] = state.angle.getDegrees();
        desiredStates[i * 2 + 1] = state.speedMetersPerSecond;
      }
    }

    moduleCountPublisher.set(moduleCount);
    measuredStatesArrayPublisher.set(measuredStates);
    desiredStatesArrayPublisher.set(desiredStates);
    robotRotationPublisher.set(robotRotation);
    maxAngularVelocityPublisher.set(maxAngularVelocity);

    measuredChassisSpeedsArrayPublisher.set(measuredChassisSpeeds);
    desiredChassisSpeedsArrayPublisher.set(desiredChassisSpeeds);

    desiredStatesStruct.set(desiredStatesObj);
    measuredStatesStruct.set(measuredStatesObj);
    desiredChassisSpeedsStruct.set(desiredChassisSpeedsObj);
    measuredChassisSpeedsStruct.set(measuredChassisSpeedsObj);
    robotRotationStruct.set(robotRotationObj);
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
     * Medium telemetry data, swerve directory
     */
    INFO,
    /**
     * Info level + field info
     */
    POSE,
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

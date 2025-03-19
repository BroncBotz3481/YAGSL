package swervelib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import swervelib.math.SwerveMath;

/**
 * Helper class to easily transform Controller inputs into workable Chassis speeds. Intended to easily create an
 * interface that generates {@link ChassisSpeeds} from {@link XboxController}
 * <p>
 * <br /> Inspired by SciBorgs FRC 1155. <br /> Example:
 * <pre>
 * {@code
 *   XboxController driverXbox = new XboxController(0);
 *
 *   SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
 *                                                                 () -> driverXbox.getLeftY() * -1,
 *                                                                 () -> driverXbox.getLeftX() * -1) // Axis which give the desired translational angle and speed.
 *                                                             .withControllerRotationAxis(driverXbox::getRightX) // Axis which give the desired angular velocity.
 *                                                             .deadband(0.01)                  // Controller deadband
 *                                                             .scaleTranslation(0.8)           // Scaled controller translation axis
 *                                                             .allianceRelativeControl(true);  // Alliance relative controls.
 *
 *   SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()  // Copy the stream so further changes do not affect driveAngularVelocity
 *                                                            .withControllerHeadingAxis(driverXbox::getRightX,
 *                                                                                       driverXbox::getRightY) // Axis which give the desired heading angle using trigonometry.
 *                                                            .headingWhile(true); // Enable heading based control.
 * }
 * </pre>
 */
public class SwerveInputStream implements Supplier<ChassisSpeeds>
{

  /**
   * Translation suppliers.
   */
  private final DoubleSupplier                  controllerTranslationX;
  /**
   * Translational supplier.
   */
  private final DoubleSupplier                  controllerTranslationY;
  /**
   * {@link SwerveDrive} object for transformations.
   */
  private final SwerveDrive                     swerveDrive;
  /**
   * Rotation supplier as angular velocity.
   */
  private Optional<DoubleSupplier>        controllerOmega                     = Optional.empty();
  /**
   * Controller supplier as heading.
   */
  private Optional<DoubleSupplier>        controllerHeadingX                  = Optional.empty();
  /**
   * Controller supplier as heading.
   */
  private Optional<DoubleSupplier>        controllerHeadingY                  = Optional.empty();
  /**
   * Axis deadband for the controller.
   */
  private Optional<Double>                axisDeadband                        = Optional.empty();
  /**
   * Translational axis scalar value, should be between (0, 1].
   */
  private Optional<Double>                translationAxisScale                = Optional.empty();
  /**
   * Angular velocity axis scalar value, should be between (0, 1]
   */
  private Optional<Double>                omegaAxisScale                      = Optional.empty();
  /**
   * Target to aim at.
   */
  private Optional<Pose2d>                aimTarget                           = Optional.empty();
  /**
   * Target {@link Supplier<Pose2d>} to drive towards when driveToPose is enabled.
   */
  private Optional<Supplier<Pose2d>>      driveToPose                         = Optional.empty();
  /**
   * {@link ProfiledPIDController} for the translation while driving to a pose. Units are m/s
   */
  private Optional<ProfiledPIDController> driveToPoseTranslationPIDController = Optional.empty();
  /**
   * {@link ProfiledPIDController} for the Rotational axis while driving to a pose. Units are m/s
   */
  private Optional<ProfiledPIDController> driveToPoseOmegaPIDController       = Optional.empty();
  /**
   * Output {@link ChassisSpeeds} based on heading while this is True.
   */
  private Optional<BooleanSupplier>       headingEnabled                      = Optional.empty();
  /**
   * Locked heading for {@link SwerveInputMode#TRANSLATION_ONLY}
   */
  private Optional<Rotation2d>            lockedHeading                       = Optional.empty();
  /**
   * Output {@link ChassisSpeeds} based on aim while this is True.
   */
  private Optional<BooleanSupplier>       aimEnabled                          = Optional.empty();
  /**
   * Output {@link ChassisSpeeds} to move to a specific {@link Pose2d}.
   */
  private Optional<BooleanSupplier>       driveToPoseEnabled                  = Optional.empty();
  /**
   * Maintain current heading and drive without rotating, ideally.
   */
  private Optional<BooleanSupplier>       translationOnlyEnabled              = Optional.empty();
  /**
   * Cube the translation magnitude from the controller.
   */
  private Optional<BooleanSupplier>       translationCube                     = Optional.empty();
  /**
   * Cube the angular velocity axis from the controller.
   */
  private Optional<BooleanSupplier>       omegaCube                           = Optional.empty();
  /**
   * Robot relative oriented output expected.
   */
  private Optional<BooleanSupplier>       robotRelative                       = Optional.empty();
  /**
   * Field oriented chassis output is relative to your current alliance.
   */
  private Optional<BooleanSupplier>       allianceRelative                    = Optional.empty();
  /**
   * Heading offset enable state.
   */
  private Optional<BooleanSupplier>       translationHeadingOffsetEnabled     = Optional.empty();
  /**
   * Heading offset to apply during heading based control.
   */
  private Optional<Rotation2d>            translationHeadingOffset            = Optional.empty();
  /**
   * {@link SwerveController} for simple control over heading.
   */
  private SwerveController                swerveController                    = null;
  /**
   * Current {@link SwerveInputMode} to use.
   */
  private SwerveInputMode                 currentMode                         = SwerveInputMode.ANGULAR_VELOCITY;


  /**
   * Create a {@link SwerveInputStream} for an easy way to generate {@link ChassisSpeeds} from a driver controller.
   *
   * @param drive {@link SwerveDrive} object for transformation.
   * @param x     Translation X input in range of [-1, 1]
   * @param y     Translation Y input in range of [-1, 1]
   */
  private SwerveInputStream(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y)
  {
    controllerTranslationX = x;
    controllerTranslationY = y;
    swerveDrive = drive;
  }

  /**
   * Create a {@link SwerveInputStream} for an easy way to generate {@link ChassisSpeeds} from a driver controller.
   *
   * @param drive {@link SwerveDrive} object for transformation.
   * @param x     Translation X input in range of [-1, 1]
   * @param y     Translation Y input in range of [-1, 1]
   * @param rot   Rotation input in range of [-1, 1]
   */
  public SwerveInputStream(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot)
  {
    this(drive, x, y);
    controllerOmega = Optional.of(rot);
  }

  /**
   * Create a {@link SwerveInputStream} for an easy way to generate {@link ChassisSpeeds} from a driver controller.
   *
   * @param drive    {@link SwerveDrive} object for transformation.
   * @param x        Translation X input in range of [-1, 1]
   * @param y        Translation Y input in range of [-1, 1]
   * @param headingX Heading X input in range of [-1, 1]
   * @param headingY Heading Y input in range of [-1, 1]
   */
  public SwerveInputStream(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier headingX,
                           DoubleSupplier headingY)
  {
    this(drive, x, y);
    controllerHeadingX = Optional.of(headingX);
    controllerHeadingY = Optional.of(headingY);
  }

  /**
   * Create basic {@link SwerveInputStream} without any rotation components.
   *
   * @param drive {@link SwerveDrive} object for transformation.
   * @param x     {@link DoubleSupplier} of the translation X axis of the controller joystick to use.
   * @param y     {@link DoubleSupplier} of the translation X axis of the controller joystick to use.
   * @return {@link SwerveInputStream} to use as you see fit.
   */
  public static SwerveInputStream of(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y)
  {
    return new SwerveInputStream(drive, x, y);
  }

  /**
   * Copy the {@link SwerveInputStream} object.
   *
   * @return Clone of current {@link SwerveInputStream}
   */
  public SwerveInputStream copy()
  {
    SwerveInputStream newStream = new SwerveInputStream(swerveDrive, controllerTranslationX, controllerTranslationY);
    newStream.controllerOmega = controllerOmega;
    newStream.controllerHeadingX = controllerHeadingX;
    newStream.controllerHeadingY = controllerHeadingY;
    newStream.axisDeadband = axisDeadband;
    newStream.translationAxisScale = translationAxisScale;
    newStream.omegaAxisScale = omegaAxisScale;
    newStream.driveToPose = driveToPose;
    newStream.driveToPoseTranslationPIDController = driveToPoseTranslationPIDController;
    newStream.driveToPoseOmegaPIDController = driveToPoseOmegaPIDController;
    newStream.aimTarget = aimTarget;
    newStream.headingEnabled = headingEnabled;
    newStream.aimEnabled = aimEnabled;
    newStream.driveToPoseEnabled = driveToPoseEnabled;
    newStream.currentMode = currentMode;
    newStream.translationOnlyEnabled = translationOnlyEnabled;
    newStream.lockedHeading = lockedHeading;
    newStream.swerveController = swerveController;
    newStream.omegaCube = omegaCube;
    newStream.translationCube = translationCube;
    newStream.robotRelative = robotRelative;
    newStream.allianceRelative = allianceRelative;
    newStream.translationHeadingOffsetEnabled = translationHeadingOffsetEnabled;
    newStream.translationHeadingOffset = translationHeadingOffset;
    return newStream;
  }

  /**
   * Set the stream to output robot relative {@link ChassisSpeeds}
   *
   * @param enabled Robot-Relative {@link ChassisSpeeds} output.
   * @return self
   */
  public SwerveInputStream robotRelative(BooleanSupplier enabled)
  {
    robotRelative = Optional.of(enabled);
    return this;
  }

  /**
   * Set the stream to output robot relative {@link ChassisSpeeds}
   *
   * @param enabled Robot-Relative {@link ChassisSpeeds} output.
   * @return self
   */
  public SwerveInputStream robotRelative(boolean enabled)
  {
    robotRelative = enabled ? Optional.of(() -> enabled) : Optional.empty();
    return this;
  }

  /**
   * Drive to a given pose with the provided {@link ProfiledPIDController}s
   *
   * @param pose               {@link Supplier<Pose2d>} for ease of use.
   * @param xPIDController     PID controller for the translational axis, units are m/s.
   * @param omegaPIDController PID Controller for rotational axis, units are rad/s.
   * @return self
   */
  public SwerveInputStream driveToPose(Supplier<Pose2d> pose, ProfiledPIDController xPIDController,
                                       ProfiledPIDController omegaPIDController)
  {
    omegaPIDController.reset(swerveDrive.getPose().getRotation().getRadians());
    xPIDController.reset(swerveDrive.getPose().getTranslation().getDistance(pose.get().getTranslation()));
    omegaPIDController.enableContinuousInput(-Math.PI, Math.PI);
    xPIDController.setGoal(new State(0, 0));
    driveToPose = Optional.of(pose);
    driveToPoseTranslationPIDController = Optional.of(xPIDController);
    driveToPoseOmegaPIDController = Optional.of(omegaPIDController);
    return this;
  }


  /**
   * Enable driving to the target pose.
   *
   * @param enabled Enable state of drive to pose.
   * @return self.
   */
  public SwerveInputStream driveToPoseEnabled(BooleanSupplier enabled)
  {
    driveToPoseEnabled = Optional.of(enabled);
    return this;
  }

  /**
   * Enable driving to the target pose.
   *
   * @param enabled Enable state of drive to pose.
   * @return self.
   */
  public SwerveInputStream driveToPoseEnabled(boolean enabled)
  {
    driveToPoseEnabled = enabled ? Optional.of(() -> enabled) : Optional.empty();
    Pose2d swervePose = swerveDrive.getPose();
//    driveToPoseXPIDController.ifPresent(profiledPIDController -> profiledPIDController.reset(swervePose.getX()));
//    driveToPoseYPIDController.ifPresent(profiledPIDController -> profiledPIDController.reset(swervePose.getY()));
//    driveToPoseOmegaPIDController.ifPresent(profiledPIDController -> profiledPIDController.reset(swervePose.getRotation()
//                                                                                                           .getRadians()));
    return this;
  }


  /**
   * Heading offset enabled boolean supplier.
   *
   * @param enabled Enable state
   * @return self
   */
  public SwerveInputStream translationHeadingOffset(BooleanSupplier enabled)
  {
    translationHeadingOffsetEnabled = Optional.of(enabled);
    return this;
  }

  /**
   * Heading offset enable
   *
   * @param enabled Enable state
   * @return self
   */
  public SwerveInputStream translationHeadingOffset(boolean enabled)
  {
    translationHeadingOffsetEnabled = enabled ? Optional.of(() -> enabled) : Optional.empty();
    return this;
  }

  /**
   * Set the heading offset angle.
   *
   * @param angle {@link Rotation2d} offset to apply
   * @return self
   */
  public SwerveInputStream translationHeadingOffset(Rotation2d angle)
  {
    translationHeadingOffset = Optional.of(angle);
    return this;
  }

  /**
   * Modify the output {@link ChassisSpeeds} so that it is always relative to your alliance.
   *
   * @param enabled Alliance aware {@link ChassisSpeeds} output.
   * @return self
   */
  public SwerveInputStream allianceRelativeControl(BooleanSupplier enabled)
  {
    allianceRelative = Optional.of(enabled);
    return this;
  }

  /**
   * Modify the output {@link ChassisSpeeds} so that it is always relative to your alliance.
   *
   * @param enabled Alliance aware {@link ChassisSpeeds} output.
   * @return self
   */
  public SwerveInputStream allianceRelativeControl(boolean enabled)
  {
    allianceRelative = enabled ? Optional.of(() -> enabled) : Optional.empty();
    return this;
  }

  /**
   * Cube the angular velocity controller axis for a non-linear controls scheme.
   *
   * @param enabled Enabled state for the stream.
   * @return self.
   */
  public SwerveInputStream cubeRotationControllerAxis(BooleanSupplier enabled)
  {
    omegaCube = Optional.of(enabled);
    return this;
  }

  /**
   * Cube the angular velocity controller axis for a non-linear controls scheme.
   *
   * @param enabled Enabled state for the stream.
   * @return self.
   */
  public SwerveInputStream cubeRotationControllerAxis(boolean enabled)
  {
    omegaCube = Optional.of(() -> enabled);
    return this;
  }

  /**
   * Cube the translation axis magnitude for a non-linear control scheme.
   *
   * @param enabled Enabled state for the stream
   * @return self
   */
  public SwerveInputStream cubeTranslationControllerAxis(BooleanSupplier enabled)
  {
    translationOnlyEnabled = Optional.of(enabled);
    return this;
  }

  /**
   * Cube the translation axis magnitude for a non-linear control scheme
   *
   * @param enabled Enabled state for the stream
   * @return self
   */
  public SwerveInputStream cubeTranslationControllerAxis(boolean enabled)
  {
    translationCube = enabled ? Optional.of(() -> enabled) : Optional.empty();
    return this;
  }

  /**
   * Add a rotation axis for Angular Velocity control
   *
   * @param rot Rotation axis with values from [-1, 1]
   * @return self
   */
  public SwerveInputStream withControllerRotationAxis(DoubleSupplier rot)
  {
    controllerOmega = Optional.of(rot);
    return this;
  }

  /**
   * Add heading axis for Heading based control.
   *
   * @param headingX Heading X axis with values from [-1, 1]
   * @param headingY Heading Y axis with values from [-1, 1]
   * @return self
   */
  public SwerveInputStream withControllerHeadingAxis(DoubleSupplier headingX, DoubleSupplier headingY)
  {
    controllerHeadingX = Optional.of(headingX);
    controllerHeadingY = Optional.of(headingY);
    return this;
  }

  /**
   * Set a deadband for all controller axis.
   *
   * @param deadband Deadband to set, should be between [0, 1)
   * @return self
   */
  public SwerveInputStream deadband(double deadband)
  {
    axisDeadband = deadband == 0 ? Optional.empty() : Optional.of(deadband);
    return this;
  }

  /**
   * Scale the translation axis for {@link SwerveInputStream} by a constant scalar value.
   *
   * @param scaleTranslation Translation axis scalar value. (0, 1]
   * @return this
   */
  public SwerveInputStream scaleTranslation(double scaleTranslation)
  {
    translationAxisScale = scaleTranslation == 0 ? Optional.empty() : Optional.of(scaleTranslation);
    return this;
  }

  /**
   * Scale the rotation axis input for {@link SwerveInputStream} to reduce the range in which they operate.
   *
   * @param scaleRotation Angular velocity axis scalar value. (0, 1]
   * @return this
   */
  public SwerveInputStream scaleRotation(double scaleRotation)
  {
    omegaAxisScale = scaleRotation == 0 ? Optional.empty() : Optional.of(scaleRotation);
    return this;
  }

  /**
   * Output {@link ChassisSpeeds} based on heading while the supplier is True.
   *
   * @param trigger Supplier to use.
   * @return this.
   */
  public SwerveInputStream headingWhile(BooleanSupplier trigger)
  {
    headingEnabled = Optional.of(trigger);
    return this;
  }

  /**
   * Set the heading enable state.
   *
   * @param headingState Heading enabled state.
   * @return this
   */
  public SwerveInputStream headingWhile(boolean headingState)
  {
    if (headingState)
    {
      headingEnabled = Optional.of(() -> true);
    } else
    {
      headingEnabled = Optional.empty();
    }
    return this;
  }

  /**
   * Aim the {@link SwerveDrive} at this pose while driving.
   *
   * @param aimTarget {@link Pose2d} to point at.
   * @return this
   */
  public SwerveInputStream aim(Pose2d aimTarget)
  {
    this.aimTarget = aimTarget.equals(Pose2d.kZero) ? Optional.empty() : Optional.of(aimTarget);
    return this;
  }

  /**
   * Enable aiming while the trigger is true.
   *
   * @param trigger When True will enable aiming at the current target.
   * @return this.
   */
  public SwerveInputStream aimWhile(BooleanSupplier trigger)
  {
    aimEnabled = Optional.of(trigger);
    return this;
  }

  /**
   * Enable aiming while the trigger is true.
   *
   * @param trigger When True will enable aiming at the current target.
   * @return this.
   */
  public SwerveInputStream aimWhile(boolean trigger)
  {
    if (trigger)
    {
      aimEnabled = Optional.of(() -> true);
    } else
    {
      aimEnabled = Optional.empty();
    }
    return this;
  }

  /**
   * Enable locking of rotation and only translating, overrides everything.
   *
   * @param trigger Translation only while returns true.
   * @return this
   */
  public SwerveInputStream translationOnlyWhile(BooleanSupplier trigger)
  {
    translationOnlyEnabled = Optional.of(trigger);
    return this;
  }

  /**
   * Enable locking of rotation and only translating, overrides everything.
   *
   * @param translationState Translation only if true.
   * @return this
   */
  public SwerveInputStream translationOnlyWhile(boolean translationState)
  {
    if (translationState)
    {
      translationOnlyEnabled = Optional.of(() -> true);
    } else
    {
      translationOnlyEnabled = Optional.empty();
    }
    return this;
  }

  /**
   * Find {@link SwerveInputMode} based off existing parameters of the {@link SwerveInputStream}
   *
   * @return The calculated {@link SwerveInputMode}, defaults to {@link SwerveInputMode#ANGULAR_VELOCITY}.
   */
  private SwerveInputMode findMode()
  {
    if (driveToPoseEnabled.isPresent() && driveToPoseEnabled.get().getAsBoolean())
    {
      if (driveToPose.isPresent())
      {
        if (driveToPoseOmegaPIDController.isPresent() && driveToPoseTranslationPIDController.isPresent())
        {
          return SwerveInputMode.DRIVE_TO_POSE;
        }
        System.out.println("Drive to pose present");
        DriverStation.reportError("Drive to pose not supplied with pid controllers.", false);
      }
      DriverStation.reportError("Drive to pose enabled without supplier present.", false);
    } else if (translationOnlyEnabled.isPresent() && translationOnlyEnabled.get().getAsBoolean())
    {
      return SwerveInputMode.TRANSLATION_ONLY;
    } else if (aimEnabled.isPresent() && aimEnabled.get().getAsBoolean())
    {
      if (aimTarget.isPresent())
      {
        return SwerveInputMode.AIM;
      } else
      {
        DriverStation.reportError(
            "Attempting to enter AIM mode without target, please use SwerveInputStream.aim() to select a target first!",
            false);
      }
    } else if (headingEnabled.isPresent() && headingEnabled.get().getAsBoolean())
    {
      if (controllerHeadingX.isPresent() && controllerHeadingY.isPresent())
      {
        return SwerveInputMode.HEADING;
      } else
      {
        DriverStation.reportError(
            "Attempting to enter HEADING mode without heading axis, please use SwerveInputStream.withControllerHeadingAxis to add heading axis!",
            false);
      }
    } else if (controllerOmega.isEmpty())
    {
      DriverStation.reportError(
          "Attempting to enter ANGULAR_VELOCITY mode without a rotation axis, please use SwerveInputStream.withControllerRotationAxis to add angular velocity axis!",
          false);
      return SwerveInputMode.TRANSLATION_ONLY;
    }
    return SwerveInputMode.ANGULAR_VELOCITY;
  }

  /**
   * Transition smoothly from one mode to another.
   *
   * @param newMode New mode to transition too.
   */
  private void transitionMode(SwerveInputMode newMode)
  {
    // Handle removing of current mode.
    switch (currentMode)
    {
      case TRANSLATION_ONLY ->
      {
        lockedHeading = Optional.empty();
        break;
      }
      case ANGULAR_VELOCITY, HEADING, AIM ->
      {
        // Do nothing
        break;
      }
      case DRIVE_TO_POSE ->
      {
        break;
      }
    }

    // Transitioning to new mode
    switch (newMode)
    {
      case TRANSLATION_ONLY ->
      {
        lockedHeading = Optional.of(swerveDrive.getOdometryHeading());
        break;
      }
      case ANGULAR_VELOCITY ->
      {
        if (swerveDrive.headingCorrection)
        {
          swerveDrive.setHeadingCorrection(false);
        }
        break;
      }
      case HEADING, AIM ->
      {
        // Do nothing
        break;
      }
      case DRIVE_TO_POSE ->
      {
        if (swerveDrive.headingCorrection)
        {
          swerveDrive.setHeadingCorrection(false);
        }
      }
    }
  }

  /**
   * Apply the deadband if it exists.
   *
   * @param axisValue Axis value to apply the deadband too.
   * @return axis value with deadband, else axis value straight.
   */
  private double applyDeadband(double axisValue)
  {
    if (axisDeadband.isPresent())
    {
      return MathUtil.applyDeadband(axisValue, axisDeadband.get());
    }
    return axisValue;
  }

  /**
   * Apply the scalar value if it exists.
   *
   * @param axisValue Axis value to apply teh scalar too.
   * @return Axis value scaled by scalar value.
   */
  private double applyRotationalScalar(double axisValue)
  {
    if (omegaAxisScale.isPresent())
    {
      return axisValue * omegaAxisScale.get();
    }
    return axisValue;
  }

  /**
   * Scale the translational axis by the {@link SwerveInputStream#translationAxisScale} if it exists.
   *
   * @param xAxis X axis to scale.
   * @param yAxis Y axis to scale.
   * @return Scaled {@link Translation2d}
   */
  private Translation2d applyTranslationScalar(double xAxis, double yAxis)
  {
    if (translationAxisScale.isPresent())

    {
      return SwerveMath.scaleTranslation(new Translation2d(xAxis, yAxis),
                                         translationAxisScale.get());
    }
    return new Translation2d(xAxis, yAxis);
  }

  /**
   * Apply the cube transformation on the given {@link Translation2d}
   *
   * @param translation {@link Translation2d} representing controller input
   * @return Cubed {@link Translation2d} if the {@link SwerveInputStream#translationCube} is present.
   */
  private Translation2d applyTranslationCube(Translation2d translation)
  {
    if (translationCube.isPresent() && translationCube.get().getAsBoolean())
    {
      return SwerveMath.cubeTranslation(translation);
    }
    return translation;
  }

  /**
   * Apply the cube transformation on the given rotation controller axis
   *
   * @param rotationAxis Rotation controller axis to cube.
   * @return Cubed axis value if the {@link SwerveInputStream#omegaCube} is present.
   */
  private double applyOmegaCube(double rotationAxis)
  {
    if (omegaCube.isPresent() && omegaCube.get().getAsBoolean())
    {
      return Math.pow(rotationAxis, 3);
    }
    return rotationAxis;
  }

  /**
   * Change {@link ChassisSpeeds} from robot relative if enabled.
   *
   * @param fieldRelativeSpeeds Field or robot relative speeds to translate into robot-relative speeds.
   * @return Field relative {@link ChassisSpeeds}.
   */
  private ChassisSpeeds applyRobotRelativeTranslation(ChassisSpeeds fieldRelativeSpeeds)
  {
    if (robotRelative.isPresent() && robotRelative.get().getAsBoolean())
    {
      return ChassisSpeeds.fromRobotRelativeSpeeds(fieldRelativeSpeeds, swerveDrive.getOdometryHeading());
    }
    return fieldRelativeSpeeds;
  }

  /**
   * Apply alliance aware translation which flips the {@link Translation2d} if the robot is on the Blue alliance.
   *
   * @param fieldRelativeTranslation Field-relative {@link Translation2d} to flip.
   * @return Alliance-oriented {@link Translation2d}
   */
  private Translation2d applyAllianceAwareTranslation(Translation2d fieldRelativeTranslation)
  {
    if (allianceRelative.isPresent() && allianceRelative.get().getAsBoolean())
    {
      if (robotRelative.isPresent() && robotRelative.get().getAsBoolean())
      {
        if (driveToPoseEnabled.isPresent() && driveToPoseEnabled.get().getAsBoolean())
        {
          return fieldRelativeTranslation;
        }
        throw new RuntimeException("Cannot use robot oriented control with Alliance aware movement!");
      }
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
      {
        return fieldRelativeTranslation.rotateBy(Rotation2d.k180deg);
      }
    }
    return fieldRelativeTranslation;
  }

  /**
   * Adds offset to translation if one is set.
   *
   * @param speeds {@link ChassisSpeeds} to offset
   * @return Offsetted {@link ChassisSpeeds}
   */
  private ChassisSpeeds applyTranslationHeadingOffset(ChassisSpeeds speeds)
  {
    if (translationHeadingOffsetEnabled.isPresent() && translationHeadingOffsetEnabled.get().getAsBoolean())
    {
      if (translationHeadingOffset.isPresent())
      {
        Translation2d speedsTranslation = new Translation2d(speeds.vxMetersPerSecond,
                                                            speeds.vyMetersPerSecond).rotateBy(translationHeadingOffset.get());
        return new ChassisSpeeds(speedsTranslation.getX(), speedsTranslation.getY(), speeds.omegaRadiansPerSecond);
      }
    }
    return speeds;
  }

  /**
   * When the {@link SwerveInputStream} is in {@link SwerveInputMode#DRIVE_TO_POSE} this function will return if the
   * robot is at the desired pose within the defined tolerance.
   *
   * @param toleranceMeters Tolerance in meters.
   * @return At target pose, true if current mode is not {@link SwerveInputMode#DRIVE_TO_POSE} and no pose supplier has
   * been given.
   */
  public boolean atTargetPose(double toleranceMeters)
  {
    if (currentMode != SwerveInputMode.DRIVE_TO_POSE)
    {
      DriverStation.reportError("SwerveInputStream.atTargetPose called while not set to DriveToPose.", false);
      if (!driveToPose.isPresent())
      {
        return true;
      }
    }
    if (driveToPose.isPresent())
    {
      Pose2d targetPose = driveToPose.get().get();
      return swerveDrive.getPose().getTranslation().getDistance(targetPose.getTranslation()) <= toleranceMeters;
    }
    return true;
  }

  /**
   * Gets a {@link ChassisSpeeds}
   *
   * @return {@link ChassisSpeeds}
   */
  @Override
  public ChassisSpeeds get()
  {
    double maximumChassisVelocity = swerveDrive.getMaximumChassisVelocity();
    Translation2d scaledTranslation = applyTranslationScalar(applyDeadband(controllerTranslationX.getAsDouble()),
                                                             applyDeadband(controllerTranslationY.getAsDouble()));
    scaledTranslation = applyTranslationCube(scaledTranslation);
    scaledTranslation = applyAllianceAwareTranslation(scaledTranslation);

    double        vxMetersPerSecond     = scaledTranslation.getX() * maximumChassisVelocity;
    double        vyMetersPerSecond     = scaledTranslation.getY() * maximumChassisVelocity;
    double        omegaRadiansPerSecond = 0;
    ChassisSpeeds speeds                = new ChassisSpeeds();

    SwerveInputMode newMode = findMode();
    // Handle transitions here.
    if (currentMode != newMode)
    {
      transitionMode(newMode);
    }
    if (swerveController == null)
    {
      swerveController = swerveDrive.getSwerveController();
    }
    switch (newMode)
    {
      case TRANSLATION_ONLY ->
      {
        omegaRadiansPerSecond = swerveController.headingCalculate(swerveDrive.getOdometryHeading().getRadians(),
                                                                  lockedHeading.get().getRadians());
        speeds = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        break;
      }
      case ANGULAR_VELOCITY ->
      {
        omegaRadiansPerSecond = applyOmegaCube(applyRotationalScalar(applyDeadband(controllerOmega.get()
                                                                                                  .getAsDouble()))) *
                                swerveDrive.getMaximumChassisAngularVelocity();
        speeds = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        break;
      }
      case HEADING ->
      {
        omegaRadiansPerSecond = swerveController.headingCalculate(swerveDrive.getOdometryHeading().getRadians(),
                                                                  Rotation2d.fromRadians(
                                                                                swerveController.getJoystickAngle(
                                                                                    controllerHeadingX.get()
                                                                                                      .getAsDouble(),
                                                                                    controllerHeadingY.get()
                                                                                                      .getAsDouble()))
                                                                            .getRadians());

        // Prevent rotation if controller heading inputs are not past axisDeadband
        if (Math.abs(controllerHeadingX.get().getAsDouble()) + Math.abs(controllerHeadingY.get().getAsDouble()) <
            axisDeadband.get())
        {
          omegaRadiansPerSecond = 0;
        }
        speeds = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        break;
      }
      case AIM ->
      {
        Rotation2d    currentHeading = swerveDrive.getOdometryHeading();
        Translation2d relativeTrl    = aimTarget.get().relativeTo(swerveDrive.getPose()).getTranslation();
        Rotation2d    target         = new Rotation2d(relativeTrl.getX(), relativeTrl.getY()).plus(currentHeading);
        omegaRadiansPerSecond = swerveController.headingCalculate(currentHeading.getRadians(), target.getRadians());
        speeds = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        break;
      }
      case DRIVE_TO_POSE ->
      {
        // Written by team 8865!
        ProfiledPIDController translationPIDController = driveToPoseTranslationPIDController.get();
        ProfiledPIDController rotationPIDController    = driveToPoseOmegaPIDController.get();
        Pose2d                swervePoseSetpoint       = driveToPose.get().get();
        Pose2d                robotPose                = swerveDrive.getPose();
        Vector<N2>            robotVec                 = robotPose.getTranslation().toVector();
        Vector<N2> targetPoseRelativeToRobotPose = swervePoseSetpoint.getTranslation().toVector().minus(
            robotVec);
        double distanceFromTarget = targetPoseRelativeToRobotPose.norm();

        Vector<N2> traversalVector = new Vector(Nat.N2());
        traversalVector.set(0, 0, targetPoseRelativeToRobotPose.get(0, 0));
        traversalVector.set(1, 0, targetPoseRelativeToRobotPose.get(1, 0));
        traversalVector = traversalVector.unit()
                                         .times(-translationPIDController.calculate(distanceFromTarget, 0));

        Vector<N2> robotForwardVec = robotPose.transformBy(new Transform2d(1, 0, new Rotation2d())).getTranslation()
                                              .toVector().minus(robotVec);
        Vector<N2> robotLateralVec = robotPose.transformBy(new Transform2d(0, 1, new Rotation2d())).getTranslation()
                                              .toVector().minus(robotVec);

        currentMode = newMode;
        speeds = ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(
                                                           robotForwardVec.norm() * traversalVector.dot(robotForwardVec),
                                                           robotLateralVec.norm() * traversalVector.dot(robotLateralVec),
                                                           rotationPIDController.calculate(robotPose.getRotation().getRadians(),
                                                                                           swervePoseSetpoint.getRotation().getRadians())),
                                                       swerveDrive.getOdometryHeading());
        double lerpDistance = robotPose.getTranslation().plus(new Translation2d(speeds.vxMetersPerSecond,
                                                                                vyMetersPerSecond).times(0.02))
                                       .getDistance(swervePoseSetpoint.getTranslation());
        // Filter out incorrect ChassisSpeeds.
        if (lerpDistance > distanceFromTarget)
        {
          speeds = new ChassisSpeeds(0, 0, 0);
        }

        return speeds;
      }
    }

    currentMode = newMode;

    return applyTranslationHeadingOffset(applyRobotRelativeTranslation(speeds));
  }

  /**
   * Drive modes to keep track of.
   */
  enum SwerveInputMode
  {
    /**
     * Translation only mode, does not allow for rotation and maintains current heading.
     */
    TRANSLATION_ONLY,
    /**
     * Output based off angular velocity
     */
    ANGULAR_VELOCITY,
    /**
     * Output based off of heading.
     */
    HEADING,
    /**
     * Output based off of targeting.
     */
    AIM,
    /**
     * Drive to a target pose.
     */
    DRIVE_TO_POSE
  }
}

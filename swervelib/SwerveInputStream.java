package swervelib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
  private final DoubleSupplier            controllerTranslationX;
  /**
   * Translational supplier.
   */
  private final DoubleSupplier            controllerTranslationY;
  /**
   * {@link SwerveDrive} object for transformations.
   */
  private final SwerveDrive               swerveDrive;
  /**
   * Rotation supplier as angular velocity.
   */
  private Optional<DoubleSupplier>  controllerOmega      = Optional.empty();
  /**
   * Controller supplier as heading.
   */
  private       Optional<DoubleSupplier>  controllerHeadingX     = Optional.empty();
  /**
   * Controller supplier as heading.
   */
  private       Optional<DoubleSupplier>  controllerHeadingY     = Optional.empty();
  /**
   * Axis deadband for the controller.
   */
  private Optional<Double>          axisDeadband         = Optional.empty();
  /**
   * Translational axis scalar value, should be between (0, 1].
   */
  private Optional<Double>          translationAxisScale = Optional.empty();
  /**
   * Angular velocity axis scalar value, should be between (0, 1]
   */
  private Optional<Double>          omegaAxisScale       = Optional.empty();
  /**
   * Target to aim at.
   */
  private Optional<Pose2d>          aimTarget            = Optional.empty();
  /**
   * Output {@link ChassisSpeeds} based on heading while this is True.
   */
  private       Optional<BooleanSupplier> headingEnabled         = Optional.empty();
  /**
   * Locked heading for {@link SwerveInputMode#TRANSLATION_ONLY}
   */
  private       Optional<Rotation2d>      lockedHeading          = Optional.empty();
  /**
   * Output {@link ChassisSpeeds} based on aim while this is True.
   */
  private       Optional<BooleanSupplier> aimEnabled             = Optional.empty();
  /**
   * Maintain current heading and drive without rotating, ideally.
   */
  private       Optional<BooleanSupplier> translationOnlyEnabled = Optional.empty();
  /**
   * Cube the translation magnitude from the controller.
   */
  private Optional<BooleanSupplier> translationCube      = Optional.empty();
  /**
   * Cube the angular velocity axis from the controller.
   */
  private Optional<BooleanSupplier> omegaCube            = Optional.empty();
  /**
   * Robot relative oriented output expected.
   */
  private Optional<BooleanSupplier> robotRelative        = Optional.empty();
  /**
   * Field oriented chassis output is relative to your current alliance.
   */
  private Optional<BooleanSupplier> allianceRelative     = Optional.empty();
  /**
   * {@link SwerveController} for simple control over heading.
   */
  private       SwerveController          swerveController       = null;
  /**
   * Current {@link SwerveInputMode} to use.
   */
  private       SwerveInputMode           currentMode            = SwerveInputMode.ANGULAR_VELOCITY;


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
    newStream.aimTarget = aimTarget;
    newStream.headingEnabled = headingEnabled;
    newStream.aimEnabled = aimEnabled;
    newStream.currentMode = currentMode;
    newStream.translationOnlyEnabled = translationOnlyEnabled;
    newStream.lockedHeading = lockedHeading;
    newStream.swerveController = swerveController;
    newStream.omegaCube = omegaCube;
    newStream.translationCube = translationCube;
    newStream.robotRelative = robotRelative;
    newStream.allianceRelative = allianceRelative;
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
    if (translationOnlyEnabled.isPresent() && translationOnlyEnabled.get().getAsBoolean())
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
      case ANGULAR_VELOCITY ->
      {
        // Do nothing
        break;
      }
      case HEADING ->
      {
        // Do nothing
        break;
      }
      case AIM ->
      {
        // Do nothing
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
      case HEADING ->
      {
        // Do nothing
        break;
      }
      case AIM ->
      {
        // Do nothing
        break;
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
   * Change {@link ChassisSpeeds} to robot relative.
   *
   * @param fieldRelativeSpeeds Field relative speeds to translate into robot-relative speeds.
   * @return Robot relative {@link ChassisSpeeds}.
   */
  private ChassisSpeeds applyRobotRelativeTranslation(ChassisSpeeds fieldRelativeSpeeds)
  {
    if (robotRelative.isPresent() && robotRelative.get().getAsBoolean())
    {
      return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, swerveDrive.getOdometryHeading());
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
   * Apply alliance aware translation which flips the {@link Rotation2d} if the robot is on the Blue alliance.
   *
   * @param fieldRelativeRotation Field-relative {@link Rotation2d} to flip.
   * @return Alliance-oriented {@link Rotation2d}
   */
  private Rotation2d applyAllianceAwareRotation(Rotation2d fieldRelativeRotation)
  {
    if (allianceRelative.isPresent() && allianceRelative.get().getAsBoolean())
    {
      if (robotRelative.isPresent() && robotRelative.get().getAsBoolean())
      {
        throw new RuntimeException("Cannot use robot oriented control with Alliance aware movement!");
      }
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
      {
        return fieldRelativeRotation.rotateBy(Rotation2d.k180deg);
      }
    }
    return fieldRelativeRotation;
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
                                                                  applyAllianceAwareRotation(Rotation2d.fromRadians(
                                                                      swerveController.getJoystickAngle(
                                                                          controllerHeadingX.get()
                                                                                            .getAsDouble(),
                                                                          controllerHeadingY.get()
                                                                                            .getAsDouble()))).getRadians());
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
    }

    currentMode = newMode;

    return applyRobotRelativeTranslation(speeds);
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
    AIM
  }
}

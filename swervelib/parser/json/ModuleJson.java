package swervelib.parser.json;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import edu.wpi.first.math.util.Units;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.motors.SwerveMotor;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.parser.SwerveModulePhysicalCharacteristics;
import swervelib.parser.json.modules.BoolMotorJson;
import swervelib.parser.json.modules.LocationJson;

/**
 * {@link swervelib.SwerveModule} JSON parsed class. Used to access the JSON data.
 */
public class ModuleJson
{

  /**
   * Drive motor device configuration.
   */
  public DeviceJson        drive;
  /**
   * Angle motor device configuration.
   */
  public DeviceJson        angle;
  /**
   * Conversion factor for the module, if different from the one in swervedrive.json
   * <p>
   * Conversion factor applied to the motor controllers PID loops. Can be calculated with
   * {@link swervelib.math.SwerveMath#calculateDegreesPerSteeringRotation(double, double)} for angle motors or
   * {@link swervelib.math.SwerveMath#calculateMetersPerRotation(double, double, double)} for drive motors.
   */
  public MotorConfigDouble conversionFactor        = new MotorConfigDouble(0, 0);
  /**
   * Absolute encoder device configuration.
   */
  public DeviceJson        encoder;
  /**
   * Defines which motors are inverted.
   */
  public BoolMotorJson     inverted;
  /**
   * Absolute encoder offset from 0 in degrees.
   */
  public double            absoluteEncoderOffset;
  /**
   * Absolute encoder inversion state.
   */
  public boolean           absoluteEncoderInverted = false;
  /**
   * The location of the swerve module from the center of the robot in inches.
   */
  public LocationJson      location;

  /**
   * Create the swerve module configuration based off of parsed data.
   *
   * @param anglePIDF               The PIDF values for the angle motor.
   * @param velocityPIDF            The velocity PIDF values for the drive motor.
   * @param physicalCharacteristics Physical characteristics of the swerve module.
   * @param name                    Module json filename.
   * @return {@link SwerveModuleConfiguration} based on the provided data and parsed data.
   */
  public SwerveModuleConfiguration createModuleConfiguration(
      PIDFConfig anglePIDF,
      PIDFConfig velocityPIDF,
      SwerveModulePhysicalCharacteristics physicalCharacteristics,
      String name)
  {
    SwerveMotor           angleMotor = angle.createMotor(false);
    SwerveAbsoluteEncoder absEncoder = encoder.createEncoder(angleMotor);

    // If the absolute encoder is attached.
    if (absEncoder != null && angleMotor.getMotor() instanceof CANSparkMax)
    {
      if (absEncoder.getAbsoluteEncoder() instanceof MotorFeedbackSensor)
      {
        angleMotor.setAbsoluteEncoder(absEncoder);
      }
    }

    // Set the conversion factors to null if they are both 0.
    if (this.conversionFactor != null)
    {
      if (this.conversionFactor.angle == 0 && this.conversionFactor.drive == 0)
      {
        this.conversionFactor = null;
      }
    }

    if (this.conversionFactor == null && physicalCharacteristics.conversionFactor == null)
    {
      throw new RuntimeException("No Conversion Factor configured! Please create SwerveDrive using \n" +
                                 "SwerveParser.createSwerveDrive(driveFeedforward, maxSpeed, angleMotorConversionFactor, driveMotorConversion)\n" +
                                 "OR\n" +
                                 "SwerveParser.createSwerveDrive(maxSpeed, angleMotorConversionFactor, driveMotorConversion)\n" +
                                 "OR\n" +
                                 "Set the conversion factor in physicalproperties.json OR the module JSON file." +
                                 "REMEMBER: You can calculate the conversion factors using SwerveMath.calculateMetersPerRotation AND SwerveMath.calculateDegreesPerSteeringRotation\n");
    } else if (physicalCharacteristics.conversionFactor != null && this.conversionFactor == null)
    {
      this.conversionFactor = physicalCharacteristics.conversionFactor;
    } else if (physicalCharacteristics.conversionFactor !=
               null) // If both are defined, override 0 with the physical characterstics input.
    {
      this.conversionFactor.angle = this.conversionFactor.angle == 0 ? physicalCharacteristics.conversionFactor.angle
                                                                     : this.conversionFactor.angle;
      this.conversionFactor.drive = this.conversionFactor.drive == 0 ? physicalCharacteristics.conversionFactor.drive
                                                                     : this.conversionFactor.drive;
    }

    if (this.conversionFactor.drive == 0 || this.conversionFactor.angle == 0)
    {
      throw new RuntimeException(
          "Conversion factors cannot be 0, please configure conversion factors in physicalproperties.json or the module JSON files.");
    }

    return new SwerveModuleConfiguration(
        drive.createMotor(true),
        angleMotor,
        conversionFactor,
        absEncoder,
        absoluteEncoderOffset,
        Units.inchesToMeters(Math.round(location.x) == 0 ? location.front : location.x),
        Units.inchesToMeters(Math.round(location.y) == 0 ? location.left : location.y),
        anglePIDF,
        velocityPIDF,
        physicalCharacteristics,
        absoluteEncoderInverted,
        inverted.drive,
        inverted.angle,
        name.replaceAll("\\.json", ""));
  }
}

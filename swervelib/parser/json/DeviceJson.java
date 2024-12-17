package swervelib.parser.json;

import static swervelib.telemetry.SwerveDriveTelemetry.canIdWarning;
import static swervelib.telemetry.SwerveDriveTelemetry.i2cLockupWarning;
import static swervelib.telemetry.SwerveDriveTelemetry.serialCommsIssueWarning;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import swervelib.encoders.AnalogAbsoluteEncoderSwerve;
import swervelib.encoders.CANCoderSwerve;
import swervelib.encoders.CanAndMagSwerve;
import swervelib.encoders.PWMDutyCycleEncoderSwerve;
import swervelib.encoders.SparkMaxAnalogEncoderSwerve;
import swervelib.encoders.SparkMaxEncoderSwerve;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.encoders.TalonSRXEncoderSwerve;
import swervelib.imu.ADIS16448Swerve;
import swervelib.imu.ADIS16470Swerve;
import swervelib.imu.ADXRS450Swerve;
import swervelib.imu.AnalogGyroSwerve;
import swervelib.imu.CanandgyroSwerve;
import swervelib.imu.NavXSwerve;
import swervelib.imu.Pigeon2Swerve;
import swervelib.imu.PigeonSwerve;
import swervelib.imu.SwerveIMU;
import swervelib.motors.SparkFlexSwerve;
import swervelib.motors.SparkMaxBrushedMotorSwerve;
import swervelib.motors.SparkMaxBrushedMotorSwerve.Type;
import swervelib.motors.SparkMaxSwerve;
import swervelib.motors.SwerveMotor;
import swervelib.motors.TalonFXSwerve;
import swervelib.motors.TalonSRXSwerve;

/**
 * Device JSON parsed class. Used to access the JSON data.
 */
public class DeviceJson
{

  /**
   * The device type, e.g. pigeon/pigeon2/sparkmax/talonfx/navx
   */
  public String type;
  /**
   * The CAN ID or pin ID of the device.
   */
  public int    id;
  /**
   * The CAN bus name which the device resides on if using CAN.
   */
  public String canbus = "";

  /**
   * Create a {@link SwerveAbsoluteEncoder} from the current configuration.
   *
   * @param motor {@link SwerveMotor} of which attached encoders will be created from, only used when the type is
   *              "attached" or "canandencoder".
   * @return {@link SwerveAbsoluteEncoder} given.
   */
  public SwerveAbsoluteEncoder createEncoder(SwerveMotor motor)
  {
    if (id > 40)
    {
      canIdWarning.set(true);
    }
    switch (type)
    {
      case "none":
        return null;
      case "integrated":
      case "attached":
      case "canandmag":
      case "canandcoder":
        return new SparkMaxEncoderSwerve(motor, 360);
      case "sparkmax_analog":
        return new SparkMaxAnalogEncoderSwerve(motor, 3.3);
      case "sparkmax_analog5v":
        return new SparkMaxAnalogEncoderSwerve(motor, 5);
      case "canandcoder_can":
      case "canandmag_can":
        return new CanAndMagSwerve(id);
      case "ctre_mag":
      case "rev_hex":
      case "throughbore":
      case "am_mag":
      case "dutycycle":
        return new PWMDutyCycleEncoderSwerve(id);
      case "thrifty":
      case "ma3":
      case "analog":
        return new AnalogAbsoluteEncoderSwerve(id);
      case "cancoder":
        return new CANCoderSwerve(id, canbus != null ? canbus : "");
      case "talonsrx_pwm":
        return new TalonSRXEncoderSwerve(motor, FeedbackDevice.PulseWidthEncodedPosition);
      case "talonsrx_analog":
        return new TalonSRXEncoderSwerve(motor, FeedbackDevice.Analog);
      default:
        throw new RuntimeException(type + " is not a recognized absolute encoder type.");
    }
  }

  /**
   * Create a {@link SwerveIMU} from the given configuration.
   *
   * @return {@link SwerveIMU} given.
   */
  public SwerveIMU createIMU()
  {
    if (id > 40)
    {
      canIdWarning.set(true);
    }
    switch (type)
    {
      case "adis16448":
        return new ADIS16448Swerve();
      case "adis16470":
        return new ADIS16470Swerve();
      case "adxrs450":
        return new ADXRS450Swerve();
      case "analog":
        return new AnalogGyroSwerve(id);
      case "canandgyro":
        return new CanandgyroSwerve(id);
      case "navx":
      case "navx_spi":
        return new NavXSwerve(NavXComType.kMXP_SPI);
      case "navx_i2c":
        DriverStation.reportWarning(
            "WARNING: There exists an I2C lockup issue on the roboRIO that could occur, more information here: " +
            "\nhttps://docs.wpilib.org/en/stable/docs/yearly-overview/known-issues" +
            ".html#onboard-i2c-causing-system-lockups",
            false);
        i2cLockupWarning.set(true);
        return new NavXSwerve(NavXComType.kI2C);
      case "navx_usb":
        DriverStation.reportWarning("WARNING: There is issues when using USB camera's and the NavX like this!\n" +
                                    "https://pdocs.kauailabs.com/navx-mxp/guidance/selecting-an-interface/", false);
        serialCommsIssueWarning.set(true);
        return new NavXSwerve(NavXComType.kUSB1);
      case "navx_mxp_serial":
        serialCommsIssueWarning.set(true);
        return new NavXSwerve(NavXComType.kMXP_UART);
      case "pigeon":
        return new PigeonSwerve(id);
      case "pigeon2":
        return new Pigeon2Swerve(id, canbus != null ? canbus : "");
      default:
        throw new RuntimeException(type + " is not a recognized imu/gyroscope type.");
    }
  }

  /**
   * Create a {@link SwerveMotor} from the given configuration.
   *
   * @param isDriveMotor If the motor being generated is a drive motor.
   * @return {@link SwerveMotor} given.
   */
  public SwerveMotor createMotor(boolean isDriveMotor)
  {
    if (id > 40)
    {
      canIdWarning.set(true);
    }
    switch (type)
    {
      case "sparkmax_neo":
      case "neo":
      case "sparkmax":
        return new SparkMaxSwerve(id, isDriveMotor, DCMotor.getNEO(1));
      case "sparkmax_neo550":
      case "neo550":
        return new SparkMaxSwerve(id, isDriveMotor, DCMotor.getNeo550(1));
      case "sparkflex_vortex":
      case "vortex":
      case "sparkflex":
        return new SparkFlexSwerve(id, isDriveMotor, DCMotor.getNeoVortex(1));
      case "sparkflex_neo":
        return new SparkFlexSwerve(id, isDriveMotor, DCMotor.getNEO(1));
      case "sparkflex_neo550":
        return new SparkFlexSwerve(id, isDriveMotor, DCMotor.getNeo550(1));
      case "falcon500":
      case "falcon":
        return new TalonFXSwerve(id, canbus != null ? canbus : "", isDriveMotor, DCMotor.getFalcon500(1));
      case "falcon500foc":
        return new TalonFXSwerve(id, canbus != null ? canbus : "", isDriveMotor, DCMotor.getFalcon500Foc(1));
      case "krakenx60":
      case "talonfx":
        return new TalonFXSwerve(id, canbus != null ? canbus : "", isDriveMotor, DCMotor.getKrakenX60(1));
      case "krakenx60foc":
        return new TalonFXSwerve(id, canbus != null ? canbus : "", isDriveMotor, DCMotor.getKrakenX60Foc(1));
      case "talonsrx":
        return new TalonSRXSwerve(id, isDriveMotor, DCMotor.getCIM(1));
      case "sparkmax_brushed":
        if (canbus == null)
        {
          canbus = "";
        }
        switch (canbus)
        {
          case "greyhill_63r256":
            return new SparkMaxBrushedMotorSwerve(id, isDriveMotor, Type.kQuadrature, 1024, false, DCMotor.getCIM(1));
          case "srx_mag_encoder":
            return new SparkMaxBrushedMotorSwerve(id, isDriveMotor, Type.kQuadrature, 4096, false, DCMotor.getCIM(1));
          case "throughbore":
            return new SparkMaxBrushedMotorSwerve(id, isDriveMotor, Type.kQuadrature, 8192, false, DCMotor.getCIM(1));
          case "throughbore_dataport":
            return new SparkMaxBrushedMotorSwerve(id, isDriveMotor, Type.kNoSensor, 8192, true, DCMotor.getCIM(1));
          case "greyhill_63r256_dataport":
            return new SparkMaxBrushedMotorSwerve(id, isDriveMotor, Type.kQuadrature, 1024, true, DCMotor.getCIM(1));
          case "srx_mag_encoder_dataport":
            return new SparkMaxBrushedMotorSwerve(id, isDriveMotor, Type.kQuadrature, 4096, true, DCMotor.getCIM(1));
          default:
            if (isDriveMotor)
            {
              throw new RuntimeException("Spark MAX " + id + " MUST have a encoder attached to the motor controller.");
            }
            // We are creating a motor for an angle motor which will use the absolute encoder attached to the data port.
            return new SparkMaxBrushedMotorSwerve(id, isDriveMotor, Type.kNoSensor, 0, false, DCMotor.getCIM(1));
        }
      default:
        throw new RuntimeException(type + " is not a recognized motor type.");
    }
  }
}

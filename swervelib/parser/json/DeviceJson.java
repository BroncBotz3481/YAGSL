package swervelib.parser.json;

import com.revrobotics.SparkMaxRelativeEncoder.Type;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort.Port;
import swervelib.encoders.AnalogAbsoluteEncoderSwerve;
import swervelib.encoders.CANCoderSwerve;
import swervelib.encoders.CanAndCoderSwerve;
import swervelib.encoders.PWMDutyCycleEncoderSwerve;
import swervelib.encoders.SparkMaxEncoderSwerve;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.imu.ADIS16448Swerve;
import swervelib.imu.ADIS16470Swerve;
import swervelib.imu.ADXRS450Swerve;
import swervelib.imu.AnalogGyroSwerve;
import swervelib.imu.NavXSwerve;
import swervelib.imu.Pigeon2Swerve;
import swervelib.imu.PigeonSwerve;
import swervelib.imu.SwerveIMU;
import swervelib.motors.SparkMaxBrushedMotorSwerve;
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
    switch (type)
    {
      case "none":
      case "integrated":
      case "attached":
        return null;
      case "canandcoder":
        return new SparkMaxEncoderSwerve(motor, 360);
      case "canandcoder_can":
        return new CanAndCoderSwerve(id);
      case "ma3":
      case "ctre_mag":
      case "rev_hex":
      case "am_mag":
      case "dutycycle":
        return new PWMDutyCycleEncoderSwerve(id);
      case "thrifty":
      case "throughbore":
      case "analog":
        return new AnalogAbsoluteEncoderSwerve(id);
      case "cancoder":
        return new CANCoderSwerve(id, canbus != null ? canbus : "");
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
      case "navx_spi":
        return new NavXSwerve(SPI.Port.kMXP);
      case "navx_i2c":
        DriverStation.reportWarning(
            "WARNING: There exists an I2C lockup issue on the roboRIO that could occur, more information here: " +
            "\nhttps://docs.wpilib.org/en/stable/docs/yearly-overview/known-issues" +
            ".html#onboard-i2c-causing-system-lockups",
            false);
        return new NavXSwerve(I2C.Port.kMXP);
      case "navx_onborard":
        return new NavXSwerve(Port.kOnboard);
      case "navx_usb":
        return new NavXSwerve(Port.kUSB);
      case "navx_mxp":
      case "navx":
        return new NavXSwerve(Port.kMXP);
      case "pigeon":
        return new PigeonSwerve(id);
      case "pigeon2":
        return new Pigeon2Swerve(id, canbus != null ? canbus : "");
      default:
        throw new RuntimeException(type + " is not a recognized absolute encoder type.");
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
    switch (type)
    {
      case "sparkmax_brushed":
        switch (canbus)
        {
          case "greyhill_63r256":
            return new SparkMaxBrushedMotorSwerve(id, isDriveMotor, Type.kQuadrature, 1024, false);
          case "srx_mag_encoder":
            return new SparkMaxBrushedMotorSwerve(id, isDriveMotor, Type.kQuadrature, 4096, false);
          case "throughbore":
            return new SparkMaxBrushedMotorSwerve(id, isDriveMotor, Type.kQuadrature, 8192, false);
          case "throughbore_dataport":
            return new SparkMaxBrushedMotorSwerve(id, isDriveMotor, Type.kNoSensor, 8192, true);
          case "greyhill_63r256_dataport":
            return new SparkMaxBrushedMotorSwerve(id, isDriveMotor, Type.kQuadrature, 1024, true);
          case "srx_mag_encoder_dataport":
            return new SparkMaxBrushedMotorSwerve(id, isDriveMotor, Type.kQuadrature, 4096, true);
          default:
            if (isDriveMotor)
            {
              throw new RuntimeException("Spark MAX " + id + " MUST have a encoder attached to the motor controller.");
            }
            // We are creating a motor for an angle motor which will use the absolute encoder attached to the data port.
            return new SparkMaxBrushedMotorSwerve(id, isDriveMotor, Type.kNoSensor, 0, false);
        }
      case "neo":
      case "sparkmax":
        return new SparkMaxSwerve(id, isDriveMotor);
      case "falcon":
      case "talonfx":
        return new TalonFXSwerve(id, canbus != null ? canbus : "", isDriveMotor);
      case "talonsrx":
        return new TalonSRXSwerve(id, isDriveMotor);
      default:
        throw new RuntimeException(type + " is not a recognized absolute encoder type.");
    }
  }

  /**
   * Create a {@link SwerveAbsoluteEncoder} from the data port on the motor controller.
   *
   * @param motor The motor to create the absolute encoder from.
   * @return {@link SwerveAbsoluteEncoder} from the motor controller.
   */
  public SwerveAbsoluteEncoder createIntegratedEncoder(SwerveMotor motor)
  {
    switch (type)
    {
      case "sparkmax":
        return new SparkMaxEncoderSwerve(motor, 1);
      case "falcon":
      case "talonfx":
        return null;
    }
    throw new RuntimeException(
        "Could not create absolute encoder from data port of " + type + " id " + id);
  }

  /**
   * Get the encoder pulse per rotation based off of the encoder type.
   *
   * @param angleEncoderPulsePerRotation The configured pulse per rotation.
   * @return The correct pulse per rotation based off of the encoder type.
   */
  public int getPulsePerRotation(int angleEncoderPulsePerRotation)
  {
    switch (type)
    {
      case "canandcoder":
        return 360;
      case "falcon":
      case "talonfx":
        return 2048;
      default:
        return angleEncoderPulsePerRotation;
    }
  }
}

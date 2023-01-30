package frc.robot.subsystems.swervedrive.swerve;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.subsystems.swervedrive.swerve.SwerveModule.SwerveModuleLocation;
import java.io.File;
import java.io.IOException;

/**
 * Swerve Drive JSON parser.
 */
public class SwerveParser
{

  /**
   * Open JSON file.
   *
   * @param file JSON File to open.
   * @return JsonNode of file.
   */
  private static JsonNode openJson(File file)
  {
    try
    {
      return new ObjectMapper().readTree(file);
    } catch (IOException e)
    {
      throw new RuntimeException(e);
    }
  }

  /**
   * Check directory structure.
   *
   * @param directory JSON Configuration Directory
   */
  private static void checkDirectory(File directory)
  {
    assert new File(directory, "swerve.json").exists();
    assert new File(directory, "modules").exists() && new File(directory, "modules").isDirectory();
    assert new File(directory, "modules/back").exists() && new File(directory, "modules/back").isDirectory();
    assert new File(directory, "modules/front").exists() && new File(directory, "modules/front").isDirectory();
    assert new File(directory, "modules/back/left.json").exists();
    assert new File(directory, "modules/back/right.json").exists();
    assert new File(directory, "modules/front/left.json").exists();
    assert new File(directory, "modules/front/right.json").exists();
  }

  /**
   * Create the motor controller based off the config.
   *
   * @param motorConfig JSON Motor config object.
   * @return Motor Controller
   */
  private static MotorController createMotor(JsonNode motorConfig)
  {
    int motorID = motorConfig.get("ID").asInt();
    switch (motorConfig.get("Type").asText())
    {
      case "Neo":
        return new CANSparkMax(motorID, MotorType.kBrushless);
      case "Falcon":
        return motorConfig.has("CANBus") ? new WPI_TalonFX(motorID, motorConfig.get("CANBus").asText())
                                         : new WPI_TalonFX(motorID);
    }
    return null;
  }

  /**
   * Create SwerveEncoder from JSON config.
   *
   * @param encoderConfig Swerve Encoder JSON config.
   * @return SwerveEncoder
   */
  private static Object createEncoder(JsonNode encoderConfig)
  {
    int encoderID = encoderConfig.get("ID").asInt();
    switch (encoderConfig.get("Type").asText())
    {
      case "CANCoder":
        return encoderConfig.has("CANBus") ? new CANCoder(encoderID, encoderConfig.get("CANBus").asText())
                                           : new CANCoder(encoderID);
      case "ThriftyBot":
      case "AnalogEncoder":
        return new AnalogEncoder(encoderID);
      case "SRXMagEncoder":
      case "REVThroughBore":
        return new DutyCycleEncoder(encoderID); // This probably doesn't work
    }
    return null;
  }

  /**
   * Create the gyro object based off the configuration.
   *
   * @param gyroConfig Gyro configuration.
   * @return Gyro object.
   */
  private static WPI_Pigeon2 createGyro(JsonNode gyroConfig)
  {
    int gyroID = gyroConfig.get("ID").asInt();

    if (gyroConfig.get("Type").asText().equals("Pigeon2"))
    {
      return gyroConfig.has("CANBus") ? new WPI_Pigeon2(gyroID, gyroConfig.get("CANBus").asText())
                                      : new WPI_Pigeon2(gyroID);
//      case "Pigeon":
//        return new WPI_PigeonIMU(gyroID);
    }
    return null;
  }

  /**
   * Check module JSON config for existing values.
   *
   * @param moduleJson Module JSON object.
   */
  private static void checkModule(JsonNode moduleJson)
  {
    assert moduleJson.has("Motor");
    assert moduleJson.get("Motor").has("Drive");
    assert moduleJson.get("Motor").get("Drive").has("Type") && moduleJson.get("Motor").get("Drive").has("ID");
    assert moduleJson.get("Motor").has("Steer");
    assert moduleJson.get("Motor").get("Steer").has("Type") && moduleJson.get("Motor").get("Steer").has("ID") &&
           moduleJson.get("Motor").get("Steer").has("FreeSpeedRPM");
    assert moduleJson.has("AbsoluteEncoder");
    assert moduleJson.get("AbsoluteEncoder").has("Type") &&
           moduleJson.get("AbsoluteEncoder").has("ID") &&
           moduleJson.get("AbsoluteEncoder").has("Offset");
  }

  /**
   * Check the main json for correct attributes.
   *
   * @param mainJson Main JSON.
   */
  private static void checkMain(JsonNode mainJson)
  {
    assert mainJson.has("WheelBase");
    assert mainJson.has("TrackWidth");
    assert mainJson.has("WheelDiameter");
    assert mainJson.has("Speed");
    assert mainJson.get("Speed").has("MetersPerSecond") && mainJson.get("Speed").has("RadianPerSecond") &&
           mainJson.get("Speed").has("PhysicalMetersPerSecond");
    assert mainJson.has("Acceleration");
    assert mainJson.get("Acceleration").has("MetersPerSecond") && mainJson.get("Acceleration").has("RadianPerSecond");
    assert mainJson.has("Drive");
    assert mainJson.get("Drive").has("Inverted") && mainJson.get("Drive").has("GearRatio") &&
           mainJson.get("Drive").has("MaxPower");
    assert mainJson.has("Steer");
    assert mainJson.get("Steer").has("Inverted") && mainJson.get("Steer").has("GearRatio") &&
           mainJson.get("Steer").has("MaxPower");
    assert mainJson.has("Gyro");
    assert mainJson.get("Gyro").has("Inverted") && mainJson.get("Gyro").has("Type") && mainJson.get("Gyro").has("ID");
    assert mainJson.has("Initial Pose");
    assert mainJson.get("Initial Pose").has("X") && mainJson.get("Initial Pose").has("Y") &&
           mainJson.get("Initial Pose").has("Rotation");
  }

  /**
   * Create SwerveModule from JSON configuration file.
   *
   * @param mainJson       Main JSON to pull from.
   * @param moduleFile     Module specific JSON data to pull from.
   * @param moduleLocation Swerve module location that is being created.
   * @return SwerveModule
   */
  private static SwerveModule<?, ?, ?> createModule(JsonNode mainJson, File moduleFile,
                                                    SwerveModuleLocation moduleLocation)
  {
    JsonNode moduleJson = openJson(moduleFile);
    checkModule(moduleJson);
    MotorController driveMotor = createMotor(moduleJson.get("Motor").get("Drive")),
        steerMotor = createMotor(moduleJson.get("Motor").get("Steer"));
    Object encoder = createEncoder(moduleJson.get("AbsoluteEncoder"));

    SwerveModule<?, ?, ?> module = new SwerveModule<>(driveMotor,
                                                      steerMotor,
                                                      encoder,
                                                      moduleLocation,
                                                      mainJson.get("Drive").get("GearRatio").asDouble(),
                                                      mainJson.get("Steer").get("GearRatio").asDouble(),
                                                      moduleJson.get("AbsoluteEncoder").get("Offset").asDouble(),
                                                      Units.inchesToMeters(mainJson.get("WheelDiameter").asDouble()),
                                                      Units.inchesToMeters(mainJson.get("WheelBase").asDouble()),
                                                      Units.inchesToMeters(mainJson.get("TrackWidth").asDouble()),
                                                      moduleJson.get("Motor").get("Steer").get("FreeSpeedRPM")
                                                                .asDouble(),
                                                      mainJson.get("Speed").get("MetersPerSecond").asDouble(),
                                                      mainJson.get("Acceleration").get("MetersPerSecond").asDouble(),
                                                      mainJson.get("Drive").get("MaxPower").asDouble(),
                                                      mainJson.get("Steer").get("MaxPower").asDouble(),
                                                      mainJson.get("Steer").get("Inverted").asBoolean(),
                                                      mainJson.get("Drive").get("Inverted").asBoolean());

    if (moduleJson.get("Motor").get("Steer").has("PID"))
    {
      JsonNode jsonPIDF = moduleJson.get("Motor").get("Steer").get("PID");
      setModulePIDF(module.turningMotor, jsonPIDF);
    } else if (mainJson.get("Steer").has("PID"))
    {
      JsonNode jsonPIDF = mainJson.get("Steer").get("PID");
      setModulePIDF(module.turningMotor, jsonPIDF);
    }

    if (moduleJson.get("Motor").get("Drive").has("PID"))
    {
      JsonNode jsonPIDF = moduleJson.get("Motor").get("Drive").get("PID");
      setModulePIDF(module.driveMotor, jsonPIDF);
    } else if (mainJson.get("Steer").has("PID"))
    {
      JsonNode jsonPIDF = mainJson.get("Drive").get("PID");
      setModulePIDF(module.turningMotor, jsonPIDF);
    }

    if (moduleJson.get("Motor").get("Steer").has("CurrentLimit"))
    {
      module.turningMotor.setCurrentLimit(moduleJson.get("Motor").get("Steer").get("CurrentLimit")
                                                    .asInt());
    } else if (mainJson.get("Steer").has("CurrentLimit"))
    {
      module.turningMotor.setCurrentLimit(mainJson.get("Steer").get("CurrentLimit").asInt());
    }

    if (moduleJson.get("Motor").get("Drive").has("CurrentLimit"))
    {
      module.driveMotor.setCurrentLimit(moduleJson.get("Motor").get("Drive").get("CurrentLimit")
                                                  .asInt());
    } else if (mainJson.get("Drive").has("CurrentLimit"))
    {
      module.turningMotor.setCurrentLimit(mainJson.get("Drive").get("CurrentLimit").asInt());
    }

    return module;
  }

  /**
   * Set the PIDF for the module if the given JSON has the paramters.
   *
   * @param motor    Motor to configure.
   * @param jsonPIDF JSON ndoe with values.
   */
  private static void setModulePIDF(SwerveMotor motor, JsonNode jsonPIDF)
  {
    if (jsonPIDF.has("P") && jsonPIDF.has("I") && jsonPIDF.has("D") &&
        jsonPIDF.has("F") && jsonPIDF.has("IntegralZone"))
    {
      motor.setPIDF(jsonPIDF.get("P").asDouble(),
                    jsonPIDF.get("I").asDouble(),
                    jsonPIDF.get("D").asDouble(),
                    jsonPIDF.get("F").asDouble(),
                    jsonPIDF.get("IntegralZone").asDouble());
    }
  }

  /**
   * Create SwerveDrive from JSON configuration directory.
   *
   * @param directory JSON Configuration directory.
   * @return SwerveDrive
   */
  public static SwerveDrive fromJSONDirectory(File directory)
  {
    checkDirectory(directory);
    JsonNode swerveJson = openJson(new File(directory, "swerve.json"));
    checkMain(swerveJson);
    return new SwerveDrive(createModule(swerveJson,
                                        new File(directory, "modules/front/left.json"),
                                        SwerveModuleLocation.FrontLeft),
                           createModule(swerveJson,
                                        new File(directory, "modules/front/right.json"),
                                        SwerveModuleLocation.FrontRight),
                           createModule(swerveJson,
                                        new File(directory, "modules/back/left.json"),
                                        SwerveModuleLocation.BackLeft),
                           createModule(swerveJson,
                                        new File(directory, "modules/back/right.json"),
                                        SwerveModuleLocation.BackRight),
                           createGyro(swerveJson.get("Gyro")),
                           swerveJson.get("Speed").get("MetersPerSecond").asDouble(),
                           swerveJson.get("Speed").get("RadianPerSecond").asDouble() * Math.PI,
                           swerveJson.get("Acceleration").get("MetersPerSecond").asDouble(),
                           swerveJson.get("Acceleration").get("RadianPerSecond").asDouble() * Math.PI,
                           swerveJson.get("Speed").get("PhysicalMetersPerSecond").asDouble(),
                           swerveJson.get("Gyro").get("Inverted").asBoolean(),
                           new Pose2d(Units.inchesToMeters(swerveJson.get("Initial Pose").get("X").asDouble()),
                                      Units.inchesToMeters(swerveJson.get("Initial Pose").get("Y").asDouble()),
                                      Rotation2d.fromDegrees(swerveJson.get("Initial Pose").get("Rotation")
                                                                       .asDouble())));

  }
}

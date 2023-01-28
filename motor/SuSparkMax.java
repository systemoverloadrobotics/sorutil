package frc.sorutil.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.sorutil.Errors;
import frc.sorutil.motor.SensorConfiguration.ConnectedSensorSource;
import frc.sorutil.motor.SensorConfiguration.ExternalSensorSource;
import frc.sorutil.motor.SensorConfiguration.IntegratedSensorSource;

public class SuSparkMax extends SuController {
  private static final double STALL_LIMIT = 30;
  private static final double DEFAULT_CURRENT_LIMIT = 70;
  private static final int ANALOG_SAMPLE_DEPTH = 16;

  private final CANSparkMax sparkMax;

  private SuController.ControlMode lastMode;
  private double lastSetpoint;
  
  // Privately store references to the connected sensor objects that Rev uses.
  private SparkMaxAbsoluteEncoder analogSensor;
  private RelativeEncoder digitalSensor;

  public SuSparkMax(CANSparkMax sparkMax, String name, MotorConfiguration motorConfig,
      SensorConfiguration sensorConfig) {
    super(sparkMax, motorConfig, sensorConfig,
        java.util.logging.Logger.getLogger(String.format("SparkMAX(%d: %s)", sparkMax.getDeviceId(), name)), name);

    this.sparkMax = sparkMax;

    aLogger.recordOutput(controllerName + "ID", sparkMax.getDeviceId());
    configure(motorConfig, sensorConfig);
    initializeLogNames();
  }

  @Override
  protected void configure(MotorConfiguration config, SensorConfiguration sensorConfig) {
    Errors.handleRev(sparkMax.restoreFactoryDefaults(), logger, "resetting motor config");

    // This is disabled due to weird interactions on the SparkMax that seems to
    // affect position control.

    // sparkMax.enableVoltageCompensation(12);
    // if (!config.voltageCompenstationEnabled()) {
    // sparkMax.disableVoltageCompensation();
    // }

    sparkMax.setInverted(config.inverted());

    Errors.handleRev(sparkMax.getPIDController().setP(config.pidProfile().p()), logger, "setting P constant");
    Errors.handleRev(sparkMax.getPIDController().setI(config.pidProfile().i()), logger, "setting I constant");
    Errors.handleRev(sparkMax.getPIDController().setD(config.pidProfile().d()), logger, "setting D constant");
    Errors.handleRev(sparkMax.getPIDController().setFF(config.pidProfile().f()), logger, "setting F constant");

    double limit = DEFAULT_CURRENT_LIMIT;
    if (config.currentLimit() != null) {
      limit = config.currentLimit();
    }

    Errors.handleRev(sparkMax.setSmartCurrentLimit((int) STALL_LIMIT, (int) limit), logger, "setting current limit");

    CANSparkMax.IdleMode desiredMode = CANSparkMax.IdleMode.kCoast;
    if (config.idleMode() == SuController.IdleMode.BRAKE) {
      desiredMode = CANSparkMax.IdleMode.kBrake;
    }
    Errors.handleRev(sparkMax.setIdleMode(desiredMode), logger, "setting idle mode");

    // TODO: this should be improved to support multiple PID controllers.
    Errors.handleRev(sparkMax.getPIDController().setOutputRange(-config.maxOutput(), config.maxOutput()), logger,
        "setting neutral deadband");

    if (sensorConfig != null) {
      if (sensorConfig.source() instanceof ConnectedSensorSource) {
        var connected = (ConnectedSensorSource)sensorConfig.source();
        MotorFeedbackSensor sensor;

        switch(connected.type) {
          case QUAD_ENCODER:
            // fallthrough
          case MAG_ENCODER_RELATIVE:
            var alt = sparkMax.getAlternateEncoder(connected.countsPerRev);
            alt.setPositionConversionFactor(360); // Convert revolutions to degrees
            digitalSensor = alt;
            sensor = alt;
            break;
          case MAG_ENCODER_ABSOLUTE:
            // fallthrough
          case PWM_ENCODER:
            //var analog = sparkMax.getAnalog(Mode.kAbsolute);
            var analog = sparkMax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
            analog.setAverageDepth(ANALOG_SAMPLE_DEPTH);
            analog.setPositionConversionFactor(360); // Convert revolutions to degrees
            analog.setAverageDepth(32);

            // Configure the sensor to continuously rotate through 360 degrees.
            sparkMax.getPIDController().setPositionPIDWrappingEnabled(true);
            sparkMax.getPIDController().setPositionPIDWrappingMinInput(0);
            sparkMax.getPIDController().setPositionPIDWrappingMaxInput(360);
            
            analogSensor = analog;
            sensor = analog;
            break;
          default:
            throw new MotorConfigurationError("unknown encoder type: " + connected.type.toString());
        }

        Errors.handleRev(sensor.setInverted(connected.inverted()), logger, "setting inversion of connected sensor");

        Errors.handleRev(sparkMax.getPIDController().setFeedbackDevice(sensor), logger,
            "setting feedback device to sensor device");
      }

      if (sensorConfig.source() instanceof ExternalSensorSource) {
        configureSoftPid();
      }

      if (sensorConfig.source() instanceof IntegratedSensorSource) {
        Errors.handleRev(sparkMax.getPIDController().setFeedbackDevice(sparkMax.getEncoder()), logger,
            "setting feedback device to integral device");
        sparkMax.getEncoder().setPositionConversionFactor(360); // Convert revolutions to degrees
        sparkMax.getEncoder().setVelocityConversionFactor(1);
      }
    }

    Errors.handleRev(sparkMax.burnFlash(), logger, "saving settings to onboard Flash");
  }

  @Override
  public MotorController rawController() {
    return sparkMax;
  }

  boolean canDisconnectGuard = false;

  @Override
  public void tick() {
    super.tick();

    // Only print CAN disconnect messages once
    if (sparkMax.getLastError() == REVLibError.kCANDisconnected) {
      if (!canDisconnectGuard) {
        canDisconnectGuard = true;
        Errors.handleRev(sparkMax.getLastError(), logger, "in motor loop, likely due to setting output");
      }
    } else {
      Errors.handleRev(sparkMax.getLastError(), logger, "in motor loop, likely due to setting output");
    }
    sparkMax.clearFaults();

    if (softPidControllerEnabled) {
      if (softPidControllerMode) {
        // velocity mode
        double current = ((ExternalSensorSource) sensorConfig.source()).sensor.velocity();
        double output = softPidController.calculate(current);
        sparkMax.set(output);
      } else {
        // position mode
        double current = ((ExternalSensorSource) sensorConfig.source()).sensor.position();
        double output = softPidController.calculate(current);
        sparkMax.set(output);
      }
    }

    recordLogs();
  }

  @Override
  public void set(ControlMode mode, double setpoint) {
    // Skip updating the motor if the setpoint is the same, this reduces
    // unneccessary CAN messages.
    if (setpoint == lastSetpoint && mode == lastMode) {
      return;
    }
    lastSetpoint = setpoint;
    lastMode = mode;
    softPidControllerEnabled = false;

    switch (mode) {
      case PERCENT_OUTPUT:
        sparkMax.set(setpoint);
        break;
      case POSITION:
        setPosition(setpoint);
        break;
      case VELOCITY:
        setVelocity(setpoint);
        break;
      case VOLTAGE:
        Errors.handleRev(sparkMax.getPIDController().setReference(setpoint, ControlType.kVoltage), logger,
            "setting motor output");
        break;
    }
  }

  private void setPosition(double setpoint) {
    // Using the integrated Neo source
    if (sensorConfig.source() instanceof SensorConfiguration.IntegratedSensorSource) {
      var integrated = (SensorConfiguration.IntegratedSensorSource) sensorConfig.source();
      double motorDegrees = integrated.outputOffset * setpoint;
      double output = motorDegrees;

      Errors.handleRev(sparkMax.getPIDController().setReference(output, ControlType.kPosition),
          logger, "setting motor output");
      return;
    }
    // Using sensor external to the SparkMAX.
    if (sensorConfig.source() instanceof SensorConfiguration.ExternalSensorSource) {
      softPidControllerEnabled = true;
      softPidControllerMode = false;

      double current = ((ExternalSensorSource) sensorConfig.source()).sensor.position();
      double output = softPidController.calculate(current, setpoint);
      sparkMax.set(output);
      return;
    }
    if (sensorConfig.source() instanceof SensorConfiguration.ConnectedSensorSource) {
      var connected = (SensorConfiguration.ConnectedSensorSource) sensorConfig.source();
      double motorDegrees = connected.outputOffset * setpoint;
      double output = motorDegrees;

      Errors.handleRev(sparkMax.getPIDController().setReference(output, ControlType.kPosition),
          logger, "setting motor output");
      return;
    }
    throw new MotorConfigurationError(
        "unkonwn type of sensor configuration: " + sensorConfig.source().getClass().getName());
  }

  private void setVelocity(double setpoint) {
    // Using the integrated Neo source
    if (sensorConfig.source() instanceof SensorConfiguration.IntegratedSensorSource) {
      var integrated = (SensorConfiguration.IntegratedSensorSource) sensorConfig.source();
      double output = integrated.outputOffset * setpoint;

      Errors.handleRev(sparkMax.getPIDController().setReference(output, ControlType.kVelocity),
          logger, "setting motor output");
      return;
    }
    // Using sensor external to the SparkMAX.
    if (sensorConfig.source() instanceof SensorConfiguration.ExternalSensorSource) {
      softPidControllerEnabled = true;
      softPidControllerMode = true;

      double current = ((ExternalSensorSource) sensorConfig.source()).sensor.velocity();
      double output = softPidController.calculate(current, setpoint);
      sparkMax.set(output);
      return;
    }
    if (sensorConfig.source() instanceof SensorConfiguration.ConnectedSensorSource) {
      var connected = (SensorConfiguration.ConnectedSensorSource) sensorConfig.source();
      double output = connected.outputOffset * setpoint;

      Errors.handleRev(sparkMax.getPIDController().setReference(output, ControlType.kVelocity),
          logger, "setting motor output");
      return;
    }
    throw new MotorConfigurationError(
        "unkonwn type of sensor configuration: " + sensorConfig.source().getClass().getName());
  }

  @Override
  public void stop() {
    sparkMax.stopMotor();
    lastSetpoint = 0;
    lastMode = null;
  }

  @Override
  public void follow(SuController other) {
    // Techincally, the spark max can follow other motor controllers, but for now we
    // will only allow the following of other
    // SparkMaxes for safety.

    if (!(other.rawController() instanceof CANSparkMax)) {
      throw new MotorConfigurationError(
          "SparkMax may only follow other spark max, got instance of: "
              + other.rawController().getClass().getName());
    }

    sparkMax.follow((CANSparkMax) other.rawController());
  }

  @Override
  public double outputPosition() {
    if (sensorConfig.source() instanceof IntegratedSensorSource) {
      return sparkMax.getEncoder().getPosition();
    }
    if (sensorConfig.source() instanceof ConnectedSensorSource) {
      var type = ((ConnectedSensorSource) sensorConfig.source()).type;
      switch (type) {
        case QUAD_ENCODER:
          // fallthrough
        case MAG_ENCODER_RELATIVE:
          return digitalSensor.getPosition();
        case PWM_ENCODER:
          // fallthrough
        case MAG_ENCODER_ABSOLUTE:
          return analogSensor.getPosition();
        default:
          throw new MotorConfigurationError("unknown sensor type: " + type.toString());
      }
    }
    if (sensorConfig.source() instanceof ExternalSensorSource) {
      return ((ExternalSensorSource) sensorConfig.source()).sensor.position();
    }
    return 0;
  }

  @Override
  public double outputVelocity() {
    if (sensorConfig.source() instanceof IntegratedSensorSource) {
      return sparkMax.getEncoder().getVelocity();
    }
    if (sensorConfig.source() instanceof ConnectedSensorSource) {
      var type = ((ConnectedSensorSource) sensorConfig.source()).type;
      switch (type) {
        case QUAD_ENCODER:
          // fallthrough
        case MAG_ENCODER_RELATIVE:
          return digitalSensor.getVelocity();
        case PWM_ENCODER:
          // fallthrough
        case MAG_ENCODER_ABSOLUTE:
          return analogSensor.getVelocity();
        default:
          throw new MotorConfigurationError("unknown sensor type: " + type.toString());
      }
    }
    if (sensorConfig.source() instanceof ExternalSensorSource) {
      return ((ExternalSensorSource) sensorConfig.source()).sensor.velocity();
    }
    return 0;
  }

  @Override
  public void setSensorPosition(double position) {
    if (sensorConfig.source() instanceof ExternalSensorSource) {
      ((ExternalSensorSource) sensorConfig.source()).sensor.setPosition(position);
    }
    if (sensorConfig.source() instanceof ConnectedSensorSource) {
      var type = ((ConnectedSensorSource) sensorConfig.source()).type;
      switch (type) {
        case QUAD_ENCODER:
          // fallthrough
        case MAG_ENCODER_RELATIVE:
          digitalSensor.setPosition(position);
          break;
        case PWM_ENCODER:
          // fallthrough
        case MAG_ENCODER_ABSOLUTE:
          throw new MotorConfigurationError("can't override the position of an absolute sensor");
        default:
          throw new MotorConfigurationError("unknown sensor type: " + type.toString());
      }
    }
  }

  @Override
  public double currentSetpoint() {
    return lastSetpoint;
  }

  @Override
  public ControlMode currentControlMode() {
    return lastMode;
  }

  // ------ Begin logged value names -----
  private String inputVoltageName;
  private String supplyCurrentName;
  private String motorTemperatureName;
  private String motorOutputName;
  // ------- End logged value names ------

  private void initializeLogNames() {
    inputVoltageName = loggerPrefix + "InputVoltage";
    supplyCurrentName = loggerPrefix + "SupplyCurrent";
    motorTemperatureName = loggerPrefix + "MotorTemperature";
    motorOutputName = loggerPrefix + "MotorOutputPercent";
  }

  private void recordLogs() {
    aLogger.recordOutput(inputVoltageName, sparkMax.getBusVoltage());
    aLogger.recordOutput(motorOutputName, sparkMax.getAppliedOutput());
    aLogger.recordOutput(supplyCurrentName, sparkMax.getOutputCurrent());
    aLogger.recordOutput(motorTemperatureName, sparkMax.getMotorTemperature());
  }
}
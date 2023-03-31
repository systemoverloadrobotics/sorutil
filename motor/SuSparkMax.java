package frc.sorutil.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants;
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
  private double lastArbFf;
  
  // Privately store references to the connected sensor objects that Rev uses.
  private SparkMaxAbsoluteEncoder analogSensor;
  private RelativeEncoder digitalSensor;

  public SuSparkMax(CANSparkMax sparkMax, String name, MotorConfiguration motorConfig,
      SensorConfiguration sensorConfig) {
    super(sparkMax, motorConfig, sensorConfig,
        java.util.logging.Logger.getLogger(String.format("SparkMAX(%d: %s)", sparkMax.getDeviceId(), name)), name);

    this.sparkMax = sparkMax;

    configure(motorConfig, sensorConfig);
    initializeLogNames();
    aLogger.recordOutput(loggerPrefix + "ID", sparkMax.getDeviceId());
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
  private REVLibError lastCode;

  @Override
  public void tick() {
    super.tick();

    if (lastCode != sparkMax.getLastError()) {
      Errors.handleRev(sparkMax.getLastError(), logger, "in motor loop, likely from setting a motor update");
      lastCode = sparkMax.getLastError();
    }

    if (softPidControllerEnabled) {
      if (softPidControllerMode) {
        // velocity mode
        double current = ((ExternalSensorSource) sensorConfig.source()).sensor.velocity();
        double output = softPidController.calculate(current);
        sparkMax.set(output + (lastArbFf / Constants.NOMINAL_VOLTAGE));
      } else {
        // position mode
        double current = ((ExternalSensorSource) sensorConfig.source()).sensor.position();
        double output = softPidController.calculate(current);
        sparkMax.set(output + (lastArbFf / Constants.NOMINAL_VOLTAGE));
      }
    }

    recordLogs();
  }


  @Override
  public void set(SuController.ControlMode mode, double setpoint) {
    set(mode, setpoint, 0);
  }

  @Override
  public void set(SuController.ControlMode mode, double setpoint, double arbFfVolts) {
    // Skip updating the motor if the setpoint is the same, this reduces
    // unneccessary CAN messages.
    if (setpoint == lastSetpoint && mode == lastMode && arbFfVolts == lastArbFf) {
      return;
    }
    lastSetpoint = setpoint;
    lastMode = mode;
    softPidControllerEnabled = false;
    lastArbFf = arbFfVolts;

    switch (mode) {
      case PERCENT_OUTPUT:
        sparkMax.set(setpoint);
        break;
      case POSITION:
        setPosition(setpoint, arbFfVolts);
        break;
      case VELOCITY:
        setVelocity(setpoint, arbFfVolts);
        break;
      case VOLTAGE:
        Errors.handleRev(sparkMax.getPIDController().setReference(setpoint, ControlType.kVoltage), logger,
            "setting motor output");
        break;
    }
  }

  private void setPosition(double setpoint, double arbFfVolts) {
    // Using the integrated Neo source
    if (sensorConfig.source() instanceof SensorConfiguration.IntegratedSensorSource) {
      var integrated = (SensorConfiguration.IntegratedSensorSource) sensorConfig.source();
      double motorDegrees = integrated.outputGearRatio * setpoint;
      double output = motorDegrees;

      if (arbFfVolts == 0) {
        Errors.handleRev(sparkMax.getPIDController().setReference(output, ControlType.kPosition),
            logger, "setting motor output");
      } else {
        Errors.handleRev(
            sparkMax.getPIDController().setReference(output, ControlType.kPosition, 0, arbFfVolts, ArbFFUnits.kVoltage),
            logger, "setting motor output");
      }
      return;
    }

    // Using sensor external to the SparkMAX.
    if (sensorConfig.source() instanceof SensorConfiguration.ExternalSensorSource) {
      softPidControllerEnabled = true;
      softPidControllerMode = false;

      double current = ((ExternalSensorSource) sensorConfig.source()).sensor.position();
      double output = softPidController.calculate(current, setpoint);
      sparkMax.set(output + (arbFfVolts / Constants.NOMINAL_VOLTAGE));
      return;
    }

    if (sensorConfig.source() instanceof SensorConfiguration.ConnectedSensorSource) {
      var connected = (SensorConfiguration.ConnectedSensorSource) sensorConfig.source();
      double motorDegrees = connected.outputGearRatio * setpoint;
      double output = motorDegrees;

      if (arbFfVolts == 0) {
        Errors.handleRev(sparkMax.getPIDController().setReference(output, ControlType.kPosition),
            logger, "setting motor output");
      } else {
        Errors.handleRev(
            sparkMax.getPIDController().setReference(output, ControlType.kPosition, 0, arbFfVolts, ArbFFUnits.kVoltage),
            logger, "setting motor output");
      }
      return;
    }
    throw new MotorConfigurationError(
        "unkonwn type of sensor configuration: " + sensorConfig.source().getClass().getName());
  }

  private void setVelocity(double setpoint, double arbFfVolts) {
    // Using the integrated Neo source
    if (sensorConfig.source() instanceof SensorConfiguration.IntegratedSensorSource) {
      var integrated = (SensorConfiguration.IntegratedSensorSource) sensorConfig.source();
      double output = integrated.outputGearRatio * setpoint;

      if (arbFfVolts == 0) {
        Errors.handleRev(sparkMax.getPIDController().setReference(output, ControlType.kVelocity),
            logger, "setting motor output");
      } else {
        Errors.handleRev(
            sparkMax.getPIDController().setReference(output, ControlType.kVelocity, 0, arbFfVolts, ArbFFUnits.kVoltage),
            logger, "setting motor output");
      }
      return;
    }

    // Using sensor external to the SparkMAX.
    if (sensorConfig.source() instanceof SensorConfiguration.ExternalSensorSource) {
      softPidControllerEnabled = true;
      softPidControllerMode = true;

      double current = ((ExternalSensorSource) sensorConfig.source()).sensor.velocity();
      double output = softPidController.calculate(current, setpoint);
      sparkMax.set(output + (arbFfVolts / Constants.NOMINAL_VOLTAGE));
      return;
    }

    if (sensorConfig.source() instanceof SensorConfiguration.ConnectedSensorSource) {
      var connected = (SensorConfiguration.ConnectedSensorSource) sensorConfig.source();
      double output = connected.outputGearRatio * setpoint;

      if (arbFfVolts == 0) {
        Errors.handleRev(sparkMax.getPIDController().setReference(output, ControlType.kVelocity),
            logger, "setting motor output");
      } else {
        Errors.handleRev(
            sparkMax.getPIDController().setReference(output, ControlType.kVelocity, 0, arbFfVolts, ArbFFUnits.kVoltage),
            logger, "setting motor output");
      }
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
      IntegratedSensorSource source = (IntegratedSensorSource) sensorConfig.source();
      return sparkMax.getEncoder().getPosition() / source.outputGearRatio;
    }
    if (sensorConfig.source() instanceof ConnectedSensorSource) {
      var source = (ConnectedSensorSource) sensorConfig.source();
      switch (source.type) {
        case QUAD_ENCODER:
          // fallthrough
        case MAG_ENCODER_RELATIVE:
          return digitalSensor.getPosition() / source.outputGearRatio;
        case PWM_ENCODER:
          // fallthrough
        case MAG_ENCODER_ABSOLUTE:
          // While this being divided by the ratio is techincally correct, it 
          // probably doesn't make much sense in practice. This should always be
          // 1 when used in absolute mode.
          return analogSensor.getPosition() / source.outputGearRatio;
        default:
          throw new MotorConfigurationError("unknown sensor type: " + source.type.toString());
      }
    }
    if (sensorConfig.source() instanceof ExternalSensorSource) {
      ExternalSensorSource source = (ExternalSensorSource) sensorConfig.source();
      return source.sensor.position() / source.outputGearRatio;
    }
    return 0;
  }

  @Override
  public double outputVelocity() {
    if (sensorConfig.source() instanceof IntegratedSensorSource) {
      IntegratedSensorSource source = (IntegratedSensorSource) sensorConfig.source();
      return sparkMax.getEncoder().getVelocity() / source.outputGearRatio;
    }
    if (sensorConfig.source() instanceof ConnectedSensorSource) {
      var source = (ConnectedSensorSource) sensorConfig.source();
      switch (source.type) {
        case QUAD_ENCODER:
          // fallthrough
        case MAG_ENCODER_RELATIVE:
          return digitalSensor.getVelocity() / source.outputGearRatio;
        case PWM_ENCODER:
          // fallthrough
        case MAG_ENCODER_ABSOLUTE:
          // While this being divided by the ratio is techincally correct, it 
          // probably doesn't make much sense in practice. This should always be
          // 1 when used in absolute mode.
          return analogSensor.getVelocity() / source.outputGearRatio;
        default:
          throw new MotorConfigurationError("unknown sensor type: " + source.type.toString());
      }
    }
    if (sensorConfig.source() instanceof ExternalSensorSource) {
      ExternalSensorSource source = (ExternalSensorSource) sensorConfig.source();
      return source.sensor.velocity() / source.outputGearRatio;
    }
    return 0;
  }

  @Override
  public void setSensorPosition(double position) {
    if (sensorConfig.source() instanceof IntegratedSensorSource) {
      IntegratedSensorSource source = (IntegratedSensorSource) sensorConfig.source();
      sparkMax.getEncoder().setPosition(position * source.outputGearRatio);
    }
    if (sensorConfig.source() instanceof ExternalSensorSource) {
      ExternalSensorSource source = (ExternalSensorSource) sensorConfig.source();
      source.sensor.setPosition(position * source.outputGearRatio);
    }
    if (sensorConfig.source() instanceof ConnectedSensorSource) {
      var source = (ConnectedSensorSource) sensorConfig.source();
      switch (source.type) {
        case QUAD_ENCODER:
          // fallthrough
        case MAG_ENCODER_RELATIVE:
          digitalSensor.setPosition(position * source.outputGearRatio);
          break;
        case PWM_ENCODER:
          // fallthrough
        case MAG_ENCODER_ABSOLUTE:
          throw new MotorConfigurationError("can't override the position of an absolute sensor");
        default:
          throw new MotorConfigurationError("unknown sensor type: " + source.type.toString());
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
  private String arbFfName;
  // ------- End logged value names ------

  private void initializeLogNames() {
    inputVoltageName = loggerPrefix + "InputVoltage";
    supplyCurrentName = loggerPrefix + "SupplyCurrent";
    motorTemperatureName = loggerPrefix + "MotorTemperature";
    motorOutputName = loggerPrefix + "MotorOutputPercent";
    arbFfName = loggerPrefix + "ArbitraryFeedForwardVolts";
  }

  private void recordLogs() {
    aLogger.recordOutput(inputVoltageName, sparkMax.getBusVoltage());
    aLogger.recordOutput(motorOutputName, sparkMax.getAppliedOutput());
    aLogger.recordOutput(supplyCurrentName, sparkMax.getOutputCurrent());
    aLogger.recordOutput(motorTemperatureName, sparkMax.getMotorTemperature());
    aLogger.recordOutput(arbFfName, lastArbFf);
  }
}
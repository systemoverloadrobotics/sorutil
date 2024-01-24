package frc.sorutil.motor;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants;
import frc.sorutil.Errors;
import frc.sorutil.motor.SensorConfiguration.ConnectedSensorSource;
import frc.sorutil.motor.SensorConfiguration.ExternalSensorSource;
import frc.sorutil.motor.SensorConfiguration.IntegratedSensorSource;

public class SuSparkFlex extends SuController {
  private static final double STALL_LIMIT = 30;
  private static final double DEFAULT_CURRENT_LIMIT = 70;
  private static final int ANALOG_SAMPLE_DEPTH = 16;

  private final CANSparkFlex sparkFlex;

  private SuController.ControlMode lastMode;
  private double lastSetpoint;
  private double lastArbFf;
  
  // Privately store references to the connected sensor objects that Rev uses.
  private SparkAbsoluteEncoder analogSensor;
  private RelativeEncoder digitalSensor;

  public SuSparkFlex(CANSparkFlex sparkFlex, String name, MotorConfiguration motorConfig,
      SensorConfiguration sensorConfig) {
    super(sparkFlex, motorConfig, sensorConfig,
        java.util.logging.Logger.getLogger(String.format("sparkFlex(%d: %s)", sparkFlex.getDeviceId(), name)), name);

    this.sparkFlex = sparkFlex;

    configure(motorConfig, sensorConfig);
    initializeLogNames();
    aLogger.recordOutput(loggerPrefix + "ID", sparkFlex.getDeviceId());
  }

  @Override
  protected void configure(MotorConfiguration config, SensorConfiguration sensorConfig) {
    Errors.handleRev(sparkFlex.restoreFactoryDefaults(), logger, "resetting motor config");

    // This is disabled due to weird interactions on the sparkFlex that seems to
    // affect position control.

    // sparkFlex.enableVoltageCompensation(12);
    // if (!config.voltageCompenstationEnabled()) {
    // sparkFlex.disableVoltageCompensation();
    // }

    sparkFlex.setInverted(config.inverted());

    Errors.handleRev(sparkFlex.getPIDController().setP(config.pidProfile().p()), logger, "setting P constant");
    Errors.handleRev(sparkFlex.getPIDController().setI(config.pidProfile().i()), logger, "setting I constant");
    Errors.handleRev(sparkFlex.getPIDController().setD(config.pidProfile().d()), logger, "setting D constant");
    Errors.handleRev(sparkFlex.getPIDController().setFF(config.pidProfile().f()), logger, "setting F constant");

    double limit = DEFAULT_CURRENT_LIMIT;
    if (config.currentLimit() != null) {
      limit = config.currentLimit();
    }

    Errors.handleRev(sparkFlex.setSmartCurrentLimit((int) STALL_LIMIT, (int) limit), logger, "setting current limit");

    CANSparkFlex.IdleMode desiredMode = CANSparkFlex.IdleMode.kCoast;
    if (config.idleMode() == SuController.IdleMode.BRAKE) {
      desiredMode = CANSparkFlex.IdleMode.kBrake;
    }
    Errors.handleRev(sparkFlex.setIdleMode(desiredMode), logger, "setting idle mode");

    // TODOO: this should be improved to support multiple PID controllers.
    Errors.handleRev(sparkFlex.getPIDController().setOutputRange(-config.maxOutput(), config.maxOutput()), logger,
        "setting neutral deadband");

    if (sensorConfig != null) {
      if (sensorConfig.source() instanceof ConnectedSensorSource) {
        var connected = (ConnectedSensorSource)sensorConfig.source();
        MotorFeedbackSensor sensor;

        switch(connected.type) {
          case QUAD_ENCODER:
            // fallthrough
          case MAG_ENCODER_RELATIVE:
            var alt = sparkFlex.getExternalEncoder(connected.countsPerRev);
            alt.setPositionConversionFactor(360); // Convert revolutions to degrees
            digitalSensor = alt;
            sensor = alt;
            break;
          case MAG_ENCODER_ABSOLUTE:
            // fallthrough
          case PWM_ENCODER:
            //var analog = sparkFlex.getAnalog(Mode.kAbsolute);
            var analog = sparkFlex.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
            analog.setAverageDepth(ANALOG_SAMPLE_DEPTH);
            analog.setPositionConversionFactor(360); // Convert revolutions to degrees
            analog.setAverageDepth(32);

            // Configure the sensor to continuously rotate through 360 degrees.
            sparkFlex.getPIDController().setPositionPIDWrappingEnabled(true);
            sparkFlex.getPIDController().setPositionPIDWrappingMinInput(0);
            sparkFlex.getPIDController().setPositionPIDWrappingMaxInput(360);
            
            analogSensor = analog;
            sensor = analog;
            break;
          default:
            throw new MotorConfigurationError("unknown encoder type: " + connected.type.toString());
        }

        Errors.handleRev(sensor.setInverted(connected.inverted()), logger, "setting inversion of connected sensor");

        Errors.handleRev(sparkFlex.getPIDController().setFeedbackDevice(sensor), logger,
            "setting feedback device to sensor device");
      }

      if (sensorConfig.source() instanceof ExternalSensorSource) {
        configureSoftPid();
      }

      if (sensorConfig.source() instanceof IntegratedSensorSource) {
        Errors.handleRev(sparkFlex.getPIDController().setFeedbackDevice(sparkFlex.getEncoder()), logger,
            "setting feedback device to integral device");
        sparkFlex.getEncoder().setPositionConversionFactor(360); // Convert revolutions to degrees
        sparkFlex.getEncoder().setVelocityConversionFactor(1);
      }
    }

    Errors.handleRev(sparkFlex.burnFlash(), logger, "saving settings to onboard Flash");
  }

  @Override
  public MotorController rawController() {
    return sparkFlex;
  }

  boolean canDisconnectGuard = false;
  private REVLibError lastCode;

  @Override
  public void tick() {
    super.tick();

    if (lastCode != sparkFlex.getLastError()) {
      Errors.handleRev(sparkFlex.getLastError(), logger, "in motor loop, likely from setting a motor update");
      lastCode = sparkFlex.getLastError();
    }

    if (softPidControllerEnabled) {
      if (softPidControllerMode) {
        // velocity mode
        double current = ((ExternalSensorSource) sensorConfig.source()).sensor.velocity();
        double output = softPidController.calculate(current);
        sparkFlex.set(output + (lastArbFf / Constants.NOMINAL_VOLTAGE));
      } else {
        // position mode
        double current = ((ExternalSensorSource) sensorConfig.source()).sensor.position();
        double output = softPidController.calculate(current);
        sparkFlex.set(output + (lastArbFf / Constants.NOMINAL_VOLTAGE));
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
        sparkFlex.set(setpoint);
        break;
      case POSITION:
        setPosition(setpoint, arbFfVolts);
        break;
      case VELOCITY:
        setVelocity(setpoint, arbFfVolts);
        break;
      case VOLTAGE:
        Errors.handleRev(sparkFlex.getPIDController().setReference(setpoint, ControlType.kVoltage), logger,
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
        Errors.handleRev(sparkFlex.getPIDController().setReference(output, ControlType.kPosition),
            logger, "setting motor output");
      } else {
        Errors.handleRev(
            sparkFlex.getPIDController().setReference(output, ControlType.kPosition, 0, arbFfVolts, ArbFFUnits.kVoltage),
            logger, "setting motor output");
      }
      return;
    }

    // Using sensor external to the sparkFlex.
    if (sensorConfig.source() instanceof SensorConfiguration.ExternalSensorSource) {
      softPidControllerEnabled = true;
      softPidControllerMode = false;

      double current = ((ExternalSensorSource) sensorConfig.source()).sensor.position();
      double output = softPidController.calculate(current, setpoint);
      sparkFlex.set(output + (arbFfVolts / Constants.NOMINAL_VOLTAGE));
      return;
    }

    if (sensorConfig.source() instanceof SensorConfiguration.ConnectedSensorSource) {
      var connected = (SensorConfiguration.ConnectedSensorSource) sensorConfig.source();
      double motorDegrees = connected.outputGearRatio * setpoint;
      double output = motorDegrees;

      if (arbFfVolts == 0) {
        Errors.handleRev(sparkFlex.getPIDController().setReference(output, ControlType.kPosition),
            logger, "setting motor output");
      } else {
        Errors.handleRev(
            sparkFlex.getPIDController().setReference(output, ControlType.kPosition, 0, arbFfVolts, ArbFFUnits.kVoltage),
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
        Errors.handleRev(sparkFlex.getPIDController().setReference(output, ControlType.kVelocity),
            logger, "setting motor output");
      } else {
        Errors.handleRev(
            sparkFlex.getPIDController().setReference(output, ControlType.kVelocity, 0, arbFfVolts, ArbFFUnits.kVoltage),
            logger, "setting motor output");
      }
      return;
    }

    // Using sensor external to the sparkFlex.
    if (sensorConfig.source() instanceof SensorConfiguration.ExternalSensorSource) {
      softPidControllerEnabled = true;
      softPidControllerMode = true;

      double current = ((ExternalSensorSource) sensorConfig.source()).sensor.velocity();
      double output = softPidController.calculate(current, setpoint);
      sparkFlex.set(output + (arbFfVolts / Constants.NOMINAL_VOLTAGE));
      return;
    }

    if (sensorConfig.source() instanceof SensorConfiguration.ConnectedSensorSource) {
      var connected = (SensorConfiguration.ConnectedSensorSource) sensorConfig.source();
      double output = connected.outputGearRatio * setpoint;

      if (arbFfVolts == 0) {
        Errors.handleRev(sparkFlex.getPIDController().setReference(output, ControlType.kVelocity),
            logger, "setting motor output");
      } else {
        Errors.handleRev(
            sparkFlex.getPIDController().setReference(output, ControlType.kVelocity, 0, arbFfVolts, ArbFFUnits.kVoltage),
            logger, "setting motor output");
      }
      return;
    }
    throw new MotorConfigurationError(
        "unkonwn type of sensor configuration: " + sensorConfig.source().getClass().getName());
  }

  @Override
  public void stop() {
    sparkFlex.stopMotor();
    lastSetpoint = 0;
    lastMode = null;
  }

  @Override
  public void follow(SuController other) {
    // Techincally, the spark flex can follow other motor controllers, but for now we
    // will only allow the following of other
    // sparkFlexes for safety.

    if (!(other.rawController() instanceof CANSparkFlex)) {
      throw new MotorConfigurationError(
          "Sparkflex may only follow other spark flex, got instance of: "
              + other.rawController().getClass().getName());
    }

    sparkFlex.follow((CANSparkFlex) other.rawController());
  }

  @Override
  public double outputPosition() {
    if (sensorConfig.source() instanceof IntegratedSensorSource) {
      IntegratedSensorSource source = (IntegratedSensorSource) sensorConfig.source();
      return sparkFlex.getEncoder().getPosition() / source.outputGearRatio;
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
      return sparkFlex.getEncoder().getVelocity() / source.outputGearRatio;
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
      sparkFlex.getEncoder().setPosition(position * source.outputGearRatio);
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
    aLogger.recordOutput(inputVoltageName, sparkFlex.getBusVoltage());
    aLogger.recordOutput(motorOutputName, sparkFlex.getAppliedOutput());
    aLogger.recordOutput(supplyCurrentName, sparkFlex.getOutputCurrent());
    aLogger.recordOutput(motorTemperatureName, sparkFlex.getMotorTemperature());
    aLogger.recordOutput(arbFfName, lastArbFf);
  }
}
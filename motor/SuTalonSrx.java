package frc.sorutil.motor;

import java.util.logging.Logger;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IFollower;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.sorutil.Errors;
import frc.sorutil.motor.SensorConfiguration.ConnectedSensorSource;
import frc.sorutil.motor.SensorConfiguration.ExternalSensorSource;
import frc.sorutil.motor.SensorConfiguration.IntegratedSensorSource;

public class SuTalonSrx extends SuController {
  private static final double DEFAULT_CURRENT_LIMIT = 60;
  private static final double DEFAULT_NEUTRAL_DEADBAND = 0.04;

  private final WPI_TalonSRX talon;
  private MotorConfiguration config;

  private boolean voltageControlOverrideSet = false;
  private Double lastVoltageCompensation = null;

  private SuController.ControlMode lastMode;
  private double lastSetpoint;

  public SuTalonSrx(WPI_TalonSRX talon, String name, MotorConfiguration motorConfig, SensorConfiguration sensorConfig) {
    super(talon, motorConfig, sensorConfig, Logger.getLogger(String.format("TalonSRX(%d: %s)", talon.getDeviceID(), name)));

    this.talon = talon;
  }

  @Override
  public void configure(MotorConfiguration config, SensorConfiguration sensorConfig) {
    Errors.handleCtre(talon.clearMotionProfileHasUnderrun(), logger, "clearing motion profile");
    Errors.handleCtre(talon.clearMotionProfileTrajectories(), logger, "clearing motion profile trajectories");

    Errors.handleCtre(talon.configFactoryDefault(), logger, "resetting motor config");

    talon.setInverted(config.inverted());

    Errors.handleCtre(talon.config_kP(0, config.pidProfile().p()), logger, "setting P constant");
    Errors.handleCtre(talon.config_kI(0, config.pidProfile().i()), logger, "setting I constant");
    Errors.handleCtre(talon.config_kD(0, config.pidProfile().d()), logger, "setting D constant");
    Errors.handleCtre(talon.config_kF(0, config.pidProfile().f()), logger, "setting F constant");

    double limit = DEFAULT_CURRENT_LIMIT;
    if (config.currentLimit() != null) {
      limit = config.currentLimit();
    }

    SupplyCurrentLimitConfiguration limitConfig = new SupplyCurrentLimitConfiguration();
    limitConfig.currentLimit = limit;
    limitConfig.triggerThresholdCurrent = limit;

    Errors.handleCtre(talon.configSupplyCurrentLimit(limitConfig), logger, "setting current limit");

    restoreDefaultVoltageCompensation();

    NeutralMode desiredMode = NeutralMode.Coast;
    if (config.idleMode() == IdleMode.BRAKE) {
      desiredMode = NeutralMode.Brake;
    }
    talon.setNeutralMode(desiredMode);

    double neutralDeadband = DEFAULT_NEUTRAL_DEADBAND;
    if (config.neutralDeadband() != null){
      neutralDeadband = config.neutralDeadband();
    }
    Errors.handleCtre(talon.configNeutralDeadband(neutralDeadband), logger, "setting neutral deadband");

    Errors.handleCtre(talon.configPeakOutputForward(config.maxOutput()), logger, "configuring max output");
    Errors.handleCtre(talon.configPeakOutputReverse(-config.maxOutput()), logger, "configuring max output");

    if (sensorConfig != null) {
      if (sensorConfig.source() instanceof IntegratedSensorSource) {
        throw new MotorConfigurationError(
            "Talon SRX has no integrated sensor, but motor was configured to use integerated sensor source, use ConnectedSensorSource instead for sensors connected to the Talon's IO port.");
      }

      if (sensorConfig.source() instanceof ConnectedSensorSource) {
        var connectedSensor = ((ConnectedSensorSource) sensorConfig.source());
        FeedbackDevice device = null;
        switch (connectedSensor.type) {
          case MAG_ENCODER_ABSOLUTE:
            device = FeedbackDevice.CTRE_MagEncoder_Absolute;
            break;
          case MAG_ENCODER_RELATIVE:
            device = FeedbackDevice.CTRE_MagEncoder_Relative;
            break;
          case PWM_ENCODER:
            device = FeedbackDevice.PulseWidthEncodedPosition;
            break;
          case QUAD_ENCODER:
            device = FeedbackDevice.QuadEncoder;
            break;
        }
        Errors.handleCtre(talon.configSelectedFeedbackSensor(device), logger,
            "configuring selected sensor");
        if (connectedSensor.inverted()) {
          talon.setSensorPhase(true);
        }
      }

      if (sensorConfig.source() instanceof ExternalSensorSource) {
        configureSoftPid();
      }
    }
  }

  @Override
  public MotorController rawController() {
    return talon;
  }

  @Override
  public void tick() {
    Errors.handleCtre(talon.getLastError(), logger, "in motor loop, likely from setting a motor update");

    if (softPidControllerEnabled) {
      if (softPidControllerMode) {
        // velocity mode
        double current = ((ExternalSensorSource) sensorConfig.source()).sensor.velocity();
        double output = softPidController.calculate(current);
        talon.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, output);
      } else {
        // position mode
        double current = ((ExternalSensorSource) sensorConfig.source()).sensor.position();
        double output = softPidController.calculate(current);
        talon.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, output);
      }
    }
  }

  private void restoreDefaultVoltageCompensation() {
    Errors.handleCtre(talon.configVoltageCompSaturation(SuController.DEFAULT_VOLTAGE_COMPENSTAION),
        logger, "configuring voltage compenstation");
    talon.enableVoltageCompensation(config.voltageCompenstationEnabled());
  }

  @Override
  public void set(SuController.ControlMode mode, double setpoint) {
    if (voltageControlOverrideSet && mode != SuController.ControlMode.VOLTAGE) {
      restoreDefaultVoltageCompensation();
      voltageControlOverrideSet = false;
      lastVoltageCompensation = null;
    }

    // Skip updating the motor if the setpoint is the same, this reduces unneccessary CAN messages.
    if (setpoint == lastSetpoint && mode == lastMode) {
      return;
    }
    lastSetpoint = setpoint;
    lastMode = mode;
    softPidControllerEnabled = false;

    switch(mode) {
      case PERCENT_OUTPUT:
        talon.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, setpoint);
        break;
      case POSITION:
        setPosition(setpoint);
        break;
      case VELOCITY:
        setVelocity(setpoint);
        break;
      case VOLTAGE:
        boolean negative = setpoint < 0;
        double abs = Math.abs(setpoint);
        if (lastVoltageCompensation != null && setpoint != lastVoltageCompensation) {
          Errors.handleCtre(talon.configVoltageCompSaturation(abs), logger,
              "configuring voltage compenstation for voltage control");
          lastVoltageCompensation = setpoint;
        }
        if (!voltageControlOverrideSet) {
          talon.enableVoltageCompensation(true);
          voltageControlOverrideSet = true;
        }
        talon.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, negative ? -1 : 1);
        break;
    }
  }

  private void setPosition(double setpoint) {
    // Using sensor external to the Falcon.
    if (sensorConfig.source() instanceof SensorConfiguration.ExternalSensorSource) {
      softPidControllerEnabled = true;
      softPidControllerMode = false;

      double current = ((ExternalSensorSource) sensorConfig.source()).sensor.position();
      double output = softPidController.calculate(current, setpoint);
      talon.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, output);
      return;
    }
    if (sensorConfig.source() instanceof SensorConfiguration.ConnectedSensorSource) {
      var connected = (SensorConfiguration.ConnectedSensorSource)sensorConfig.source();
      if (connected.type == SensorConfiguration.ConnectedSensorType.QUAD_ENCODER
          || connected.type == SensorConfiguration.ConnectedSensorType.MAG_ENCODER_RELATIVE) {
        double sensorDegrees = connected.outputOffset * setpoint;
        double countsToDegrees = connected.countsPerRev / 360.0;
        double output = sensorDegrees*countsToDegrees;

        talon.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, output);
        return;
      }
      
      if (connected.type == SensorConfiguration.ConnectedSensorType.PWM_ENCODER
          || connected.type == SensorConfiguration.ConnectedSensorType.MAG_ENCODER_ABSOLUTE) {
        double sensorDegrees = connected.outputOffset * setpoint;
        // Map the full range of the rotation to 0-1, assuming that the sensor can't over-rotate.
        double output = sensorDegrees/360.0;

        talon.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, output);
        return;
      }
      throw new MotorConfigurationError(
          "unsupported type of connected sensor: " + sensorConfig.source().getClass().getName());
    }
    throw new MotorConfigurationError(
        "unkonwn type of sensor configuration: " + sensorConfig.source().getClass().getName());
  }

  private void setVelocity(double setpoint) {
    // Using sensor external to the Falcon.
    if (sensorConfig.source() instanceof SensorConfiguration.ExternalSensorSource) {
      softPidControllerEnabled = true;
      softPidControllerMode = true;
      double current = ((ExternalSensorSource) sensorConfig.source()).sensor.velocity();
      double output = softPidController.calculate(current, setpoint);
      
      talon.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, output);
      return;
    }
    if (sensorConfig.source() instanceof SensorConfiguration.ConnectedSensorSource) {
      var connected = (SensorConfiguration.ConnectedSensorSource) sensorConfig.source();
      if (connected.type == SensorConfiguration.ConnectedSensorType.QUAD_ENCODER
          || connected.type == SensorConfiguration.ConnectedSensorType.MAG_ENCODER_RELATIVE) {
        double motorRpm = connected.outputOffset * setpoint;
        double motorRps = motorRpm / 60.0;
        double output = (connected.countsPerRev * motorRps) / 10.0;

        talon.set(com.ctre.phoenix.motorcontrol.ControlMode.Velocity, output);
        return;
      }

      if (connected.type == SensorConfiguration.ConnectedSensorType.PWM_ENCODER
          || connected.type == SensorConfiguration.ConnectedSensorType.MAG_ENCODER_ABSOLUTE) {
        throw new MotorConfigurationError(
            "cannot configure velocity setpoint mode while using an absolute sensor.");
      }
      throw new MotorConfigurationError(
          "unsupported type of connected sensor: " + sensorConfig.source().getClass().getName());
    }
    throw new MotorConfigurationError(
        "unkonwn type of sensor configuration: " + sensorConfig.source().getClass().getName());
  }

  @Override
  public void stop() {
    talon.stopMotor();
  }

  @Override
  public void follow(SuController other) { 
    if (!(other.rawController() instanceof IFollower)) {
      throw new MotorConfigurationError(
          "CTRE motor controllers can only follow other motor controllers from CTRE");
    }

    talon.follow((IMotorController) other.rawController());
  }

  @Override
  public double outputPosition() {
    return 0;
  }

  @Override
  public double outputVelocity() {
    return 0;
  }

  @Override
  public void setSensorPosition(double position) {

  }
}

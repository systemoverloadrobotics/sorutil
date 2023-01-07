package frc.sorutil.motor;

import java.util.logging.Logger;
import com.ctre.phoenix.motorcontrol.IFollower;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.sorutil.Errors;
import frc.sorutil.motor.SensorConfiguration.ConnectedSensorSource;
import frc.sorutil.motor.SensorConfiguration.IntegratedSensorSource;

public class SuVictorSpx extends SuController {
  private static final double DEFAULT_NEUTRAL_DEADBAND = 0.04;

  private final WPI_VictorSPX victor;

  private boolean voltageControlOverrideSet = false;
  private Double lastVoltageCompensation = null;

  private SuController.ControlMode lastMode;
  private double lastSetpoint;

  public SuVictorSpx(WPI_VictorSPX victor, String name, MotorConfiguration motorConfig, SensorConfiguration sensorConfig) {
    super(victor, motorConfig, sensorConfig, Logger.getLogger(String.format("VictorSPX(%d: %s)", victor.getDeviceID(), name)));

    this.victor = victor;
  }

  @Override
  public void configure(MotorConfiguration config, SensorConfiguration sensorConfig) {
    Errors.handleCtre(victor.configFactoryDefault(), logger, "resetting motor config");

    victor.setInverted(config.inverted());

    Errors.handleCtre(victor.config_kP(0, config.pidProfile().p()), logger, "setting P constant");
    Errors.handleCtre(victor.config_kI(0, config.pidProfile().i()), logger, "setting I constant");
    Errors.handleCtre(victor.config_kD(0, config.pidProfile().d()), logger, "setting D constant");
    Errors.handleCtre(victor.config_kF(0, config.pidProfile().f()), logger, "setting F constant");

    if (config.currentLimit() != null) {
      logger.warning(
          "Victor SPX initialized with current limit, current limits are NOT SUPPORTED, ignoring instruction.");
    }

    Errors.handleCtre(victor.configVoltageCompSaturation(SuController.DEFAULT_VOLTAGE_COMPENSTAION),
        logger, "configuring voltage compenstation");
    victor.enableVoltageCompensation(config.voltageCompenstationEnabled());

    NeutralMode desiredMode = NeutralMode.Coast;
    if (config.idleMode() == IdleMode.BRAKE) {
      desiredMode = NeutralMode.Brake;
    }
    victor.setNeutralMode(desiredMode);

    double neutralDeadband = DEFAULT_NEUTRAL_DEADBAND;
    if (config.neutralDeadband() != null){
      neutralDeadband = config.neutralDeadband();
    }
    Errors.handleCtre(victor.configNeutralDeadband(neutralDeadband), logger, "setting neutral deadband");

    Errors.handleCtre(victor.configPeakOutputForward(config.maxOutput()), logger, "configuring max output");
    Errors.handleCtre(victor.configPeakOutputReverse(-config.maxOutput()), logger, "configuring max output");

    if (sensorConfig != null) {
      if (sensorConfig.source() instanceof IntegratedSensorSource) {
        throw new MotorConfigurationError(
            "Victor SPX has no integrated sensor, but motor was configured to use integerated sensor source");
      }
      if (sensorConfig.source() instanceof ConnectedSensorSource) {
        throw new MotorConfigurationError(
            "Victor SPX does not supported directly connected sensors, but was configured to use one.");
      }
    }
  }

  @Override
  public MotorController rawController() {
    return victor;
  }

  @Override
  public void tick() {
    Errors.handleCtre(victor.getLastError(), logger, "in motor loop, likely from setting a motor update");
  }

  private void restoreDefaultVoltageCompensation() {
    Errors.handleCtre(victor.configVoltageCompSaturation(SuController.DEFAULT_VOLTAGE_COMPENSTAION),
        logger, "configuring voltage compenstation");
    victor.enableVoltageCompensation(config.voltageCompenstationEnabled());
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

    switch (mode) {
      case PERCENT_OUTPUT:
        victor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, setpoint);
      case POSITION:
        victor.set(com.ctre.phoenix.motorcontrol.ControlMode.Position, positionSetpoint(setpoint));
      case VELOCITY:
        victor.set(com.ctre.phoenix.motorcontrol.ControlMode.Velocity, velocitySetpoint(setpoint));
      case VOLTAGE:
        boolean negative = setpoint < 0;
        double abs = Math.abs(setpoint);
        if (lastVoltageCompensation != null && setpoint != lastVoltageCompensation) {
          Errors.handleCtre(victor.configVoltageCompSaturation(abs), logger,
              "configuring voltage compenstation for voltage control");
          lastVoltageCompensation = setpoint;
        }
        if (!voltageControlOverrideSet) {
          victor.enableVoltageCompensation(true);
          voltageControlOverrideSet = true;
        }
        victor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, negative ? -1 : 1);
    }
  }

  private double positionSetpoint(double setpoint) {
    // Using sensor external to the Falcon.
    if (sensorConfig.source() instanceof SensorConfiguration.ExternalSensorSource) {
      throw new MotorConfigurationError("compensated external velocity control is not yet supported.");
    }
    if (sensorConfig.source() instanceof SensorConfiguration.ConnectedSensorSource) {
      throw new MotorConfigurationError("VictorSPX motor controllers don't support connected sensors.");
    }
    throw new MotorConfigurationError(
        "unkonwn type of sensor configuration: " + sensorConfig.source().getClass().getName());
  }

  private double velocitySetpoint(double setpoint) {
    // Using sensor external to the Falcon.
    if (sensorConfig.source() instanceof SensorConfiguration.ExternalSensorSource) {
      throw new MotorConfigurationError("compensated external velocity control is not yet supported.");
    }
    if (sensorConfig.source() instanceof SensorConfiguration.ConnectedSensorSource) {
      throw new MotorConfigurationError("VictorSPX motor controllers don't support connected sensors.");
    }
    throw new MotorConfigurationError(
        "unkonwn type of sensor configuration: " + sensorConfig.source().getClass().getName());
  }

  @Override
  public void stop() {
    victor.stopMotor();
  }

  @Override
  public void follow(SuController other) { 
    if (!(other.rawController() instanceof IFollower)) {
      throw new MotorConfigurationError(
          "CTRE motor controllers can only follow other motor controllers from CTRE");
    }

    victor.follow((IMotorController) other.rawController());
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

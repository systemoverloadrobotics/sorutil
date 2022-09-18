package frc.sorutil.motor;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public interface SuController {
  public void configure(MotorConfiguration config, SensorConfiguration sensorConfig); 

  MotorController rawController();

  public void tick();

  public void set(SuMotor.ControlMode mode, double setpoint);

  /**
   * Stops the motor regardless of output mode.
   */
  public void stop();

  public void follow(SuController other);

  /**
   * Using the SensorConfiguration on this motor, retreive the end effector position in degrees. This will take into
   * account the scaling factor provided in the SensorConfiguration.
   */
  public double outputPosition();

  /**
   * Using the SensorConfiguration on this motor, retreive the end effector output velocity in RPM. This will take into
   * account the scaling factor provided in the SensorConfiguration.
   */
  public double outputVelocity();

  /**
   * setSensorPosition will override the current sensor position and update the internal counter to the new position. As
   * with outputPosition, the value is in degrees.
   */
  public void setSensorPosition(double position);
}

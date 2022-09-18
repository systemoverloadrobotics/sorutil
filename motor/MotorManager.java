package frc.sorutil.motor;

import java.util.HashSet;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorManager extends SubsystemBase{
  private static final MotorManager instance;
  static {
    instance = new MotorManager();
  }

  private HashSet<SuMotor> motors = new HashSet<>();

  public static MotorManager instance() {
    return instance;
  }

  // Because this is a subsystem, this method will be called periodically by the command scheduler.
  @Override
  public void periodic() {
    for (SuMotor motor : motors) {
      motor.tick();
    }
  }

  protected void addMotor(SuMotor motor) {
    motors.add(motor);
  }
}

package frc.sorutil.motor;

import java.util.HashSet;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorManager extends SubsystemBase{
  private static final MotorManager instance;
  static {
    instance = new MotorManager();
  }

  private HashSet<SuController> motors = new HashSet<>();

  public static MotorManager instance() {
    return instance;
  }

  // Because this is a subsystem, this method will be called periodically by the command scheduler.
  @Override
  public void periodic() {
    for (SuController motor : motors) {
      motor.tick();
    }
  }

  protected void addMotor(SuController motor) {
    motors.add(motor);
  }
}

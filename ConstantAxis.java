package frc.sorutil;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class ConstantAxis {
  private final int stick;
  private final int idx;
  
  public ConstantAxis(int joystick, int idx) {
    this.stick = joystick;
    this.idx = idx;
  }

  public DoubleSupplier get() {
    return () -> ConstantInput.get().lazyJoy(stick).getRawAxis(idx);
  }

  public BooleanEvent getButton() {
    return ConstantInput.get().lazyJoy(stick).axisGreaterThan(stick, idx, CommandScheduler.getInstance().getDefaultButtonLoop());
  }
}

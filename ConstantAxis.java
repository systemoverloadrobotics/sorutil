package frc.sorutil;

import java.util.function.DoubleSupplier;

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
}

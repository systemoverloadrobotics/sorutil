package frc.sorutil;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ConstantButton {
  private final int stick;
  private final int idx;

  public ConstantButton(int joystick, int idx) {
    this.idx = idx;
    this.stick = joystick;
  }  

  public JoystickButton get() {
    return new JoystickButton(ConstantInput.get().lazyJoy(stick), idx);
  }

  public BooleanSupplier supplier() {
    return () -> ConstantInput.get().lazyJoy(stick).getRawButton(idx);
  }
}

package frc.sorutil;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj.Joystick;

// ConstantInput is a singleton that manages the Joystick inputs to the program,
// allowing for "constant" declarations of Joystick buttons and axes.
public class ConstantInput {
  private static final ConstantInput inst = new ConstantInput();  

  private final Map<Integer, Joystick> joys = new HashMap<>();

  public static ConstantInput get() {
    return inst;
  }

  public Joystick lazyJoy(int idx) {
    if (!joys.containsKey(idx)) {
      joys.put(idx, new Joystick(idx));
    }

    return joys.get(idx);
  }
}

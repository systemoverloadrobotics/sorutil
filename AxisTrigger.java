package frc.sorutil;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A {@link Trigger} that gets its state from a {@link GenericHID}.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class AxisTrigger extends Trigger {
  /**
   * Creates a joystick button for triggering commands.
   *
   * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick, etc)
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
   */
  public AxisTrigger(GenericHID joystick, int axisNumber, double lim) {
    super(() -> joystick.getRawAxis(axisNumber) > lim);
    System.out.println();
    requireNonNullParam(joystick, "joystick", "JoystickButton");
  }
}

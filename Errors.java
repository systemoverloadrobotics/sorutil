package frc.sorutil;

import java.util.logging.Logger;
import com.ctre.phoenix6.StatusCode;
import com.revrobotics.REVLibError;

public class Errors {
  public static void handleCtre(StatusCode e, Logger l) {
    handleCtre(e, l, null);
  }

  public static void handleCtre(StatusCode e, Logger l, String desc) {
    if (e == StatusCode.OK) {
      return;
    }

    if (desc != null && desc != "") {
      l.warning("Error in motor controller: " + e.name() + ": " + desc);
    } else {
      l.warning("Error in motor controller: " + e.name());
    }
  }

  public static void handleRev(REVLibError e, Logger l) {
    handleRev(e, l, null);
  }

  public static void handleRev(REVLibError e, Logger l, String desc) {
    if (e == REVLibError.kOk) {
      return;
    }

    if (desc != null && desc != "") {
      l.warning("Error in motor controller: " + e.name() + ": " + desc);
    } else {
      l.warning("Error in motor controller: " + e.name());
    }
  }
  
}

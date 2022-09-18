package frc.sorutil;

import java.util.logging.Logger;
import com.ctre.phoenix.ErrorCode;
import com.revrobotics.REVLibError;

public class Errors {
  public static void handleCtre(ErrorCode e, Logger l) {
    handleCtre(e, l, null);
  }

  public static void handleCtre(ErrorCode e, Logger l, String desc) {
    if (e == ErrorCode.OK) {
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

package frc.sorutil;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.logging.LogManager;
import edu.wpi.first.wpilibj.Filesystem;

public class Logging {
  public static void initLogging() {
    try {
      File dir = Filesystem.getDeployDirectory();
      if (dir.isDirectory()) {
        String logFile = dir.getAbsolutePath() + "/logging.properties";
        FileInputStream configFile = new FileInputStream(logFile);
        LogManager.getLogManager().readConfiguration(configFile);
        System.out.println("---- Logging Initialized ----");
      } else {
        throw new IOException("Failed to find log file config.");
      }
    } catch (IOException ex) {
      System.out.println("WARNING: Could not open configuration file");
      System.out.println("WARNING: Logging not configured (console output only)");
    }
  } 
}

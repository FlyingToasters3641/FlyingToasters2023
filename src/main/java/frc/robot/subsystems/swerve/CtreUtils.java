package frc.robot.subsystems.swerve;

import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public final class CtreUtils {
  private CtreUtils() {
  }

  public static void checkCtreError(ErrorCode errorCode, String message) {
    if (errorCode != ErrorCode.OK) {
      String error = String.format("%s: %s", message, errorCode.toString());
      DriverStation.reportError(error, false);
      try {
        DataLogManager.log(error);
      } catch (Exception e) {
        System.out.println("Couldn't log ctre error to datalog manager!" + e.getStackTrace());
      }
    }
  }
}

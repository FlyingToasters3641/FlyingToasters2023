package frc.robot.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

import org.opencv.core.Point;



public final class Units {
    private final static double kMetersPerInch = 0.0254;

    public static double inchesToMeters(double inches) {
        return inches * kMetersPerInch;
    }

    public static double feetToMeters(double feet) {
        return feet * (kMetersPerInch * 12.0);
    }
    public static double average(double num1, double num2) {
        return (num1 + num2) / 2;  
    }

    public static double millisecondsToSeconds(double milliseconds) {
        return milliseconds / 1000.0;
    }
    
   public static Rotation2d toRegularCoordinates(Rotation2d angle) {
       double zeroTothreesixtyangle;
       double m_angle = angle.getDegrees();
    if (Math.signum(m_angle) == -1) {
        zeroTothreesixtyangle = 360 - m_angle;
    } else {
        zeroTothreesixtyangle = m_angle;
    }    
    return new Rotation2d(zeroTothreesixtyangle);
   }
   
   public static Rotation2d getTargetAngle(Translation2d currentPosition) {
        double x = currentPosition.getX();
        double y = currentPosition.getY();
        double rotationRadians = Math.atan((4.115 - y) / (8.2295 - x));
         if (x > 8.2295) {
             rotationRadians -= Math.PI;
             //rotationRadians *= -1;
         } else {
         }
         SmartDashboard.putNumber("angle to target", rotationRadians);

        return new Rotation2d(rotationRadians);
   }
    /**
     * Converts radians (0 to 6.28) to degrees (0 to 360)
     * @param radians the value to be converted
     * @return the degree value
     */
    public static double radiansToDegrees(double radians) {
        return radians * 180 / Math.PI;
    }

    /**
     * Converts degrees (0 to 360) to radians (0 to 6.28)
     * @param degrees the value to be converted
     * @return the radian value
     */
    public static double degreesToRadians(double degrees) {
        return degrees / 180 * Math.PI;
    }
}

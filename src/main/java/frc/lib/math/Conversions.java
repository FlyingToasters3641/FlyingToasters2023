package frc.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Conversions {

    /**
     * @param positionCounts CANCoder Position Counts
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double CANcoderToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return CANCoder Position Counts
     */
    public static double degreesToCANcoder(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 4096.0));
    }

    // /**
    //  * @param counts Falcon Position Counts
    //  * @param gearRatio Gear Ratio between Falcon and Mechanism
    //  * @return Degrees of Rotation of Mechanism
    //  */
    // public static double falconToDegrees(double positionCounts, double gearRatio) {
    //     return positionCounts * (360.0 / (gearRatio * 2048.0));
    // }

    /**
     * @param counts Falcon Position Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconFusedUnitsToDegrees(double positionCounts) {
        return positionCounts * 360.0 + 180.0;
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Position Counts
     */
    // public static double degreesToFalcon(double degrees, double gearRatio) {
    //     return degrees / (360.0 / (gearRatio * 2048.0));
    // }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @return Falcon fused angle (-0.5 to 0.5)
     */
    public static double degreesToFalconFusedUnits(double degrees) {
        return (degrees - 180.0) / 360.0;
    }

    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    // public static double falconToRPM(double velocityCounts, double gearRatio) {
    //     double motorRPM = velocityCounts * (600.0 / 2048.0);        
    //     double mechRPM = motorRPM / gearRatio;
    //     return mechRPM;
    // }

    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param velocitycounts Falcon Velocity Counts (RPS)
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
     * @return Wheel Velocity Counts (MPS)
     */
    public static double falconRPSToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelMPS = ((velocitycounts / gearRatio) * circumference);
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
     * @return Falcon Velocity Counts
     */
    public static double MPStoRPS(double velocity, double circumference, double gearRatio) {
        return (velocity * 60) / circumference;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
     * @return Falcon Velocity Counts
     */
    // public static double MPSToFalcon(double velocity, double circumference, double gearRatio){
    //     double wheelRPM = ((velocity * 60) / circumference);
    //     double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
    //     return wheelVelocity;
    // }

    /**
     * @param positionCounts Falcon Position Counts (Rotations)
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Wheel
     * @return Meters
     */
    public static double falconRotationsToMeters(double positionCounts, double circumference, double gearRatio){
        return positionCounts * circumference / gearRatio;
    }

    /**
     * @param meters Meters
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Wheel
     * @return Falcon Position Counts
     */
    public static double MetersToFalcon(double meters, double circumference, double gearRatio){
        return meters / (circumference / (gearRatio * 2048.0));
    }

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
    
   public static Rotation2d toUnsingedProgramming(Rotation2d angle) {
       double zeroTothreesixtyangle;
       double m_angle = angle.getDegrees();
    if (Math.signum(m_angle) == -1) {
        zeroTothreesixtyangle = 360 - m_angle;
    } else {
        zeroTothreesixtyangle = m_angle;
    }    
    return new Rotation2d(zeroTothreesixtyangle);
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
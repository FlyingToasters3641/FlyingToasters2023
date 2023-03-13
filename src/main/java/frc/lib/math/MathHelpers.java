package frc.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;

public class MathHelpers {
    public class SineInterpolation {
        double firstPoint;
        double secondPoint;
        double totalTime;
     
        /**
         * A quick one dimensional sine interpolator to smoothly interpolate between two values.
         * 
         * @param firstPoint the first point
         * @param secondPoint   the end point
         * @param totalTime  the total interpolation time
         */
        public SineInterpolation(double firstPoint, double secondPoint, double totalTime) {
           this.firstPoint = firstPoint;
           this.secondPoint = secondPoint;
           this.totalTime = totalTime;
        }
     
        /**
         * Calculates the point for a given time
         * 
         * @param curTime time in seconds
         * @return the angle in radians
         */
        public double calculatePoint(double curTime) {
           Double difference = Math.abs(firstPoint - secondPoint / 2);
           Double value = difference * Math.sin(curTime - Math.PI / 2) + difference;
           return value;
        }
     
        /**
         * Calculates the derivative for a given time
         * 
         * @param curTime time in seconds
         * @return the velocity in radians
         */
        public double calculateDerivative(double curTime) {
           Double difference = (secondPoint - firstPoint) / 2;
           Double derivative = (Math.sin((Math.PI * curTime) / totalTime) / totalTime) * (Math.PI * difference);
           return derivative;
        }
        //Link: https://www.desmos.com/calculator/cp8gp7kolt
     }
}

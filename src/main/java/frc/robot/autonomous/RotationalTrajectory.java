package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;

public class RotationalTrajectory {
   Rotation2d startAngle;
   Rotation2d endAngle;
   double totalTime;

   /**
    * Rotational Trajectory to calculate interpolate and deliver a rotational
    * trajectory
    * 
    * @param startAngle the starting angle of the trajectory
    * @param endAngle   the ending angle of the trajectory
    * @param totalTime  the total time that the trajectory will take in seconds
    */
   public RotationalTrajectory(Rotation2d startAngle, Rotation2d endAngle, double totalTime) {
      this.startAngle = startAngle;
      this.endAngle = endAngle;
      this.totalTime = totalTime;
   }

   /**
    * Calculates the angle of the trajectory for a given time
    * 
    * @param curTime time in seconds
    * @return the angle in radians
    */
   public double calculateRotation(double curTime) {
      Double difference = Math.abs(startAngle.getRadians() - endAngle.getRadians() / 2);
      Double angle = difference * Math.sin(curTime - Math.PI / 2) + difference;
      return angle;
   }

   /**
    * Calculates the velocity of the trajectory for a given time
    * 
    * @param curTime time in seconds
    * @return the velocity in radians
    */
   public double calculateRotationalVelocity(double curTime) {
      Double difference = (endAngle.getRadians() - startAngle.getRadians()) / 2;
      Double velocity = (Math.sin((Math.PI * curTime) / totalTime) / totalTime) * (Math.PI * difference);
      return velocity;
   }
   //Link: https://www.desmos.com/calculator/cp8gp7kolt
}

package frc.robot.util;

public class MathUtil {

    //gives a randon number within a min and max
    public static int randomNum(int min, int max) {
        int number = (int)(Math.random()*(max-min+1)+min);
        return number;
    }
}

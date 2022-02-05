package frc.robot;


public class Vision {
    
    private static double cameraHeight = 26.5;
    private static double cameraAngle = 0.0;

    //Error from the limelight when reading the angle. It appears to be consistent
    private static double angleError = 0.04;

    public static double getDistancefromY(double verticalAngle, double targetHeight)
    {
        return ((targetHeight - cameraHeight) / Math.tan(Math.toRadians(verticalAngle + cameraAngle - angleError)));
    }
}

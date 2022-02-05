package frc.robot;

public class QuadraticController {

    //minimum and maximum motor speeds

    private double minMotSpeed;
    private double maxMotSpeed;

    public QuadraticController(double min, double max)
    {
        minMotSpeed = min;
        maxMotSpeed = max;
    }

    public double quadraticPositionAndSpeed(double positionGoal, double currentPosition) {

        // position could be angle offset, encoder count, inches, etc.

        // maximum speed should be reached at the middle position


        double minimumMotorSpeed = minMotSpeed;
        double maximumMotorSpeed = maxMotSpeed; 

        // need to look at system of equations again and see if I want endpoint to just be positionGoal with speed 0. (position, 0)
    
        double a = ((positionGoal * maximumMotorSpeed - minimumMotorSpeed * positionGoal - minimumMotorSpeed * (positionGoal / 2) + minimumMotorSpeed * (positionGoal / 2)) / (positionGoal * (positionGoal / 2) * ((positionGoal / 2) - positionGoal)));
    
        double b = ((maximumMotorSpeed - a * (positionGoal / 2) * (positionGoal / 2) - minimumMotorSpeed) / (positionGoal / 2));

        double speed = a * currentPosition * currentPosition + b * currentPosition + minimumMotorSpeed;
        
        return speed; // value would be from 0.0 to 1.0 like any motor needs to be
    }
}
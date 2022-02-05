package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Drivetrain {

    CANSparkMax flMotor = new CANSparkMax(4, MotorType.kBrushless);
    CANSparkMax frMotor = new CANSparkMax(6, MotorType.kBrushless);
    CANSparkMax blMotor = new CANSparkMax(7, MotorType.kBrushless);
    CANSparkMax brMotor = new CANSparkMax(3, MotorType.kBrushless);




    // making all doubles in case of weird mathematic things happening
    final static double ENCODERS_PER_REV = 42.0; // encoder counts per revolution of the motor
    final static double GEAR_RATIO = 12.75; // inches // motor spins 12.75 times for wheel to spin once
    final static double wheelRadius = 4.0; // inches
    final static double driveTrainInchesOffset = 1.5 + 3.5; // inches

    final double minMotorSpeedEncoders = 0.1; // need to test these numbers for best accuracy
    final double maxMotorSpeedEncoders = 0.4; // may change with robot weight

    final double motorSpeedThresholdTeleop = 0.2;
    final double maxMotorSpeedTeleop = 0.87;

    //final double encoderCount = ((25 - driveTrainInchesOffset)/(2.0*Math.PI*wheelRadius)) * ENCODERS_PER_REV * GEAR_RATIO;

    QuadraticController driveTrainQuadraticController = new QuadraticController(minMotorSpeedEncoders, maxMotorSpeedEncoders); 
    

    MecanumDrive mecanumDrive = new MecanumDrive(flMotor, blMotor, frMotor, brMotor);

    public void init(){
        System.out.println("init called!");
        //System.out.println(((25 - driveTrainInchesOffset)/(2.0*Math.PI*wheelRadius)) * ENCODERS_PER_REV * GEAR_RATIO);
        frMotor.setInverted(true);
        brMotor.setInverted(true);

        resetDriveTrainEncoders();
        
        flMotor.getEncoder().setPositionConversionFactor(ENCODERS_PER_REV);
        frMotor.getEncoder().setPositionConversionFactor(ENCODERS_PER_REV);
        blMotor.getEncoder().setPositionConversionFactor(ENCODERS_PER_REV);
        brMotor.getEncoder().setPositionConversionFactor(ENCODERS_PER_REV);
    }

    public void driveTrainTeleop(double xInput, double yInput, double zInput) {

        //Changed the method slightly so it can take input from any controller setup
        double ySpeed = yInput;
        double xSpeed = xInput;
        double zSpeed = zInput;

        if((ySpeed > -motorSpeedThresholdTeleop) && (ySpeed < motorSpeedThresholdTeleop)) { // minimum thresholds to go so no interference
            ySpeed = 0.00;
        }

        if((xSpeed > -motorSpeedThresholdTeleop) && (xSpeed < motorSpeedThresholdTeleop)) {
            xSpeed = 0.00;
        }

        if((zSpeed > -motorSpeedThresholdTeleop) && (zSpeed < motorSpeedThresholdTeleop)) {
            zSpeed = 0.00;
        }

        if(ySpeed > maxMotorSpeedTeleop) {
           ySpeed = maxMotorSpeedTeleop;
        }

        if(xSpeed > maxMotorSpeedTeleop) {
            xSpeed = maxMotorSpeedTeleop;
        }

        if(zSpeed > maxMotorSpeedTeleop) {
            zSpeed = maxMotorSpeedTeleop;
        }

        mecanumDrive.driveCartesian(-ySpeed, xSpeed, zSpeed);
    }


    public void driveTrainByInches(double inches, int direction){ // 0 = forward. 1 = back. 2 = left. 3 = right. 4 = turn left. 5 = turn right
        
        //double encoderCounts = 0.0;
        // minus 1.5 (driveTrainInchesOffset) will have to be test for accuracy on different speeds and when robot weight changes
        // we could literally make the encoderCount PositionConversionFactor equate to inches outright. That could
        // just be for the drive train

        if(direction == 0){ // forward
            mecanumDrive.driveCartesian(Robot.quadraticPositionAndSpeed(minMotorSpeedEncoders, 
            maxMotorSpeedEncoders, inchesToEncoders(inches), flMotor.getEncoder().getPosition()), 0.0, 0.0);
        }

        if(direction == 1){ // back
            mecanumDrive.driveCartesian(-Robot.quadraticPositionAndSpeed(minMotorSpeedEncoders, 
            maxMotorSpeedEncoders, inchesToEncoders(inches), -flMotor.getEncoder().getPosition()), 0.0, 0.0);
        }

        if(direction == 2){ // left
            mecanumDrive.driveCartesian(0.0, Robot.quadraticPositionAndSpeed(-minMotorSpeedEncoders, 
            -maxMotorSpeedEncoders, inchesToEncoders(inches), frMotor.getEncoder().getPosition()), 0.0);
        }

        if(direction == 3){ // right
            mecanumDrive.driveCartesian(0.0, Robot.quadraticPositionAndSpeed(minMotorSpeedEncoders, 
            maxMotorSpeedEncoders, inchesToEncoders(inches), flMotor.getEncoder().getPosition()), 0.0);
        }

        if(direction == 4){ // turn left
            mecanumDrive.driveCartesian(0.0, 0.0, Robot.quadraticPositionAndSpeed(-minMotorSpeedEncoders, 
            -maxMotorSpeedEncoders, inchesToEncoders(inches), frMotor.getEncoder().getPosition()));
        }

        if(direction == 5){ // turn right
            mecanumDrive.driveCartesian(0.0, 0.0, Robot.quadraticPositionAndSpeed(minMotorSpeedEncoders, 
            maxMotorSpeedEncoders, inchesToEncoders(inches), flMotor.getEncoder().getPosition()));
        }
    }

    public static double inchesToEncoders(double inches){
        return ((   (Math.abs(inches) - driveTrainInchesOffset)  /  (2.0*Math.PI*wheelRadius)  ) * ENCODERS_PER_REV * GEAR_RATIO);
    }

    
    public void resetDriveTrainEncoders() {
        flMotor.getEncoder().setPosition(0);
        frMotor.getEncoder().setPosition(0);
        blMotor.getEncoder().setPosition(0);
        brMotor.getEncoder().setPosition(0);
    }

    public boolean isMoving()
    {
        return flMotor.getEncoder().getVelocity() > 0.05 && frMotor.getEncoder().getVelocity() > 0.05 && blMotor.getEncoder().getVelocity() > 0.05 && brMotor.getEncoder().getVelocity() > 0.05;
    }

    public void stopMotors()
    {
        flMotor.set(0.0);
        frMotor.set(0.0);
        blMotor.set(0.0);
        brMotor.set(0.0);

        
    }
}




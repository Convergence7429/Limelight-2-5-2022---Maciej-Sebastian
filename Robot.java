// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.*;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
//import edu.wpi.first.wpilibj.
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.util.WPILibVersion;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import edu.wpi.first.wpilibj.util.Color;



public class Robot extends TimedRobot {

  //Solenoid solenoid = new Solenoid(ModuleType.kRev, 0);
  
  CvSource outputStream;

  Timer timer = new Timer();

  public Drivetrain drive = new Drivetrain();
  //NetworkTableInstance networkTableInstance = new NetworkTableInstance(NetworkTableInstance.getDefault());


  static Joystick stick;
  static Joystick aux;

  //Sensitivity of inputs
  double xSensitivity = 0.5;
  double ySensitivity = 0.5;
  double zSensitivity = 0.3;
  
  //disable for testing purposes, or if we need auto in teleop
  boolean manualControlEnabled = false;

  //Joystick is 1, xbox controller is 3
  int stickID = 1;
  int auxID = 3;


  //do not modify from here
  int stage;

  //set a value for this
  double targetHeight = 98.5;

  //do not set any value for this here
  double targetDistance;



  //for testing purposes, might use in actual code
  boolean shouldBeDriving;

  //enables "drive until the color sensor sees a color" mode. Here because Mr. Lathrop thinks we need it
  boolean colorTestMode = false;

  Limelight forwardCamera;

  //variables for a limit switch (for testing purposes)
  DigitalInput limitSwitch;
  int limitSwitchPort = 0;
  
  QuadraticController strafeController = new QuadraticController(0.1, 0.15);
  
  Compressor compressor = new Compressor(10, PneumaticsModuleType.REVPH);

  

  //color sensor on i2c
  // private final I2C.Port port = I2C.Port.kOnboard;
  // private final ColorSensorV3 m_colorSensor = new ColorSensorV3(port);
  // private final ColorMatch matcher = new ColorMatch();
  // private final Color blueTarget = new Color(0.143, 0.427, 0.429);
  // private final Color redTarget = new Color(0.561, 0.232, 0.114);
  // private final Color defaultColor = new Color(0.336, 0.470,0.195);


  Color detectedColor;
  
  @Override
  public void robotInit() {

    //CameraServer.startAutomaticCapture();
    

    NetworkTable cameraTable = NetworkTableInstance.getDefault().getTable("limelight");
    forwardCamera = new Limelight(cameraTable);
    
    stick = new Joystick(stickID);
    aux = new Joystick(auxID);

    
    //initialize limit switch
    limitSwitch = new DigitalInput(limitSwitchPort);

    //Add coolor to matching algorithm
    // matcher.addColorMatch(blueTarget);
    // matcher.addColorMatch(redTarget);
    // matcher.addColorMatch(defaultColor);

    shouldBeDriving = true;
  }

  @Override
  public void autonomousInit() {
    drive.init();
    timer.start();
    //forwardCamera = new Limelight(limelight);

    stage = 0;
  }

  @Override
  public void autonomousPeriodic() {

    System.out.println(Vision.getDistancefromY(forwardCamera.angleOffsetY(), targetHeight));

    switch (stage) {
      
      case 0 :
      {
        targetDistance = (Vision.getDistancefromY(forwardCamera.angleOffsetY(), targetHeight) - 100);
        //targetDistance = 60;
        drive.resetDriveTrainEncoders();
        stage = 1;
        break;
      }

      case 1 :
      {
        drive.driveTrainByInches(targetDistance, 1);
        break;
      }
      //   if(timer.get() > 4.0 && !drive.isMoving())
      //   {
      //     stage = 2;
      //   }
      //   break;
      // }
      // case 2 :
      // {
      //   drive.stopMotors();
      //   break;
      // }
    }
  
  }
  

  @Override
  public void teleopInit() {
    drive.init();
    drive.resetDriveTrainEncoders();
    forwardCamera.setPipeline(1);
    

  }

  @Override
  public void teleopPeriodic() {

    // targetDistance = Vision.getDistancefromY(forwardCamera.angleOffsetY(), targetHeight);
    //targetDistance = Vision.getDistancefromY(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0), targetHeight);
    
    //System.out.println(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));
    //System.out.println("x angle: " + NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
    //System.out.println("y angle: " + NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0));
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    //System.out.println(20 / Math.tan(Math.toRadians(forwardCamera.angleOffsetY()-0.41)));

    //*****System.out.println(Vision.getDistancefromY(forwardCamera.angleOffsetY(), targetHeight));

    

    //  if(manualControlEnabled && stickID == 1){
    //    drive.driveTrainTeleop( stick.getRawAxis(0) * xSensitivity,-1 * stick.getRawAxis(1) * ySensitivity, stick.getRawAxis(2) * zSensitivity);
    //  } else if( manualControlEnabled && stickID == 3)
    //  {
    //    drive.driveTrainTeleop( stick.getRawAxis(0) * xSensitivity,-1 * stick.getRawAxis(1) * ySensitivity, stick.getRawAxis(4) * zSensitivity);
    //  }

    

    
    }
    
  

  @Override
  public void testInit() {
    
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void disabledInit() {
    System.out.println("*********************************************");
  }

  public static double quadraticPositionAndSpeed(double minimumMotorSpeed, double maximumMotorSpeed, double positionGoal, double currentPosition) {

    // position could be angle offset, encoder count, inches, etc.

    // maximum speed should be reached at the middle position

    // need to look at system of equations again and see if I want endpoint to just be positionGoal with speed 0. (position, 0)

    double a = ((positionGoal * maximumMotorSpeed - minimumMotorSpeed * positionGoal - minimumMotorSpeed * (positionGoal / 2) + minimumMotorSpeed * (positionGoal / 2)) / (positionGoal * (positionGoal / 2) * ((positionGoal / 2) - positionGoal)));

    double b = ((maximumMotorSpeed - a * (positionGoal / 2) * (positionGoal / 2) - minimumMotorSpeed) / (positionGoal / 2));

    double speed = a * currentPosition * currentPosition + b * currentPosition + minimumMotorSpeed;
    
    return speed; // value would be from 0.0 to 1.0 like any motor needs to be
  }

}
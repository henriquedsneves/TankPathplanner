// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralState;
import frc.robot.Constants.DriveTrainState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import com.pathplanner.lib.controllers.PPLTVController;


public class DriveTrainSystem extends SubsystemBase {
  public DriveTrainState currentState = DriveTrainState.STOPPED;
  private Pose2d currentPose; // Pose atual do robô
    private ChassisSpeeds currentSpeeds; // Velocidades atuais do robô

  SparkMax rightMotorFront = new SparkMax(Constants.DriveTrainConstants.rightFrontMotorID, MotorType.kBrushless);
  SparkMax rightMotorBack = new SparkMax(Constants.DriveTrainConstants.rightBackMotorID, MotorType.kBrushless);
  SparkMax leftMotorBack = new SparkMax(Constants.DriveTrainConstants.leftBackMotorID, MotorType.kBrushless);
  SparkMax leftMotorFront = new SparkMax(Constants.DriveTrainConstants.leftFrontMotorID, MotorType.kBrushless);

  Pigeon2 pigeon2 = new Pigeon2(17);
  private final PIDController rotationPID = new PIDController(0.0733, 2, 0.0042);

  SparkMaxConfig configSparkRight = new SparkMaxConfig();
  SparkMaxConfig configSparkLeft = new SparkMaxConfig();

  MotorControllerGroup leftMotorControllerGroup = new MotorControllerGroup(leftMotorFront, leftMotorBack);
  MotorControllerGroup rightMotorControllerGroup = new MotorControllerGroup(rightMotorFront, rightMotorBack);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorControllerGroup, rightMotorControllerGroup);

  SlewRateLimiter filter = new SlewRateLimiter(10);

  public DriveTrainSystem() {

    rotationPID.setIntegratorRange(-1.0, 1.0);
    rotationPID.setTolerance(0.1);

    configSparkRight
      .inverted(true)
      .idleMode(IdleMode.kBrake);

    configSparkLeft
      .inverted(false)
      .idleMode(IdleMode.kBrake);

    configSparkRight.smartCurrentLimit(30);
    configSparkLeft.smartCurrentLimit(30);
  
    rightMotorFront.configure(configSparkRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotorBack.configure(configSparkRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftMotorFront.configure(configSparkLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftMotorBack.configure(configSparkLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.currentPose = new Pose2d(); // Pose inicial em (0, 0) com rotação 0
    this.currentSpeeds = new ChassisSpeeds(); 
    //setupPathplanner();
  }
  /*public void setupPathplanner(){
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
  
    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
  
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );*/

  


  @Override
  public void periodic() {
    // System.out.println(AnglePideon());
  }

  public void arcadeMode(double drive, double turn){
    differentialDrive.arcadeDrive(filter.calculate(drive), turn);
  }

  public void tankmode(double left, double right){
    differentialDrive.tankDrive(left, right);
  }
  public void stop(){
    differentialDrive.stopMotor(); 
  }

  public void SetCurrentState(DriveTrainState state){
    this.currentState = state;
  }

  public double AnglePideon(){
    return pigeon2.getAngle();
 }

  public void resetPideon(){
    pigeon2.reset();
  }
   public Pose2d getPose() {
        return currentPose;
    }
    public void resetPose(Pose2d pose) {
      this.currentPose = pose;
  }
  public ChassisSpeeds getRobotRelativeSpeeds() {
        return currentSpeeds;
    }
    public void driveRobotRelative(ChassisSpeeds speeds) {
      // Atualiza as velocidades atuais
      this.currentSpeeds = speeds;
    }
     
  public void rotate(double targetAngle){
    rotationPID.setSetpoint(targetAngle);
    double output = rotationPID.calculate(AnglePideon());
    double scaledOutput = output * 0.125;

    tankmode(scaledOutput, -scaledOutput);
  }
}


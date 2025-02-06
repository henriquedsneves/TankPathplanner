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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainState;
import frc.robot.Constants.ModeElevator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;


public class DriveTrainSystem extends SubsystemBase {
  private Pose2d currentPose = new Pose2d(); // Armazena a pose do robô
  public DriveTrainState currentState = DriveTrainState.STOPPED;
  private ChassisSpeeds currentSpeeds = new ChassisSpeeds();
  SparkMax rightMotorFront = new SparkMax(Constants.DriveTrainConstants.rightFrontMotorID, MotorType.kBrushed);
  SparkMax rightMotorBack = new SparkMax(Constants.DriveTrainConstants.rightBackMotorID, MotorType.kBrushed);
  SparkMax leftMotorBack = new SparkMax(Constants.DriveTrainConstants.leftBackMotorID, MotorType.kBrushed);
  SparkMax leftMotorFront = new SparkMax(Constants.DriveTrainConstants.leftFrontMotorID, MotorType.kBrushed);

  Pigeon2 pigeon2 = new Pigeon2(17);
  private final PIDController rotationPID = new PIDController(0.08, 2, 0.01);

  SparkMaxConfig configSparkRight = new SparkMaxConfig();
  SparkMaxConfig configSparkLeft = new SparkMaxConfig();

  MotorControllerGroup leftMotorControllerGroup = new MotorControllerGroup(leftMotorFront, leftMotorBack);
  MotorControllerGroup rightMotorControllerGroup = new MotorControllerGroup(rightMotorFront, rightMotorBack);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorControllerGroup, rightMotorControllerGroup);

  SlewRateLimiter filter = new SlewRateLimiter(50);

  public DriveTrainSystem() {

    rotationPID.setIntegratorRange(-1.0, 1.0);
    rotationPID.setTolerance(0.1);

    configSparkRight
      .inverted(true)
      .idleMode(IdleMode.kBrake);

    configSparkLeft
      .inverted(false)
      .idleMode(IdleMode.kBrake);

    configSparkRight.smartCurrentLimit(60);
    configSparkLeft.smartCurrentLimit(60);
  
    rightMotorFront.configure(configSparkRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotorBack.configure(configSparkRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftMotorFront.configure(configSparkLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftMotorBack.configure(configSparkLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setupPathplanner();
  }

  public void setupPathplanner(){
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      return;
    }

    AutoBuilder.configure(
            this::getPose, // Método que retorna a pose atual do robô
            this::resetPose, // Método para resetar a odometria
            this::getRobotRelativeSpeeds, // Retorna a velocidade do robô em ChassisSpeeds
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Define a movimentação do robô
            new PPLTVController(0.02), // Controlador de trajetória para Tank Drive
            config, // Configuração do robô
            () -> {
              var alliance = DriverStation.getAlliance();
              return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            },
            this // Adiciona dependência ao subsistema
    );
  }

  @Override
  public void periodic() {
  }

  public void arcadeMode(double drive, double turn){
    differentialDrive.arcadeDrive(filter.calculate(drive), -turn);
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
    return currentPose; // Substituir pelo método real de odometria
  }

  public void resetPose(Pose2d newPose) {
    this.currentPose = newPose;
    // Adicione aqui o reset real da odometria se necessário
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return currentSpeeds; // Substituir pela leitura real da cinemática do robô
  }
 

  public void driveRobotRelative(ChassisSpeeds speeds) {
    double leftSpeed = speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond;
    double rightSpeed = speeds.vxMetersPerSecond + speeds.omegaRadiansPerSecond;

    tankmode(leftSpeed, rightSpeed);
  }

   public Command getAutonomousCommand(String pathName, boolean setOdomToStart) {
    return new PathPlannerAuto(pathName);
  }

  public void followLine(double targetSpeed){
    double currentAngle = AnglePideon();
    double targetAngle = 0;

    double rotationOutput = rotationPID.calculate(currentAngle, targetAngle);
    double leftSpeed = targetSpeed + rotationOutput;
    double rightSpeed = targetSpeed - rotationOutput;

    leftSpeed = Math.max(-1.0, Math.min(1.0, leftSpeed));
    rightSpeed = Math.max(-1.0, Math.min(1.0, rightSpeed));

    tankmode(-rightSpeed, -leftSpeed);
  }
}

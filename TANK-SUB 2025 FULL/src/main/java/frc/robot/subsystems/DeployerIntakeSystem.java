// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DeployerState;
import frc.robot.Constants.IntakeState;
import frc.robot.commands.Manipulator.GetSensor;
import frc.robot.commands.Manipulator.SetMechanismState;
import edu.wpi.first.wpilibj.AnalogInput;

public class DeployerIntakeSystem extends SubsystemBase {
  public SparkMax deployerLeftMotor = new SparkMax(Constants.DeployerIntakeConstants.deployerLeftMotorID, MotorType.kBrushless);
  public SparkMax deployerRightMotor = new SparkMax(Constants.DeployerIntakeConstants.deployerRightMotorID, MotorType.kBrushless);

  SparkMaxConfig configSparkMotorLeft = new SparkMaxConfig();
  SparkMaxConfig configSparkMotorRight = new SparkMaxConfig();

  private GetSensor getSensor = new GetSensor();

  public DeployerState stateDeployer;
  public IntakeState stateIntake = IntakeState.OPERATION;

  private final AnalogInput deployerSensor;
  private final AnalogInput intakeSensor;

  public DeployerIntakeSystem() {
    configSparkMotorLeft
      .inverted(true)
      
      .idleMode(IdleMode.kBrake);
    
    configSparkMotorRight
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    
    configSparkMotorRight.follow(Constants.DeployerIntakeConstants.deployerLeftMotorID, true);

    deployerLeftMotor.configure(configSparkMotorLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    deployerRightMotor.configure(configSparkMotorRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    deployerSensor = new AnalogInput(Constants.DeployerIntakeConstants.deployerSensor);
    intakeSensor = new AnalogInput(Constants.DeployerIntakeConstants.intakeSensor);
  }

  @Override
  public void periodic() {
    // System.out.println(getSensor.getAnalogSensorDistance(deployerSensor));

    if(stateIntake.operation && !getSensor.getAnalogSensorGamePiece(deployerSensor)){
      deployerLeftMotor.set(Constants.DeployerIntakeConstants.intakeSpeed);
    } else if(stateIntake.operation && getSensor.getAnalogSensorGamePiece(deployerSensor)){
      CommandScheduler.getInstance().schedule(new SetMechanismState(IntakeState.STOPPED));
    } else if(stateDeployer == DeployerState.SHOOTING){
      deployerLeftMotor.set(stateDeployer.speed);
    } else{
      deployerLeftMotor.stopMotor();
    }

    // if(stateDeployer == DeployerState.SHOOTING || stateIntake.operation){
    //   deployerLeftMotor.set(stateDeployer.speed);
    // } else{
    //   deployerLeftMotor.stopMotor();
    // }
  }

  public void SetCurrentStateDeployer(DeployerState state){
    this.stateDeployer = state;
  }

  public void SetCurrentStateIntake(IntakeState state){
    this.stateIntake = state;
  }
}

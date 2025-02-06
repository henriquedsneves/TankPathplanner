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
import frc.robot.Constants.ClimberState;
import frc.robot.Constants.DeployerState;
import frc.robot.Constants.IntakeState;
import frc.robot.Constants.PivoState;
import frc.robot.commands.Manipulator.GetSensor;
import frc.robot.commands.Manipulator.SetMechanismState;
import edu.wpi.first.wpilibj.AnalogInput;

public class ClimberSystem extends SubsystemBase {
  // public SparkMax climberMotor = new SparkMax(Constants.ClimberConstants.climberMotorID, MotorType.kBrushless);

  // SparkMaxConfig configSparkMotor = new SparkMaxConfig();

  // public ClimberState stateClimber = ClimberState.STOPPED;

  // public ClimberSystem() {
  //   configSparkMotor
  //     .inverted(true)
  //     .idleMode(IdleMode.kBrake);

  //   configSparkMotor.smartCurrentLimit(80);

  //   climberMotor.configure(configSparkMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  // }

  // @Override
  // public void periodic() {
  //   if(stateClimber == ClimberState.OPENING || stateClimber == ClimberState.CLOSING){
  //     climberMotor.set(stateClimber.speed);
  //   } else{
  //     climberMotor.stopMotor();
  //   }
  // }

  // public void SetCurrentStateClimber(ClimberState state){
  //   this.stateClimber = state;
  // }
}

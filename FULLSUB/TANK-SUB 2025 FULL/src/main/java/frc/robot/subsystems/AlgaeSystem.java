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
import frc.robot.Constants.AlgaeState;
import frc.robot.Constants.DeployerState;
import frc.robot.Constants.IntakeState;
import frc.robot.Constants.PivoState;
import frc.robot.commands.Manipulator.GetSensor;
import frc.robot.commands.Manipulator.SetMechanismState;
import edu.wpi.first.wpilibj.AnalogInput;

public class AlgaeSystem extends SubsystemBase {
  // public SparkMax armMotor = new SparkMax(Constants.AlgaeConstants.armMotorID, MotorType.kBrushless);
  // public SparkMax intakeMotor = new SparkMax(Constants.AlgaeConstants.intakeMotorID, MotorType.kBrushless);

  // SparkMaxConfig configSparkMotorArm = new SparkMaxConfig();
  // SparkMaxConfig configSparkMotorIntake = new SparkMaxConfig();

  // public AlgaeState stateAlgae = AlgaeState.STOPPED;

  // public AlgaeSystem() {
  //   configSparkMotorArm
  //     .inverted(true)
  //     .idleMode(IdleMode.kBrake);

  //   configSparkMotorIntake
  //     .inverted(false)
  //     .idleMode(IdleMode.kBrake);

  //   configSparkMotorArm.smartCurrentLimit(60);
  //   configSparkMotorIntake.smartCurrentLimit(20);

  //   configSparkMotorIntake.follow(Constants.AlgaeConstants.armMotorID);

  //   armMotor.configure(configSparkMotorArm, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  //   intakeMotor.configure(configSparkMotorIntake, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  // }

  // @Override
  // public void periodic() {
  //   if(stateAlgae == AlgaeState.OPENING || stateAlgae == AlgaeState.CLOSING){
  //     armMotor.set(stateAlgae.speed);
  //   } else{
  //     armMotor.stopMotor();
  //   }
  // }

  // public void SetCurrentStateAlgae(AlgaeState state){
  //   this.stateAlgae = state;
  // }
}

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
import frc.robot.Constants.ArmState;
import frc.robot.Constants.DeployerState;
import frc.robot.Constants.IntakeState;
import frc.robot.Constants.PivoState;
import frc.robot.commands.Manipulator.GetSensor;
import frc.robot.commands.Manipulator.SetMechanismState;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;

public class ArmSystem extends SubsystemBase {
  // public SparkMax armMotor = new SparkMax(Constants.ArmConstants.armMotorID, MotorType.kBrushless);
  // public SparkMax secondArmMotor = new SparkMax(Constants.ArmConstants.secondArmMotorID, MotorType.kBrushless);

  // SparkMaxConfig configSparkMotorArm = new SparkMaxConfig();
  // SparkMaxConfig configSparkMotorSecondArm = new SparkMaxConfig();

  // private PIDController pidController;
  // private ElevatorFeedforward feedforward;

  // public ArmState stateArm = ArmState.STOPPED;

  // private boolean inTolerance = false;
  // private boolean inToleranceRest = false;

  // public ArmSystem() {
  //   configSparkMotorArm
  //     .inverted(true)
  //     .idleMode(IdleMode.kBrake);

  //   configSparkMotorSecondArm
  //     .inverted(true)
  //     .idleMode(IdleMode.kBrake);

  //   configSparkMotorArm.smartCurrentLimit(60);
  //   configSparkMotorSecondArm.smartCurrentLimit(80);

  //   armMotor.configure(configSparkMotorArm, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  //   secondArmMotor.configure(configSparkMotorSecondArm, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  //   pidController = new PIDController(Constants.ArmConstants.kP,Constants.ArmConstants.kI,Constants.ArmConstants.kD);
  //   pidController.setTolerance(Constants.ElevatorConstants.PID_TOLERANCE);

  //   feedforward = new ElevatorFeedforward(Constants.ArmConstants.kS, Constants.ArmConstants.kG, Constants.ArmConstants.kV);

  //   armMotor.getEncoder().setPosition(0);
  // }

  // @Override
  // public void periodic() {
  //   pidController.setTolerance(Constants.ElevatorConstants.PID_TOLERANCE);

  //   if(stateArm == ArmState.OPERATION && !inTolerance){
  //     pidController.setSetpoint(Constants.ArmConstants.ARM_SETPOINT);
  //     inTolerance = pidController.atSetpoint();
  //     ArmPID(Constants.ArmConstants.ARM_SETPOINT);    
  //   } else if(inTolerance && !inToleranceRest){
  //     pidController.setSetpoint(Constants.ArmConstants.ARM_SETPOINT_REST);
  //     inToleranceRest = pidController.atSetpoint();
  //     ArmPID(Constants.ArmConstants.ARM_SETPOINT_REST);
  //   } else if(inToleranceRest){
  //     CommandScheduler.getInstance().schedule(new SetMechanismState(ArmState.STOPPED));
  //     inTolerance = false;
  //     inToleranceRest = false;
  //   }

  //   if (stateArm == ArmState.OPERATION){
  //     secondArmMotor.set(Constants.ArmConstants.SECOND_ARM_SPEED);
  //   } else{
  //     secondArmMotor.stopMotor();
  //   }

  //   System.out.println(stateArm.state);
  //   System.out.println(armMotor.getEncoder().getPosition());
  //   System.out.println(inTolerance);
  //   System.out.println(inToleranceRest);
  // }

  // public void ArmPID(double setPoint){
  //   double pidOutput = pidController.calculate(armMotor.getEncoder().getPosition(), setPoint);
  //   double feedforwardOutput = feedforward.calculate(armMotor.getEncoder().getPosition(), armMotor.getEncoder().getVelocity());
  //   double speed = pidOutput + feedforwardOutput;

  //   armMotor.set(speed * Constants.ArmConstants.ARM_SPEED);
  // }

  // public void SetCurrentStateArm(ArmState state){
  //   this.stateArm = state;
  // }
}

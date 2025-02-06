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
import frc.robot.RobotContainer;
import frc.robot.Constants.DeployerState;
import frc.robot.Constants.ElevatorState;
import frc.robot.Constants.IntakeState;
import frc.robot.Constants.ModeElevator;
import frc.robot.commands.Manipulator.GetSensor;
import frc.robot.commands.Manipulator.SetMechanismState;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;

public class ElevatorSystem extends SubsystemBase {
  public SparkMax elevatorLeftMotor = new SparkMax(Constants.ElevatorConstants.elevatorLeftMotorID, MotorType.kBrushless);
  public SparkMax elevatorRightMotor = new SparkMax(Constants.ElevatorConstants.elevatorRightMotorID, MotorType.kBrushless);

  private RelativeEncoder elevatorEncoder = elevatorLeftMotor.getEncoder();

  SparkMaxConfig configSparkMotorLeft = new SparkMaxConfig();
  SparkMaxConfig configSparkMotorRight = new SparkMaxConfig();

  private PIDController pidController;
  private ElevatorFeedforward feedforward;

  private GetSensor getSensor = new GetSensor();

  public ElevatorState stateElevator = ElevatorState.L1;
  public ModeElevator modeElevator = ModeElevator.MANUAL;

  private DigitalInput limitSwitchPort1;

  public ElevatorSystem() {
    configSparkMotorLeft
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    
    configSparkMotorRight
      .inverted(true)
      .idleMode(IdleMode.kBrake);
    
    configSparkMotorRight.follow(Constants.ElevatorConstants.elevatorLeftMotorID, true);

    configSparkMotorLeft.smartCurrentLimit(50);
    configSparkMotorRight.smartCurrentLimit(50);

    elevatorLeftMotor.configure(configSparkMotorLeft, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorRightMotor.configure(configSparkMotorRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pidController = new PIDController(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD);
    pidController.setTolerance(Constants.ElevatorConstants.PID_TOLERANCE);

    feedforward = new ElevatorFeedforward(Constants.ElevatorConstants.kS, Constants.ElevatorConstants.kG, Constants.ElevatorConstants.kV);

    limitSwitchPort1 = new DigitalInput(Constants.ElevatorConstants.limitSwitchPort1);

  }

  @Override
  public void periodic() {
    // System.out.println(getSensor.getDigitalSensor(limitSwitchPort1));
    System.out.println(getElevatorEncoderPosition());

    pidController.setTolerance(Constants.ElevatorConstants.PID_TOLERANCE);
    pidController.setSetpoint(stateElevator.position);

    double pidOutput = pidController.calculate(elevatorLeftMotor.getEncoder().getPosition(), stateElevator.position);
    double feedforwardOutput = feedforward.calculate(elevatorLeftMotor.getEncoder().getPosition(), elevatorLeftMotor.getEncoder().getVelocity());
    double speed = pidOutput + feedforwardOutput;

    if(!modeElevator.mode){
      elevatorLeftMotor.set(speed * Constants.ElevatorConstants.ELEVATOR_SPEED);
    }

    if(!getSensor.getDigitalSensor(limitSwitchPort1)){
      zeroElevator();
    }
  }

  public void moveElevatorManual(double speed){
    if(modeElevator.mode && getSensor.getDigitalSensor(limitSwitchPort1)){
      elevatorLeftMotor.set(-speed * Constants.ElevatorConstants.ELEVATOR_MANUAL_SPEED);
    } else if(modeElevator.mode){
      stopElevator();
    }
  }

  public double getElevatorEncoderPosition() {
    return elevatorEncoder.getPosition();
  }

  public void setElevatorEncoderPosition(double position) {
    elevatorEncoder.setPosition(position);
  }

  public void stopElevator()
  {
    elevatorLeftMotor.stopMotor();
  }

  public void zeroElevator()
  {
    elevatorEncoder.setPosition(0);
  }

  public void SetCurrentStateElevator(ElevatorState state){
    this.stateElevator = state;
  }

  public void SetCurrentModeElevator(ModeElevator state){
    this.modeElevator = state;
  }
}

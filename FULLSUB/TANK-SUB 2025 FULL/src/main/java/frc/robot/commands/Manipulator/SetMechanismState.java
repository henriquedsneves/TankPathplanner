// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manipulator;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AlgaeState;
import frc.robot.Constants.ArmState;
import frc.robot.Constants.ClimberState;
import frc.robot.Constants.DeployerState;
import frc.robot.Constants.DriveTrainState;
import frc.robot.Constants.ElevatorState;
import frc.robot.Constants.IntakeState;
import frc.robot.Constants.ModeElevator;
import frc.robot.Constants.PivoState;

/** An example command that uses an example subsystem. */
public class SetMechanismState extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  public SetMechanismState(){
    
  }

    private DriveTrainState currentDriveState = DriveTrainState.STOPPED;

    private DeployerState currentStateDeployer = DeployerState.STOPPED;
    private IntakeState currentStateIntake = IntakeState.STOPPED;

    private ElevatorState currentStateElevator = ElevatorState.L1;
    private ModeElevator currentModeElevator = ModeElevator.AUTOMATIC;

    private PivoState currentStatePivo = PivoState.STOPPED;

    private ClimberState currentStateClimber = ClimberState.STOPPED;

    private AlgaeState currentStateAlgae = AlgaeState.STOPPED;

    private ArmState currentStateArm = ArmState.STOPPED;

    boolean currentDriveTrainOnly = false;

    boolean currentStateDeployerOnly = false;
    boolean currentStateIntakeOnly = false;

    boolean currentStateElevatorOnly = false;
    boolean currentModeElevatorOnly = false;

    boolean currentStatePivoOnly = false;

    boolean currentStateClimberOnly = false;

    boolean currentStateAlgaeOnly = false;

    boolean currentStateArmOnly = false;

    public SetMechanismState(DriveTrainState state) {
      addRequirements(RobotContainer.driveTrainSystem);
      this.currentDriveState = state;
      currentDriveTrainOnly = true;
    }

    public SetMechanismState(ElevatorState state) {
      addRequirements(RobotContainer.elevatorSystem);
      this.currentStateElevator = state;
      currentStateElevatorOnly = true;
    }

    public SetMechanismState(ModeElevator state) {
      addRequirements(RobotContainer.elevatorSystem);
      this.currentModeElevator = state;
      currentModeElevatorOnly = true;
    }

    public SetMechanismState(DeployerState state) {
      addRequirements(RobotContainer.deployerIntakeSystem);
      this.currentStateDeployer = state;
      currentStateDeployerOnly = true;
    }

    public SetMechanismState(IntakeState state) {
      addRequirements(RobotContainer.deployerIntakeSystem);
      this.currentStateIntake = state;
      currentStateIntakeOnly = true;
    }

    public SetMechanismState(PivoState state) {
      addRequirements(RobotContainer.pivoSystem);
      this.currentStatePivo = state;
      currentStatePivoOnly = true;
    }

    public SetMechanismState(ClimberState state) {
      addRequirements(RobotContainer.climberSystem);
      this.currentStateClimber = state;
      currentStateClimberOnly = true;
    }

    public SetMechanismState(AlgaeState state) {
      addRequirements(RobotContainer.algaeSystem);
      this.currentStateAlgae = state;
      currentStateAlgaeOnly = true;
    }

    public SetMechanismState(ArmState state) {
      addRequirements(RobotContainer.armSystem);
      this.currentStateArm = state;
      currentStateArmOnly = true;
    }

  @Override
  public void execute() {
    if(currentDriveTrainOnly){
      RobotContainer.driveTrainSystem.SetCurrentState(this.currentDriveState);
    } 
    
    if(currentStateElevatorOnly){
      RobotContainer.elevatorSystem.SetCurrentStateElevator(this.currentStateElevator);
    } else if(currentModeElevatorOnly){
      RobotContainer.elevatorSystem.SetCurrentModeElevator(this.currentModeElevator);
    }
    
    if(currentStateDeployerOnly) {
      RobotContainer.deployerIntakeSystem.SetCurrentStateDeployer(this.currentStateDeployer);
    } else if(currentStateIntakeOnly){
      RobotContainer.deployerIntakeSystem.SetCurrentStateIntake(this.currentStateIntake);
    } 
    
    // else if(currentStatePivoOnly){
    //   RobotContainer.pivoSystem.SetCurrentStatePivo(this.currentStatePivo);
    // } else if(currentStateClimberOnly){
    //   RobotContainer.climberSystem.SetCurrentStateClimber(this.currentStateClimber);
    // } else if(currentStateAlgaeOnly){
    //   RobotContainer.algaeSystem.SetCurrentStateAlgae(this.currentStateAlgae);
    // } else if(currentStateArmOnly){
    //   RobotContainer.armSystem.SetCurrentStateArm(this.currentStateArm);
    // }
  }
}

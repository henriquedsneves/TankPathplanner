// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manipulator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.CoralState;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.DriveTrainState;

/** An example command that uses an example subsystem. */
public class SetMechanismState extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private CoralState currentStateCoral = CoralState.STOPPED;
    private DriveTrainState currentStateDriveTrain = DriveTrainState.STOPPED;

    boolean currentStateCoralOnly = false;
    boolean currentStateDriveTrainOnly = false;

    public SetMechanismState(CoralState state) {
      this.currentStateCoral = state;
      addRequirements(RobotContainer.coralScoreSystem);
      currentStateCoralOnly = true;
    }

    public SetMechanismState(DriveTrainState state) {
      this.currentStateDriveTrain = state;
      addRequirements(RobotContainer.driveTrainSystem);
      currentStateDriveTrainOnly = true;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(currentStateCoralOnly) {
      RobotContainer.coralScoreSystem.SetCurrentState(this.currentStateCoral);
    } else if(currentStateDriveTrainOnly){
      RobotContainer.driveTrainSystem.SetCurrentState(this.currentStateDriveTrain);
    }
  }
}

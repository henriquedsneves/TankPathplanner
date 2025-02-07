// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DeployerState;

/** An example command that uses an example subsystem. */
public class SetActions extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public Command PontuationL2(){

      return new SetMechanismState(DeployerState.SHOOTING).
          andThen(new WaitCommand(3).
          andThen(new SetMechanismState(DeployerState.STOPPED)));
    }
    public void pontuar(){
      System.out.println("Pontuando"); 
    }

    public void soltarAlga(){
      System.err.println("Soltando Alga");
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
}

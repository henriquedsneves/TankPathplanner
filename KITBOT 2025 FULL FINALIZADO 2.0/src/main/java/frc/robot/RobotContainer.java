// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.CoralState;
import frc.robot.Constants.DriveTrainState;
//import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.RotatioGyro;
import frc.robot.commands.Manipulator.SetMechanismState;
import frc.robot.subsystems.DriveTrainSystem;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.subsystems.CoralScoreSystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final DriveTrainSystem driveTrainSystem = new DriveTrainSystem();
  public static final CoralScoreSystem coralScoreSystem = new CoralScoreSystem();

  CommandXboxController joystick1 = new CommandXboxController(0);
  CommandXboxController joystick2 = new CommandXboxController(1);
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController m_driverController =
      //new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
    defaultcommands();
  }

  private void configureBindings() {
    joystick1.button(1).whileTrue(new SetMechanismState(DriveTrainState.STOPPED));
    joystick1.button(5).whileTrue(new SetMechanismState(DriveTrainState.MID));
    joystick1.button(6).whileTrue(new SetMechanismState(DriveTrainState.FULL));

    joystick1.button(4).whileTrue(new RotatioGyro(driveTrainSystem));
    joystick1.button(2).whileTrue(new SetMechanismState(CoralState.SHOOTING)).onFalse(new SetMechanismState(CoralState.STOPPED));
  }

  private void defaultcommands(){
    driveTrainSystem.setDefaultCommand(new DriveWithJoystick(driveTrainSystem, joystick1));
  }
}

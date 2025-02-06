// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AlgaeState;
import frc.robot.Constants.ArmState;
import frc.robot.Constants.ClimberState;
import frc.robot.Constants.DeployerState;
import frc.robot.Constants.DriveTrainState;
import frc.robot.Constants.ElevatorState;
import frc.robot.Constants.IntakeState;
import frc.robot.Constants.ModeElevator;
import frc.robot.Constants.PivoState;
import frc.robot.Constants.Trajetoria;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.moveElevatorCommand;
import frc.robot.commands.Manipulator.SetActions;
import frc.robot.commands.Manipulator.SetMechanismState;
import frc.robot.subsystems.AlgaeSystem;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.ClimberSystem;
import frc.robot.subsystems.DeployerIntakeSystem;
import frc.robot.subsystems.DriveTrainSystem;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.PivoSystem;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static final DriveTrainSystem driveTrainSystem = new DriveTrainSystem();
  public static final moveElevatorCommand moveElevator  = new moveElevatorCommand();
  public static final SetMechanismState mechanism = new SetMechanismState();
  public static final SetActions actions = new SetActions();

  public static final ElevatorSystem elevatorSystem = new ElevatorSystem();
  public static final DeployerIntakeSystem deployerIntakeSystem = new DeployerIntakeSystem();
  public static final PivoSystem pivoSystem = new PivoSystem();
  public static final ClimberSystem climberSystem = new ClimberSystem();
  public static final AlgaeSystem algaeSystem = new AlgaeSystem();
  public static final ArmSystem armSystem = new ArmSystem();

  CommandXboxController joystick1 = new CommandXboxController(1);
  CommandXboxController joystick2 = new CommandXboxController(0);
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController m_driverController =
  //new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("L1", 
    new SequentialCommandGroup(
       new InstantCommand(() -> driveTrainSystem.stop(), driveTrainSystem),
       new InstantCommand(() -> moveElevator.L1(), driveTrainSystem),
       new InstantCommand(() -> actions.pontuar(), driveTrainSystem),
       new InstantCommand(() -> moveElevator.repouso(), driveTrainSystem)
    
    )
    );

    NamedCommands.registerCommand("L2", 
    new SequentialCommandGroup(
        new InstantCommand(() -> driveTrainSystem.stop(), driveTrainSystem),
        new InstantCommand(() -> moveElevator.L2(), driveTrainSystem),
        new InstantCommand(() -> actions.pontuar(), driveTrainSystem),
        new InstantCommand(() -> moveElevator.repouso(), driveTrainSystem)
       )
    );

    NamedCommands.registerCommand("L3", 
    new SequentialCommandGroup(
        new InstantCommand(() -> driveTrainSystem.stop(), driveTrainSystem),
        new InstantCommand(() -> moveElevator.L3(), driveTrainSystem),
        new InstantCommand(() -> actions.pontuar(), driveTrainSystem),
        new InstantCommand(() -> moveElevator.repouso(), driveTrainSystem)
 
    )
 );
    
    
    NamedCommands.registerCommand("L4", 
   new SequentialCommandGroup(
       new InstantCommand(() -> driveTrainSystem.stop(), driveTrainSystem),
       new InstantCommand(() -> moveElevator.L4(), driveTrainSystem),
       new InstantCommand(() -> actions.pontuar(), driveTrainSystem),
       new InstantCommand(() -> moveElevator.repouso(), driveTrainSystem)

   )
);
    
     NamedCommands.registerCommand("L4_AlgeL2", 
    new SequentialCommandGroup(
        new InstantCommand(() -> driveTrainSystem.stop(), driveTrainSystem),
        new InstantCommand(() -> moveElevator.L2(), driveTrainSystem),
        new InstantCommand(() -> actions.soltarAlga(), driveTrainSystem),
        new InstantCommand(() -> moveElevator.L4(), driveTrainSystem),
        new InstantCommand(() -> actions.pontuar(), driveTrainSystem),
        new InstantCommand(() -> moveElevator.repouso(), driveTrainSystem)

    )
);
     NamedCommands.registerCommand("L4_AlgeL3", 
    new SequentialCommandGroup(
        new InstantCommand(() -> driveTrainSystem.stop(), driveTrainSystem),
        new InstantCommand(() -> moveElevator.L3(), driveTrainSystem),
        new InstantCommand(() -> actions.soltarAlga(), driveTrainSystem),
        new InstantCommand(() -> moveElevator.L4(), driveTrainSystem),
        new InstantCommand(() -> actions.pontuar(), driveTrainSystem),
        new InstantCommand(() -> moveElevator.repouso(), driveTrainSystem)

    )
);
     NamedCommands.registerCommand("L3_AlgeL2", 
    new SequentialCommandGroup(
        new InstantCommand(() -> driveTrainSystem.stop(), driveTrainSystem),
        new InstantCommand(() -> moveElevator.L2(), driveTrainSystem),
        new InstantCommand(() -> actions.soltarAlga(), driveTrainSystem),
        new InstantCommand(() -> moveElevator.L3(), driveTrainSystem),
        new InstantCommand(() -> actions.pontuar(), driveTrainSystem),
        new InstantCommand(() -> moveElevator.repouso(), driveTrainSystem)

    )
);
     NamedCommands.registerCommand("L3_AlgeL3", 
    new SequentialCommandGroup(
        new InstantCommand(() -> driveTrainSystem.stop(), driveTrainSystem),
        new InstantCommand(() -> moveElevator.L3(), driveTrainSystem),
        new InstantCommand(() -> actions.soltarAlga(), driveTrainSystem),
        new InstantCommand(() -> actions.pontuar(), driveTrainSystem),
        new InstantCommand(() -> moveElevator.repouso(), driveTrainSystem)

    )
);
     NamedCommands.registerCommand("L2_AlgeL2", 
    new SequentialCommandGroup(
        new InstantCommand(() -> driveTrainSystem.stop(), driveTrainSystem),
        new InstantCommand(() -> moveElevator.L2(), driveTrainSystem),
        new InstantCommand(() -> actions.soltarAlga(), driveTrainSystem),
        new InstantCommand(() -> actions.pontuar(), driveTrainSystem),
        new InstantCommand(() -> moveElevator.repouso(), driveTrainSystem)

    )
);
    
      
    configureBindings();
    defaultcommands();
  }

  private void configureBindings() {

    joystick1.button(1).whileTrue(new SetMechanismState(DriveTrainState.STOPPED));
    joystick1.button(5).whileTrue(new SetMechanismState(DriveTrainState.MID));
    joystick1.button(6).whileTrue(new SetMechanismState(DriveTrainState.FULL));

    joystick2.rightTrigger().whileTrue(new SetMechanismState(DeployerState.SHOOTING)).onFalse(new SetMechanismState(DeployerState.STOPPED));
    joystick2.leftTrigger().whileTrue(new SetMechanismState(IntakeState.OPERATION));

    joystick2.button(5).onTrue(new SetMechanismState(ElevatorState.CORAL).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
    joystick2.button(1).onTrue(new SetMechanismState(ElevatorState.L1).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
    joystick2.button(2).onTrue(new SetMechanismState(ElevatorState.L2).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
    joystick2.button(4).onTrue(new SetMechanismState(ElevatorState.L3).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));
    joystick2.button(3).onTrue(new SetMechanismState(ElevatorState.L4).andThen(new SetMechanismState(ModeElevator.AUTOMATIC)));

    // joystick2.rightTrigger().onTrue(new SetMechanismState(AlgaeState.OPENING)).onFalse(new SetMechanismState(AlgaeState.STOPPED));
    // joystick2.leftTrigger().onTrue(new SetMechanismState(AlgaeState.CLOSING)).onFalse(new SetMechanismState(AlgaeState.STOPPED));

    // joystick2.povUp().whileTrue(new SetMechanismState(PivoState.OPENING)).onFalse(new SetMechanismState(PivoState.STOPPED));
    // joystick2.povDown().whileTrue(new SetMechanismState(PivoState.CLOSING)).onFalse(new SetMechanismState(PivoState.STOPPED));

    // joystick1.rightTrigger().whileTrue(new SetMechanismState(ClimberState.OPENING)).onFalse(new SetMechanismState(ClimberState.STOPPED));
    // joystick1.leftTrigger().whileTrue(new SetMechanismState(ClimberState.CLOSING)).onFalse(new SetMechanismState(ClimberState.STOPPED));

    // joystick2.button(8).onTrue(new SetMechanismState(ArmState.OPERATION));
  }

  private void defaultcommands(){
    driveTrainSystem.setDefaultCommand(new DriveWithJoystick(driveTrainSystem, joystick1));
    elevatorSystem.setDefaultCommand(new moveElevatorCommand(joystick2, elevatorSystem));
  }

   public Command Auto() {
    // Aqui retornamos o comando que está no selecionador
    return driveTrainSystem.getAutonomousCommand(Trajetoria.nome_Trajetoria1, true);
  }
   public Command AutoTest() {
    // Aqui retornamos o comando que está no selecionador
    return driveTrainSystem.getAutonomousCommand(Trajetoria.nome_Trajetoria2, true);
  }
}

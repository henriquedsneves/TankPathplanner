package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.IntakeState;
import frc.robot.Constants.ModeElevator;
import frc.robot.commands.Manipulator.SetMechanismState;
import frc.robot.subsystems.ElevatorSystem;

public class moveElevatorCommand extends Command
{
    private ElevatorSystem elevatorSubsystem;
    private CommandXboxController joystick2;

    public moveElevatorCommand(CommandXboxController joystick2, ElevatorSystem elevatorSubsystem)
    {
        this.elevatorSubsystem = elevatorSubsystem;
        this.joystick2 = joystick2;
        addRequirements(elevatorSubsystem);
    }
    public moveElevatorCommand(){

    }

    public void L1(){
        System.out.println("Subindo para o L1");
    }
    public void L2(){
        System.out.println("Subindo para o L2");
    }
    public void L3(){
        System.out.println("Subindo para o L3");
    }
    public void L4(){
        System.out.println("Subindo para o L4");
    }
    public void repouso(){
        System.out.println("indo para o repouso");
    }

    @Override
    public void execute()
    {
        if(Math.abs(joystick2.getLeftY()) > 0.1){
            elevatorSubsystem.moveElevatorManual(joystick2.getLeftY());
            CommandScheduler.getInstance().schedule(new SetMechanismState(ModeElevator.MANUAL));
        } else{
            elevatorSubsystem.moveElevatorManual(0);
        }
    }
}

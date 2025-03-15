package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSystem;

public class Autos extends Command {
    DriveTrainSystem driveTrainSystem;
    private final Timer timer = new Timer();
    private final double duration; // Duração do comando em segundos
    public Autos(DriveTrainSystem driveTrainSystem, double duration) {
    this.driveTrainSystem = driveTrainSystem;
    this.duration = duration;
    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrainSystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize()
   {
       driveTrainSystem.tankmode(0, 0);
    timer.reset();
     timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrainSystem.tankmode(0.3, 0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    driveTrainSystem.tankmode(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= duration;
  }
    
}

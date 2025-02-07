// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manipulator;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** An example command that uses an example subsystem. */
public class GetSensor extends InstantCommand {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public boolean getAnalogSensorGamePiece(AnalogInput sensor) {
      double voltage = sensor.getVoltage();
      double distanceCm = 27.86 / (voltage - 0.42);

      if(distanceCm > 20){
        return false;
      }
      return true;
    }

    public double getAnalogSensorDistance(AnalogInput sensor) {
      double voltage = sensor.getVoltage();
      double distanceCm = 27.86 / (voltage - 0.42);
      return distanceCm;
    }

    public boolean getDigitalSensor(DigitalInput sensor){
      return sensor.get();
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
}

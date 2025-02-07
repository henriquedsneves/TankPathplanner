// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class DriveTrainConstants {
    public static int rightFrontMotorID = 14;
    public static int rightBackMotorID = 2;
    public static int leftFrontMotorID = 6;
    public static int leftBackMotorID = 11;
  }

  public static class ControlsJoystick {
    public static int leftMotors = 5;
    public static int rightMotors = 1;
  }

  public static class ElevatorConstants {
    public static int elevatorLeftMotorID = 10;
    public static int elevatorRightMotorID = 3;

    public static final int limitSwitchPort1 = 0;

    public static final double PID_TOLERANCE = 0.1;
    public static final double kP = 0.2;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;

    public static final double ELEVATOR_SPEED = 0.2;
    public static final double ELEVATOR_MANUAL_SPEED = 0.3;
  }

  public static class DeployerIntakeConstants {
    public static int deployerLeftMotorID = 4;
    public static int deployerRightMotorID = 5;

    public static int deployerSensor = 0;
    public static int intakeSensor = 1;

    public static double intakeSpeed = -0.1;
  }

  public static class PivoConstants {
    public static int pivoMotorID;
  }

  public static class ClimberConstants {
    public static int climberMotorID;
  }

  public static class AlgaeConstants {
    public static int armMotorID;
    public static int intakeMotorID;
  }

  public static class ArmConstants {
    public static int armMotorID;
    public static int secondArmMotorID;

    public static final double PID_TOLERANCE = 0.1;
    public static final double kP = 0.9;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;

    public static final double ARM_SPEED = 0.1;
    public static final double SECOND_ARM_SPEED = 0.1;
    public static final double ARM_SETPOINT = 30;
    public static final double ARM_SETPOINT_REST = 0;
  }

  //States

  public static enum DriveTrainState {
    STOPPED(0), MID(0.3), FULL(0.7);
    public final double velocity;
    
    private DriveTrainState(double velocity){
      this.velocity = velocity;
    }
  }

  public static enum ElevatorState {
    CORAL(0), L1(85), L2(150), L3(290), L4(486);
    public final double position; 
    
    private ElevatorState(double position){
      this.position = position;
    }
  }

  public static enum ModeElevator {
    MANUAL(true), AUTOMATIC(false);
    public final boolean mode;
    
    private ModeElevator(Boolean mode){
      this.mode = mode;
    }
  }

  public static enum IntakeState {
    STOPPED(false), OPERATION(true);
    public final boolean operation;
    
    private IntakeState(Boolean operation){
      this.operation = operation;
    }
  }

  public static enum DeployerState {
    STOPPED(0), SHOOTING(0.3);
    public final double speed;
    
    private DeployerState(double speed){
      this.speed = speed;
    }
  }

  public static enum PivoState {
    CLOSING(-0.1), STOPPED(0), OPENING(0.1);
    public final double speed;
    
    private PivoState(double speed){
      this.speed = speed;
    }
  }

  public static enum ClimberState {
    CLOSING(-0.1), STOPPED(0), OPENING(0.1);
    public final double speed;
    
    private ClimberState(double speed){
      this.speed = speed;
    }
  }

  public static enum AlgaeState {
    CLOSING(-0.1), STOPPED(0), OPENING(0.1);
    public final double speed;
    
    private AlgaeState(double speed){
      this.speed = speed;
    }
  }

  public static enum ArmState {
    STOPPED(false), OPERATION(true);
    public final boolean state;
    
    private ArmState(Boolean state){
      this.state = state;
    }
  }
  public static final class Trajetoria {
    public static final String nome_Trajetoria1 = "Auto";
    public static final String nome_Trajetoria2 = "AutoTest";
    
    
    }
  }


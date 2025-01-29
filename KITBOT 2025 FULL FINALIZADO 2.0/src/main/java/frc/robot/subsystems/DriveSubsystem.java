
/*package frc.robot.subsystems;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    private Pose2d currentPose = new Pose2d(); // Armazena a pose do robô
    private ChassisSpeeds currentSpeeds = new ChassisSpeeds(); // Armazena velocidades do robô
  public DriveSubsystem() {
    RobotConfig config;

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      return;
    }

    AutoBuilder.configure(
            this::getPose, // Método que retorna a pose atual do robô
            this::resetPose, // Método para resetar a odometria
            this::getRobotRelativeSpeeds, // Retorna a velocidade do robô em ChassisSpeeds
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Define a movimentação do robô
            new PPLTVController(0.02), // Controlador de trajetória para Tank Drive
            config, // Configuração do robô
            () -> {
              var alliance = DriverStation.getAlliance();
              return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            },
            this // Adiciona dependência ao subsistema
    );
  }
  public Pose2d getPose() {
    return currentPose; // Substituir pelo método real de odometria
  }

  /**
   * Reseta a pose do robô para uma nova posição.
   * @param newPose Nova pose do robô
   */
  /*public void resetPose(Pose2d newPose) {
    this.currentPose = newPose;
    // Adicione aqui o reset real da odometria se necessário
  }

  /**
   * Retorna as velocidades do robô no espaço do robô (ROBOT RELATIVE).
   */
  /*public ChassisSpeeds getRobotRelativeSpeeds() {
    return currentSpeeds; // Substituir pela leitura real da cinemática do robô
  }

  /**
   * Dirige o robô baseado nos ChassisSpeeds.
   */
  /*public void driveRobotRelative(ChassisSpeeds speeds) {
    this.currentSpeeds = speeds;
    // Adicionar aqui o controle real dos motores para movimentação
  }

}*/


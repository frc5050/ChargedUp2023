package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Brake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;

public class DriveAuto {

    public static Command autoDriveCommand(Drive drive, Intake intake, Brake brake) {
        intake.resetIntakeTimer();
        return Commands.sequence(brake.setBrakeUpCommand(), 
        intake.runShootMotorCommandUntil(-1, Constants.kShootingTimeOut), 
        drive.driveDistanceCommand(2.5, -0.4, -0.1), 
        brake.setBrakeDownCommand());
      }

      public DriveAuto(){
      }
    
    
}

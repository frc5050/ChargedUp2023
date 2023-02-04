package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;

public class DriveAuto {

    public static Command autoDriveCommand(Drive drive, Intake intake) {
        intake.resetIntakeTimer();
        return Commands.sequence(drive.controlBrakeCommand(false), intake.runShootMotorCommandUntil(-1), drive.driveDistanceCommand(2.5, -0.4, -0.1), drive.controlBrakeCommand(true));
      }

      public DriveAuto(){
      }
    
    
}

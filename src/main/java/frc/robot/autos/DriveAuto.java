package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive;

public class DriveAuto {

    public static Command autoDriveCommand(Drive subsystem) {
        return Commands.sequence(subsystem.driveDistanceCommand(5,0.3));
      }

      public DriveAuto(){

      }
    
    
}

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Brake;
import frc.robot.subsystems.Drive;

public class shimmy extends CommandBase{
         
    public static Command shimmyCommand (Drive drive, Brake brake){

        return Commands.sequence(brake.setBrakeUpCommand(),
        drive.shimmyCommand());
    }
    
}

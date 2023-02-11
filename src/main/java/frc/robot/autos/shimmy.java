package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive;

public class shimmy extends CommandBase{
         
    public static Command shimmyCommand (Drive drive){

        return Commands.sequence(drive.controlBrakeCommand(true),
        drive.shimmyCommand());
    }
    
}

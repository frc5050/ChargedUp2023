package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lifter;

public class HighConeCommand extends CommandBase {

//High cone on button :)
    public static Command HighConeConfigurationCommand (Intake intake, Lifter lifter){
        return Commands.parallel(intake.tiltToDegreesCommand(Constants.kTiltConeHighPositionDegrees), 
        lifter.elevatorPIDCommand(Constants.kElevatorHighPosition));

     
    }

    


    
}

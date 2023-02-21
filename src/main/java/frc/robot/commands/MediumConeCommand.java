package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lifter;

public class MediumConeCommand extends CommandBase {

//Medium cone on button :)
    public static Command MediumConeConfigurationCommand (Intake intake, Lifter lifter){
        return Commands.parallel(intake.tiltToPositionCommand(Constants.kConePosition), 
        lifter.elevatorPIDCommand(Constants.kElevatorMiddlePosition));

     
    }

    


    
}
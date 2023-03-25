package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Tilt;

public class HighCubeCommand extends CommandBase{

    public static Command HighCubeConfigurationCommand (Tilt tilt, Lifter lifter) {
           

        return Commands.parallel(tilt.tiltToDegreesCommand(Constants.kTiltHighCubePosition, false),
                lifter.elevatorPIDAutonCommand(Constants.kElevatorHighCubePosition));

    }

    
}

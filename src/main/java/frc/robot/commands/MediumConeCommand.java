package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Tilt;

public class MediumConeCommand extends CommandBase {

    // Medium cone on button :)
    public static Command MediumConeConfigurationCommand(Tilt tilt, Lifter lifter) {
        return Commands.parallel(
                tilt.tiltToDegreesCommand(Constants.kMidConePosition, false),
                lifter.elevatorPIDAutonCommand(Constants.kElevatorMiddlePosition));

    }

}
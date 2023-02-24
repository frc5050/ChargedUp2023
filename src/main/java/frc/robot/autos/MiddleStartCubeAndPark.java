package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.AutoStartUpCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;

public class MiddleStartCubeAndPark extends CommandBase{

    public static Command middleStartAndParkCommand(Drive drive, Intake intake){
        return Commands.sequence(
            AutoStartUpCommand.AutoStartUp(intake, drive),
            intake.runShootMotorCommandUntil(Constants.kHighShotMotorPower, Constants.kShootingTimeOut),
            drive.driveDistanceCommand(-3.5, 0.75, 0.0),
            drive.driveDistanceCommand(1.25, 0.75, 0.0),
            drive.balanceRollCommand(Constants.kNavXRollOffset)
            //drive.controlBrakeCommand(false)




        );
    }

    
}

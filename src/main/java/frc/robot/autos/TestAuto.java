package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;

public class TestAuto extends CommandBase{

    public static Command middleStartAndParkCommand(Drive drive, Intake intake){
        return Commands.sequence(
            intake.runShootMotorCommandUntil(Constants.kHighShotMotorPower, Constants.kShootingTimeOut),
            drive.driveDistanceCommand(-4, -0.5, 0.0),
            drive.driveDistanceCommand(2, 0.5, 0.0),
            drive.balanceRollCommand(Constants.kNavXRollOffset)
            //drive.controlBrakeCommand(false)
        );
    }

    
}

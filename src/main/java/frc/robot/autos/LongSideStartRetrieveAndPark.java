package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;

public class LongSideStartRetrieveAndPark extends CommandBase {
    
    public static Command sideStartRetrieveAndParkCommand(Drive drive, Intake intake) {
        drive.resetNavX();
        return Commands.sequence(intake.zeroTiltMotorCommand(),  
            drive.zeroDriveEncoderCommand(),
            drive.controlBrakeCommand(true),
            intake.runShootMotorCommandUntil(-1.0, Constants.kShootingTimeOut), 
            drive.driveDistanceCommand(-4, -0.75, -0.19), 
            drive.turnToAngleCommand(180),
            drive.driveDistanceCommand(1.5, 0.75, 0.20),
            intake.runTiltMotorCommandUntil(0.3 ),
            Commands.parallel(
            intake.runShootMotorCommandUntil(0.5, 1.5),
            drive.turnToAngleCommand(165)),
            Commands.parallel(
            intake.runShootMotorCommandUntil(0.5, 1.5),
            drive.turnToAngleCommand(195)),
            drive.turnToAngleCommand(225),
            drive.driveDistanceCommand(-3, -0.6, -0.1),
            drive.turnToAngleCommand(180),
            drive.driveDistanceCommand(-2, -0.4, -0.1),
            drive.balanceRollCommand(0));
      }

    
}

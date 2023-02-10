package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;

public class SideStartRetrieveAndPark extends CommandBase {
    
    public static Command sideStartRetrieveAndParkCommand(Drive drive, Intake intake) {
        drive.resetNavX();
        return Commands.sequence(intake.zeroTiltMotorCommand(),  
            drive.zeroDriveEncoderCommand(),
            drive.controlBrakeCommand(true),
            intake.runShootMotorCommandUntil(1.0), 
            drive.driveDistanceCommand(-1, 0.3, -0.1), 
            drive.turnToAngleCommand(180), 
            intake.runTiltMotorCommandUntil(0.3), 
            intake.runShootMotorCommandUntil(0.5),
            drive.turnToAngleCommand(135),
            drive.driveDistanceCommand(-1, 0.3, -0.1),
            drive.balanceRollCommand(0));
      }

    
}

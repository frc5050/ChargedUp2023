package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Brake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;

public class SideStartAndPark extends CommandBase{

    //unfinished

    public static Command sideStartAndParkCommand(Drive drive, Intake intake, Brake brake) {
        drive.resetNavX();
        return Commands.sequence(intake.zeroTiltMotorCommand(),  
            drive.zeroDriveEncoderCommand(),
            brake.setBrakeUpCommand(),
            intake.runShootMotorCommandUntil(-1.0, Constants.kShootingTimeOut), 
            drive.driveDistanceCommand(-5, -0.75, -0.20),
            drive.turnToAngleCommand(45),
            drive.driveDistanceCommand(3, 0.6, -0.1),
            drive.turnToAngleCommand(0),
            drive.driveDistanceCommand(2, 0.4, -0.1),
            drive.balanceRollCommand(0));
      }
    
}

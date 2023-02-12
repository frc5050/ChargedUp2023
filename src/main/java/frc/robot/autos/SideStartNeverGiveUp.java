package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;

public class SideStartNeverGiveUp {
    
    public static Command sideStartNeverGiveUpCommand(Drive drive, Intake intake){
        return Commands.sequence(
                drive.zeroDriveEncoderCommand(),
                drive.controlBrakeCommand(true),
                intake.runShootMotorCommandUntil(Constants.kHighShotMotorPower, Constants.kShootingTimeOut),
                drive.driveDistanceCommand(-5, -0.75, -0.20),
                drive.turnToAngleCommand(180),
                intake.runTiltMotorCommandUntil(0.3),

                Commands.parallel(
                intake.runShootMotorCommandUntil(Constants.kIntakePower, 1.0),
                drive.driveDistanceCommand(1.5, 0.4, 0.1))
                );

    }


}

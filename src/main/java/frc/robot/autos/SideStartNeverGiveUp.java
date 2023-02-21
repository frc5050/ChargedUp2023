package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Brake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;

public class SideStartNeverGiveUp {
    
    public static Command sideStartNeverGiveUpCommand(Drive drive, Intake intake, Brake brake){
        return Commands.sequence(
                drive.zeroDriveEncoderCommand(),
                brake.setBrakeUpCommand(),
                // intake.runShootMotorCommandUntil(Constants.kHighShotMotorPower, Constants.kShootingTimeOut),
                // drive.driveDistanceCommand(-5, 1.0, 0.0),
                drive.turnToAngleCommand(180),
                // intake.runTiltMotorCommandUntil(Constants.kIntakeTiltOutPower),

                Commands.parallel(
                intake.runShootMotorCommandUntil(Constants.kIntakePower, 1.0),
                drive.driveDistanceCommand(1, 0.4, 0.0)
                )
                );

    }


}

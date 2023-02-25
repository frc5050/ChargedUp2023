package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.AutoStartUpCommand;
import frc.robot.subsystems.Brake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

public class LongSideStartRetrieveAndPark extends CommandBase {

    public static Command sideStartRetrieveAndParkCommand(Drive drive, Intake intake, Brake brake, Tilt tilt) {
        drive.resetNavX();
        return Commands.sequence(
                AutoStartUpCommand.AutoStartUp(tilt, drive),
                tilt.zeroTiltMotorCommand(),
                drive.zeroDriveEncoderCommand(),
                brake.setBrakeUpCommand(),
                intake.runShootMotorCommandUntil(-1.0, Constants.kShootingTimeOut),
                drive.driveDistanceCommand(-4, -0.75, -0.19),
                drive.turnToAbsoluteAngleCommand(180),
                drive.driveDistanceCommand(1.5, 0.75, 0.20),
                tilt.runTiltMotorCommandUntil(0.3, Constants.kIntakeTimeout),
                Commands.parallel(
                        intake.runShootMotorCommandUntil(0.5, 1.5),
                        drive.turnToAbsoluteAngleCommand(165)),
                Commands.parallel(
                        intake.runShootMotorCommandUntil(0.5, 1.5),
                        drive.turnToAbsoluteAngleCommand(195)),
                drive.turnToAbsoluteAngleCommand(225),
                drive.driveDistanceCommand(-3, -0.6, -0.1),
                drive.turnToAbsoluteAngleCommand(180),
                drive.driveDistanceCommand(-2, -0.4, -0.1),
                drive.balanceRollCommand(Constants.kNavXRollOffset));
    }

}

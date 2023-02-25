package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.AutoStartUpCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

public class MiddleStartCubeAndPark {

    public static Command middleStartAndParkCommand(Drive drive, Intake intake, Tilt tilt) {
        return Commands.sequence(
                AutoStartUpCommand.AutoStartUp(tilt, drive),
                intake.runShootMotorCommandUntil(Constants.kHighShotMotorPower, Constants.kShootingTimeOut),
                drive.driveDistanceCommand(Constants.kDistanceOverStation, 1.0, 0.0, Constants.kBalancingAutonTimAccel,
                        0.4),
                drive.driveDistanceCommand(Constants.kDistanceBackToStation, 1.0, 0.0, Constants.kDriveTimAccel, 0.2),
                drive.balanceRollCommand(Constants.kNavXRollOffset)
        // drive.controlBrakeCommand(false)

        );
    }

}

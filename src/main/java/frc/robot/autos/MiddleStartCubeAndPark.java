package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.AutoStartUpCommand;
import frc.robot.subsystems.Brake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

public class MiddleStartCubeAndPark {

    public static Command middleStartAndParkCommand(Drive drive, Intake intake, Tilt tilt, Brake brake) {
        return Commands.sequence(
                AutoStartUpCommand.AutoStartUp(tilt, drive, brake),
                intake.runShootMotorCommandUntil(Constants.kHighShotMotorPower, Constants.kShootingTimeOut),
                drive.driveDistanceCommand(Constants.kDistanceToStation, 1.0, 0.0, Constants.kBalancingAutonTimAccel,
                        0.4, true),
                drive.balanceRollCommand(Constants.kNavXRollOffset),
                brake.setBrakeDownCommand()
                

        );
    }
    
}

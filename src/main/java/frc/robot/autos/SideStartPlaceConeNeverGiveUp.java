package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.AutoStartUpCommand;
import frc.robot.subsystems.Brake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Tilt;

public class SideStartPlaceConeNeverGiveUp {

    public static Command sideStartNeverGiveUpCommand(Drive drive, Intake intake, Brake brake, Tilt tilt,
            Lifter lifter) {
        return Commands.sequence(
                AutoStartUpCommand.AutoStartUp(tilt, drive),
                Commands.parallel(
                        Commands.sequence(Commands.run(() -> {
                        }).until(() -> lifter.minimumHeightAcquired(Constants.kAutonHeightWait)),
                                tilt.tiltToDegreesCommand(Constants.kTiltConeHighPositionDegrees, true)),
                        lifter.elevatorPIDAutonCommand(Constants.kElevatorHighPosition)),
                lifter.coneOuttakeCommand(true),

                Commands.parallel(
                        tilt.runTiltMotorCommand(Constants.kTiltMotorInMotorPower)
                                .withTimeout(Constants.kAutoReturnTiltTimeout),
                        lifter.coneOuttakeCommand(true)),
                lifter.stopConeShooting(),
                lifter.elevatorPIDAutonCommand(Constants.kElevatorDownPosition),
                drive.driveDistanceCommand(-4, 1.0, 0.0, Constants.kDriveTimAccel, 0.2),
                drive.turnToAbsoluteAngleCommand(180),
                tilt.runTiltMotorCommandUntil(Constants.kIntakeTiltOutPower, 2.0),

                Commands.parallel(
                        intake.runShootMotorCommandUntil(Constants.kIntakePower, 1.0),
                        drive.driveDistanceCommand(0.7, 0.4, 0.0)));

    }

}

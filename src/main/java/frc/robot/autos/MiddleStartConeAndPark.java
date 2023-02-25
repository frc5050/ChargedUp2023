package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.AutoStartUpCommand;
import frc.robot.commands.HighConeCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Tilt;

public class MiddleStartConeAndPark {

    public static Command middleStartAndParkCommand(Drive drive, Intake intake, Lifter lifter, Tilt tilt) {
        return Commands.sequence(
                AutoStartUpCommand.AutoStartUp(tilt, drive),
                // HighConeCommand.HighConeConfigurationCommand(intake, lifter),

                // it is important that the tiltToDegreesCommand never ends.
                // Commands.race(
                // Commands.sequence(Commands.run(() ->{}).until(() ->
                // lifter.minimumHeightAcquired(Constants.kAutonHeightWait)),
                // tilt.tiltToDegreesCommand(Constants.kTiltConeHighPositionDegrees)),
                // Commands.sequence(lifter.elevatorPIDAutonCommand(Constants.kElevatorHighPosition),
                // lifter.coneOuttakeCommand(true))),

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
                drive.driveDistanceCommand(Constants.kDistanceOverStation, 1.0, 0.0, Constants.kBalancingAutonTimAccel,
                        0.4),
                drive.driveDistanceCommand(Constants.kDistanceBackToStation, 1.0, 0.0, Constants.kDriveTimAccel, 0.2),
                drive.balanceRollCommand(Constants.kNavXRollOffset)
        // drive.controlBrakeCommand(false)

        );
    }

}
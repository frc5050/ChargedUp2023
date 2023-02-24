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

public class MiddleStartConeAndPark extends CommandBase {

    public static Command middleStartAndParkCommand(Drive drive, Intake intake, Lifter lifter) {
        return Commands.sequence(
                AutoStartUpCommand.AutoStartUp(intake, drive),
                // HighConeCommand.HighConeConfigurationCommand(intake, lifter),
                Commands.parallel(
                    intake.tiltToDegreesCommand(Constants.kTiltConeHighPositionDegrees),
                    Commands.sequence(lifter.elevatorPIDAutonCommand(Constants.kElevatorHighPosition),
                                      lifter.coneIntakeCommand(Constants.kConeOuttakeMotorPower))),

                intake.runTiltMotorCommand(Constants.kTiltMotorInMotorPower),
                lifter.elevatorPIDCommand(Constants.kElevatorDownPosition),
                drive.driveDistanceCommand(-3.5, 0.75, 0.0),
                drive.driveDistanceCommand(1.25, 0.75, 0.0),
                drive.balanceRollCommand(Constants.kNavXRollOffset)
        // drive.controlBrakeCommand(false)

        );
    }

}

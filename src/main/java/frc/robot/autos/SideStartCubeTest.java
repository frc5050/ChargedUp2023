package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.AutoStartUpCommand;
import frc.robot.subsystems.Brake;
import frc.robot.subsystems.ConeIntake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Tilt;

public class SideStartCubeTest {
    public static Command sideStartPlaceCubeCommand(Drive drive, Intake intake, Brake brake, Tilt tilt,
            ConeIntake coneIntake, Lifter lifter) {
        return Commands.sequence(
                AutoStartUpCommand.AutoStartUp(tilt, drive, brake),
                drive.zeroDriveEncoderCommand(),
                Commands.parallel(
                        Commands.sequence(Commands.run(() -> {
                        }).until(() -> lifter
                                .minimumHeightAcquired(Constants.kAutonHeightWait)),
                                tilt.tiltToDegreesCommand(
                                        Constants.kTiltHighCubePosition,
                                        true)),
                        lifter.elevatorPIDAutonCommand(Constants.kElevatorHighConePosition)),
                intake.runShootMotorCommandUntil(Constants.kVeryLowShootPower, 0.5),
                Commands.parallel(
                        tilt.runTiltMotorCommand(Constants.kTiltMotorInMotorPower)
                                .withTimeout(Constants.kAutoReturnTiltTimeout),
                        intake.runShootMotorCommandUntil(Constants.kVeryLowShootPower, 0.1)),
                // coneIntake.stopConeShooting(),
                lifter.elevatorPIDAutonCommand(Constants.kElevatorDownPosition),
                drive.driveDistanceCommand(-3.8, 2.0, 0.0, true),
                drive.turnToAbsoluteAngleCommand(180),
                tilt.runTiltMotorCommandUntil(Constants.kAutoIntakeTiltOutPower, 2.0),

                Commands.parallel(
                        coneIntake.coneIntakeCommand(),
                        drive.driveDistanceCommand(1.5, 0.4, 0.0, true)));

    }

}

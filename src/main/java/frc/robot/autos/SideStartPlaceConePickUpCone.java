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

public class SideStartPlaceConePickUpCone {
    
    public static Command sideStartNeverGiveUpCommand(Drive drive, Intake intake, Brake brake, Tilt tilt,
    Lifter lifter, ConeIntake coneIntake) {
return Commands.sequence(
            AutoStartUpCommand.AutoStartUp(tilt, drive, brake),
            Commands.parallel(
                            Commands.sequence(Commands.run(() -> {
                            }).until(() -> lifter
                                            .minimumHeightAcquired(Constants.kAutonHeightWait)),
                                            tilt.tiltToDegreesCommand(
                                                            Constants.kTiltConeHighPositionDegrees,
                                                            true)),
                            lifter.elevatorPIDAutonCommand(Constants.kElevatorHighConePosition)),
            coneIntake.coneOuttakeCommand(true),

            Commands.parallel(
                            tilt.runTiltMotorCommand(Constants.kTiltMotorInMotorPower)
                                            .withTimeout(Constants.kAutoReturnTiltTimeout),
                            coneIntake.coneOuttakeCommand(true)),
            coneIntake.stopConeShooting(),
            lifter.elevatorPIDAutonCommand(Constants.kElevatorDownPosition),
            drive.driveDistanceCommand(-4, 2.0, 0.0, (Constants.kDriveTimAccel * 2) + 0.2, 0.2, true),
            drive.turnToAbsoluteAngleCommand(180),
            tilt.runTiltMotorCommandUntil(Constants.kAutoIntakeTiltOutPower, 2.0),

            Commands.parallel(
                            coneIntake.coneIntakeCommand(),
                            drive.driveDistanceCommand(0.6, 0.4, 0.0, true)));

}
}

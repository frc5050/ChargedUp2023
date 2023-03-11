package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.AutoStartUpCommand;
import frc.robot.subsystems.Brake;
import frc.robot.subsystems.ConeIntake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;

public class SideStartShootCubePickUpCone {

    public static Command sideStartShootCubePickUpConeCommand(Drive drive, Intake intake, Brake brake, Tilt tilt, ConeIntake coneIntake) {
        return Commands.sequence(
                        AutoStartUpCommand.AutoStartUp(tilt, drive, brake),
                        drive.zeroDriveEncoderCommand(),
                        intake.runShootMotorCommandUntil(Constants.kHighShotMotorPower,
                                        Constants.kShootingTimeOut),
                        drive.driveDistanceCommand(-3.8, 1.0, 0.0),
                        drive.turnToAbsoluteAngleCommand(180),
                        tilt.runTiltMotorCommandUntil(Constants.kAutoIntakeTiltOutPower, 2.0),

                        Commands.parallel(
                                        coneIntake.coneIntakeCommand(),
                                        drive.driveDistanceCommand(1.5, 0.4, 0.0, true)));

}
    
}

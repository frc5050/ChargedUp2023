package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.ConeIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tilt;
import frc.robot.subsystems.LED;

public class FeederConeIntakeCommand extends CommandBase{

    //Cone Intake with slow Cube Outtake
    public static Command intakeConeCommand (Tilt tilt, Intake intake, ConeIntake coneIntake, LED led) {
        return Commands.parallel(tilt.tiltToDegreesCommand(Constants.kTiltFeederPositionDegrees, false),
        intake.runShootMotorCommand(-0.1, led),
        coneIntake.coneCommand(0.8)
        //led.setLEDColorCommand(25, 0, 29)
        );
    }
    
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Brake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Tilt;

public class AutoStartUpCommand {

    public static Command AutoStartUp(Tilt tilt, Drive drive, Brake brake) {
        return Commands.parallel(
                drive.zeroYawCommand(),
                tilt.zeroTiltMotorCommand(),
                brake.setBrakeUpCommand()

        );


    }
}

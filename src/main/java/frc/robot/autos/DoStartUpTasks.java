package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoStartUpCommand;
import frc.robot.subsystems.Brake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Tilt;

public class DoStartUpTasks {

        public static Command doStartUpTasksCommand(Tilt tilt, Drive drive, Brake brake) {
                return AutoStartUpCommand.AutoStartUp(tilt, drive, brake);

        }

}

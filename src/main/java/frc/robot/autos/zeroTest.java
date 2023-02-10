package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;

public class zeroTest {
     
    public static Command zeroTestCommand(Intake m_intake){

        return Commands.sequence(m_intake.zeroTiltMotorEncoderCommand());
    }

    

}

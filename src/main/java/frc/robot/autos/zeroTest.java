package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Tilt;

public class zeroTest {
     
    public static Command zeroTestCommand( Tilt tilt){

        return Commands.sequence(tilt.zeroTiltMotorEncoderCommand());
    }

    

}

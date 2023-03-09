package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.LED;

public class LEDTestCommand {
    public static Command intakeConeCommand (LED led) {
        return Commands.sequence(led.setLEDColorCommand(25, 0, 29).withTimeout(5), 
        led.turnLEDOffCommand().withTimeout(5),
        led.setLEDColorCommand(106, 76, 0));
        
    }
 

    
}

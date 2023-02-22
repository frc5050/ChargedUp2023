package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;

public class PickUpCube {


        public static Command autoTurnCommand(Drive drive, Intake intake) {
            drive.resetNavX();
            return
            Commands.sequence(drive.zeroYawCommand(), intake.runShootMotorCommandUntil(-1.0, 1),drive.turnToAbsoluteAngleCommand(180), drive.driveDistanceCommand(3.5, 0.5, 0.115), intake.runTiltMotorCommandUntil(0.3), intake.runShootMotorCommandUntil(0.5, Constants.kShootingTimeOut));
          }
    
          public PickUpCube(){
          }
        
    
    
    
}

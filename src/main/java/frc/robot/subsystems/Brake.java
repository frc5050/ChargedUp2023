package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Brake extends SubsystemBase{
    private Solenoid m_brakeSolenoid;
    public Compressor m_compressor;
    PneumaticHub m_PneumaticHub;
    public Brake() {
        m_PneumaticHub = new PneumaticHub(Constants.kPHCANID);
        m_compressor = new Compressor(PneumaticsModuleType.REVPH);
        m_brakeSolenoid = new Solenoid(Constants.kPHCANID,PneumaticsModuleType.REVPH, 7);
        m_PneumaticHub.enableCompressorDigital();
        m_compressor.enableDigital();    
    }
    public CommandBase setBrakeUpCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
              System.out.println("Setting brake to: " + true);
              m_brakeSolenoid.set(true);
            }
            );
      }
      public CommandBase setBrakeDownCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
              System.out.println("Setting brake to: " + false);
              m_brakeSolenoid.set(false);
            }
            );
      }
      public CommandBase setBrakeCommand(BooleanSupplier joystickButton) {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return run(
            () -> {
              if (!joystickButton.getAsBoolean()) {
                m_brakeSolenoid.set(true);
              } else if (joystickButton.getAsBoolean()){
                m_brakeSolenoid.set(false);
              }
      });
      }
      public void setSolenoid(boolean solenoidState){
        m_brakeSolenoid.set(solenoidState);
      }
}

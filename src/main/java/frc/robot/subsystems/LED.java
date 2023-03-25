package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase{
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;

    public LED(){
        m_led = new AddressableLED(Constants.LEDPort);
        m_ledBuffer = new AddressableLEDBuffer(Constants.LEDCount);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();

    }

    public void setLEDColor(int R, int G, int B) {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, R, G, B);
          }
      
          m_led.setData(m_ledBuffer);
    }   

    public CommandBase turnLEDOffCommand(){
        return run (
            () -> {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
          }
      
          m_led.setData(m_ledBuffer);
        });
    }

    //TODO needs help
    public CommandBase controlLEDCommand(Intake intake, Tilt tilt, ConeIntake coneIntake){

        return run (
            () -> {
        if(intake.shooterIRisTriggered()){ //cube intake is full
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, 0, 110, 0);
            }
                m_led.setData(m_ledBuffer);
        } else if (coneIntake.getCurrentConeIntakePower() > 0.0){ //cone intake 
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, 106, 76, 0);
            }
                m_led.setData(m_ledBuffer);
        } else if(intake.getCurrentIntakeMotorPower() > 0){  //cube intake
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, 25, 0, 29);
            }
                m_led.setData(m_ledBuffer);
        } else {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, 50, 50, 50);

            }
                m_led.setData(m_ledBuffer);
        }
        }
        ).ignoringDisable(true);
    }

    public CommandBase setLEDColorCommand(int R, int G, int B) {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return run(
            () -> { 
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, R, G, B);
            }
            m_led.setData(m_ledBuffer); 
            });
      }


}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase{
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;
    AddressableLED m_led2;
    AddressableLEDBuffer m_ledBuffer2;

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
}

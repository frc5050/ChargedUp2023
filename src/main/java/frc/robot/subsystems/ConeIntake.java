package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConeIntake extends SubsystemBase{
    private CANSparkMax m_ConeIntakeMotor = new CANSparkMax(Constants.kConeIntakeCANID, MotorType.kBrushless);

    public ConeIntake(){
        m_ConeIntakeMotor.restoreFactoryDefaults();
        m_ConeIntakeMotor.setIdleMode(IdleMode.kBrake);
        m_ConeIntakeMotor.setSmartCurrentLimit(10);

    }

    public CommandBase coneIntakeDoNothingCommand() {
        return run(
            () -> {
              m_ConeIntakeMotor.set(0.0);
            });
      }

    public CommandBase coneIntakeCommand() {
        return run(
            () -> {
              double power = Constants.kConeIntakeMotorPower;
              m_ConeIntakeMotor.set(power);
            });
      }
      public CommandBase coneCommand(double power) {
        return run(
            () -> {
              m_ConeIntakeMotor.set(power);
            });
      }

    public CommandBase coneOuttakeCommand(boolean usetimeout) {
        CommandBase out = run(
            () -> {
              double power = Constants.kConeOuttakeMotorPower;
              m_ConeIntakeMotor.set(power);
            });
        if (usetimeout) {
          out = out.withTimeout(0.5);
        }
        return out;
    
      }

      public CommandBase stopConeShooting() {
        return runOnce(
            () -> {
              m_ConeIntakeMotor.stopMotor();
            });
      }

    
}

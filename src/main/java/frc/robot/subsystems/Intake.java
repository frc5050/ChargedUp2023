// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private CANSparkMax m_shootMotor = new CANSparkMax(Constants.kCubeIntakeCANID, MotorType.kBrushless);
  public Solenoid m_Solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.kSolenoidChannel);

  private Timer m_intakeTimer;
  private Timer m_autonTimer;
  // private RelativeEncoder m_shootEncoder;
  // private SparkMaxPIDController m_IntegratedtiltPID;
  private SparkMaxPIDController m_shootPID;
  private DigitalInput m_shooterIR;

  private boolean m_shooterIRWasPreviouslyTriggered;

  public Intake() {

    // m_shootEncoder = m_shootMotor.getEncoder();
    // m_IntegratedtiltPID = m_tiltMotor.getPIDController();

    m_shootPID = m_shootMotor.getPIDController();

    // m_IntegratedtiltPID.setFeedbackDevice(m_tiltEncoder);

    // m_tiltPID.setFeedbackDevice(m_tiltShaftEncoder);
    // BE() CAREFUL WITH ALTERNATE ENCODER m_tiltMotor.restoreFactoryDefaults(); BE
    // CAREFUL WITH ALTERNATE ENCODER()
    m_shootMotor.restoreFactoryDefaults();
    m_shootMotor.setIdleMode(IdleMode.kCoast);

    m_shooterIR = new DigitalInput(2);
    m_shootPID.setP(0.00025);
    m_shootPID.setI(0.0000015);
    m_shootPID.setIZone(200);
    m_shootPID.setD(0);

    // m_IntegratedtiltPID.setP(0.01);
    // m_IntegratedtiltPID.setI(0.0);
    // m_IntegratedtiltPID.setIZone(0);
    // m_IntegratedtiltPID.setD(0);

    m_intakeTimer = new Timer();
    m_autonTimer = new Timer();
    m_shooterIRWasPreviouslyTriggered = false;
    m_shootMotor.enableVoltageCompensation(10.0);

    // m_shootPID.setFF(1 / 5400);

  }

  public CommandBase shootHighCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    double power = Constants.kHighShotMotorPower;
    double timeout = Constants.kShootingTimeOut;
    return runOnce(
        () -> {
          m_autonTimer.reset();
          m_autonTimer.start();

        }).beforeStarting(run(
            () -> {
              if (shooterIRisTriggered() && power > 0 && m_intakeTimer.hasElapsed(0.05)) {
                m_shootMotor.set(0);
              } else {
                m_shootMotor.set(power);
              }
            }))
        .withTimeout(timeout)
        .finallyDo((interrupted) -> m_shootMotor.set(0));
  }

  public CommandBase shootMidCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    double power = Constants.kMidShotMotorPower;
    double timeout = Constants.kShootingTimeOut;
    return runOnce(
        () -> {
          m_autonTimer.reset();
          m_autonTimer.start();

        }).beforeStarting(run(
            () -> {
              if (shooterIRisTriggered() && power > 0 && m_intakeTimer.hasElapsed(0.05)) {
                m_shootMotor.set(0);
              } else {
                m_shootMotor.set(power);
              }
            }))
        .withTimeout(timeout)
        .finallyDo((interrupted) -> m_shootMotor.set(0));
  }

  public void resetIntakeTimer() {
    m_intakeTimer.reset();
  }

  public CommandBase shootPopCommand(boolean solenoidState) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_Solenoid.set(solenoidState);
        });
  }

  public CommandBase runShootMotorCommandwithPID(double targetValue, ControlType controlType) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> m_shootPID.setReference(targetValue, controlType));
  }

  public CommandBase runShootMotorCommandUntil(double power, double timeout) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.

    return runOnce(
        () -> {
          m_autonTimer.reset();
          m_autonTimer.start();

        }).beforeStarting(run(
            () -> {
              if (shooterIRisTriggered() && power > 0 && m_intakeTimer.hasElapsed(0.05)) {
                m_shootMotor.set(0);
              } else {
                m_shootMotor.set(power);
              }
            }))
        .withTimeout(timeout)
        .finallyDo((interrupted) -> m_shootMotor.set(0));
  }

  public boolean shooterIRisTriggered() {
    return !m_shooterIR.get();
  }

  public CommandBase runShootMotorCommand(double power) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          if (shooterIRisTriggered() && power > 0 && m_intakeTimer.hasElapsed(Constants.kIntakeIRDelay)) {
            m_shootMotor.set(0);

          } else {
            m_shootMotor.set(power);
          }

        });
  }

  public CommandBase intakeDoNothingCommand() {
    return run(
        () -> {
          m_shootMotor.set(0.0);
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    if (shooterIRisTriggered() && !m_shooterIRWasPreviouslyTriggered) {
      m_intakeTimer.reset();
      m_intakeTimer.start();
    }
    if (!shooterIRisTriggered() && m_shooterIRWasPreviouslyTriggered) {
      m_intakeTimer.stop();
      m_intakeTimer.reset();
    }
    m_shooterIRWasPreviouslyTriggered = shooterIRisTriggered();

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax m_tiltMotor = new CANSparkMax(11, MotorType.kBrushless);
  private CANSparkMax m_shootMotor = new CANSparkMax(13, MotorType.kBrushless);
  public Solenoid m_Solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 5);
  private RelativeEncoder m_tiltEncoder;
  private Timer m_intakeTimer;
  private Timer m_autonTimer;
  private RelativeEncoder m_shootEncoder; 
  private SparkMaxPIDController m_tiltPID;
  private SparkMaxPIDController m_shootPID;
  private DigitalInput m_shooterIR;
  
private boolean m_shooterIRWasPreviouslyTriggered;


  public Intake() {
    m_tiltEncoder = m_tiltMotor.getEncoder();
    m_shootEncoder = m_shootMotor.getEncoder();
    m_tiltPID = m_tiltMotor.getPIDController();
    m_shootPID = m_shootMotor.getPIDController();
    m_tiltMotor.restoreFactoryDefaults();
    m_shootMotor.restoreFactoryDefaults();
    m_shootMotor.setIdleMode(IdleMode.kCoast);
    m_shooterIR = new DigitalInput(1);
    m_shootPID.setP(0.00025);
    m_shootPID.setI(0.0000015);
    m_shootPID.setIZone(200);
    m_shootPID.setD(0);
    m_intakeTimer = new Timer();
    m_autonTimer = new Timer();
    m_shooterIRWasPreviouslyTriggered = false;
    setSoftLimits();
    //m_shootPID.setFF(1 / 5400);


  }
  public void setSoftLimits(){
    m_tiltMotor.setSoftLimit(SoftLimitDirection.kForward, 37);
    m_tiltMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_tiltMotor.setSoftLimit(SoftLimitDirection.kReverse, 1);
    m_tiltMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);


  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public void resetIntakeTimer(){
    m_intakeTimer.reset();
  }

  public CommandBase shootPopCommand(boolean solenoidState) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
         m_Solenoid.set(solenoidState);
         System.out.println(solenoidState);
  });
  }

  public boolean isRunningSlowly(){
    System.out.println(m_tiltEncoder.getVelocity());
    return Math.abs(m_tiltEncoder.getVelocity()) < 1;
  }

  public void setTiltMotorPower(double power) {
    m_tiltMotor.set(power);
  }

  public void zeroTiltMotor(){
    m_tiltEncoder.setPosition(0.0);
  }

  public CommandBase runTiltMotorCommand(double power) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          setTiltMotorPower(power);
              System.out.println(m_tiltEncoder.getPosition());
  });
  }

  public CommandBase runTiltMotorCommandUntil(double power) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.

    return runTiltMotorCommand(power).withTimeout(1.0);
  }



  public CommandBase runShootMotorCommandwithPID(double targetValue, ControlType controlType) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> 
            m_shootPID.setReference(targetValue, controlType)
        );
  }

  public CommandBase runShootMotorCommandUntil(double power) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.

    return runOnce(
            () -> { m_autonTimer.reset();
                    m_autonTimer.start();

            }
    ).beforeStarting
    (   run(
        () -> {
          if (shooterIRisTriggered() && power > 0 && m_intakeTimer.hasElapsed(0.05)){
            m_shootMotor.set(0);
          }else{
          m_shootMotor.set(power);
          }}))
          .withTimeout(1)
          .finallyDo((interrupted) -> m_shootMotor.set(0));
  }

  public boolean shooterIRisTriggered(){
    return !m_shooterIR.get();
  }

  public CommandBase runShootMotorCommand(double power) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          if (shooterIRisTriggered() && power > 0 && m_intakeTimer.hasElapsed(.05)){
            m_shootMotor.set(0);
          }else{
          m_shootMotor.set(power);
          }

        }); 
  }


  


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    if(shooterIRisTriggered() && ! m_shooterIRWasPreviouslyTriggered){
      m_intakeTimer.reset();
      m_intakeTimer.start();
    }
    if (!shooterIRisTriggered() && m_shooterIRWasPreviouslyTriggered){
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

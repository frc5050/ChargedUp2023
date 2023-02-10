// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ZeroIntake extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_intake;
  private Timer m_timer;
  private boolean wasRunningSlowly;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ZeroIntake(Intake intake) {
    m_intake = intake;
    wasRunningSlowly = false;
    m_timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake.isRunningSlowly() && !wasRunningSlowly){
      m_timer.reset();
      m_timer.start();
    } else if (!m_intake.isRunningSlowly()){
      m_timer.stop();
    }
    m_intake.setTiltMotorPower(-0.2);
    wasRunningSlowly = m_intake.isRunningSlowly();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_intake.setTiltMotorPower(0.0);
      System.out.println(interrupted);
      if (!interrupted){
        m_intake.zeroTiltMotor();
        m_intake.setSoftLimits();
        System.out.println("tilt motor position: " + m_intake.getTiltMotorPosition());
        System.out.println("tilt motor soft limit: " + m_intake.getTiltSoftLimit());

      }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return m_timer.hasElapsed(0.4);
  }
}
 

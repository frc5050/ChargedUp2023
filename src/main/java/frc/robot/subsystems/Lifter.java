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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lifter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax m_elevatorMotor = new CANSparkMax(Constants.kElevatorCANID, MotorType.kBrushless);
  private RelativeEncoder m_elevatorEncoder;
  private SparkMaxPIDController m_elevatorPID;
  private final Timer m_ZeroingTimer;

  public Lifter() {
    m_ZeroingTimer = new Timer();
    m_elevatorMotor.restoreFactoryDefaults();
    m_elevatorEncoder = m_elevatorMotor.getEncoder();
    m_elevatorMotor.setIdleMode(IdleMode.kBrake);
    m_elevatorPID = m_elevatorMotor.getPIDController();
    m_elevatorPID.setP(0.7);
    m_elevatorPID.setI(0);
    m_elevatorPID.setD(0);

    setSoftLimits();
  }

  public void setSoftLimits() {
    m_elevatorMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.kElevatorTopSoftLimit);
    m_elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_elevatorMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.kElevatorBottomSoftLimit);
    m_elevatorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

  }

  public void startZeroing() {
    m_ZeroingTimer.reset();
    m_ZeroingTimer.start();
  }

  public void zeroing() {
    boolean extendZeroed = (m_elevatorEncoder.getVelocity() < 0.0001
        && m_ZeroingTimer.hasElapsed(0.3));
    if (extendZeroed) {
      m_elevatorEncoder.setPosition(0.0);
    }
  }

  public boolean minimumHeightAcquired(double minimumHeight) {
    return Math.abs(m_elevatorEncoder.getPosition()) >= Math.abs(minimumHeight);
  }

  public CommandBase elevatorDoNothingCommand() {
    return run(
        () -> {
          m_elevatorMotor.set(0.0);
        });
  }


  public CommandBase runElevatorCommand(double power) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          m_elevatorMotor.set(power);
        });
  }

  public CommandBase elevatorPIDTeleopCommand(double position) {
    return run(
        () -> {
          m_elevatorPID.setReference(position, ControlType.kPosition);
        });
  }

  public CommandBase elevatorPIDAutonCommand(double position) {
    return run(
        () -> {
          m_elevatorPID.setReference(position, ControlType.kPosition);
        }).until(
            () -> {
              //System.out.println("elevator until: " + Math.abs(m_elevatorEncoder.getPosition() - position));
              return Math.abs(m_elevatorEncoder.getPosition() - position) <= Constants.kElevatorTolerance;
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

  // commented out on sydneys request
  @Override
  public void periodic() {
    SmartDashboard.putNumber("elevator position", m_elevatorEncoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

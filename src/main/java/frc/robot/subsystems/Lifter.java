// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lifter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax m_elevatorMotor = new CANSparkMax(Constants.kElevatorCANID, MotorType.kBrushless);
  private CANSparkMax m_ConeIntakeMotor = new CANSparkMax(Constants.kConeIntakeCANID, MotorType.kBrushless);
  private RelativeEncoder m_extendEncoder;
  private final Timer m_ZeroingTimer;

  public Lifter() {
    m_ZeroingTimer = new Timer();
      m_extendEncoder = m_elevatorMotor.getEncoder();
      setSoftLimits();
      m_ConeIntakeMotor.setIdleMode(IdleMode.kBrake);
      m_elevatorMotor.restoreFactoryDefaults();
      m_ConeIntakeMotor.restoreFactoryDefaults();
  }


  public void setSoftLimits(){
      //m_elevatorMotor.setSoftLimit(SoftLimitDirection.kReverse, -293);
     // m_elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
     // m_elevatorMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
     // m_elevatorMotor.enableSoftLimit(SoftLimitDirection.kForward, true); 

  }

  public void startZeroing(){
    m_ZeroingTimer.reset();
    m_ZeroingTimer.start();
  }

  public void zeroing(){
    boolean extendZeroed = 
      (m_extendEncoder.getVelocity() < 0.0001
      && m_ZeroingTimer.hasElapsed(0.3));
      if(extendZeroed){
        m_extendEncoder.setPosition(0.0);
      }
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

  public CommandBase coneIntakeCommand(double power) {
    return run(
        () -> {
          m_ConeIntakeMotor.set( power);        });
      }

  public CommandBase runConeExtendMotorCommand(double power) {
  // Inline construction of command goes here.
  // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          m_elevatorMotor.set(power);
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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

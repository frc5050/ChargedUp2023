// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  CANSparkMax m_leftFront = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax m_leftRear =  new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax m_rightFront = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax m_rightRear = new CANSparkMax(7, MotorType.kBrushless);
  RelativeEncoder m_leftEncoder;
  RelativeEncoder m_rightEncoder;

  private final MotorControllerGroup m_leftMotors = 
    new MotorControllerGroup( 
        m_leftFront,
        m_leftRear);

  private final MotorControllerGroup m_rightMotors = 
    new MotorControllerGroup(
        m_rightFront,
        m_rightRear);
  private Solenoid m_brakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
  
  private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0, 0);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  public Drive() {

    m_compressor.enableDigital();

    m_leftFront.restoreFactoryDefaults();
    m_leftRear.restoreFactoryDefaults();
    m_rightFront.restoreFactoryDefaults();
    m_rightRear.restoreFactoryDefaults();

    m_leftEncoder = m_leftFront.getEncoder();
    m_rightEncoder = m_rightFront.getEncoder();

    m_rightMotors.setInverted(true);
    m_leftMotors.setInverted(false);

    m_leftFront.enableVoltageCompensation(12.0);
    m_leftRear.enableVoltageCompensation(12.0);
    m_rightFront.enableVoltageCompensation(12.0);
    m_rightRear.enableVoltageCompensation(12.0);
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

  public CommandBase controlBrakeCommand(boolean solenoidState) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> 
          m_brakeSolenoid.set(solenoidState)
        );
  }
  public CommandBase arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> m_drive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble())         
        );
  }

 

  
  public void setSolenoid(boolean solenoidState){
    m_brakeSolenoid.set(solenoidState);
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


  public CommandBase driveDistanceCommand(double distanceMeters, double speed) {
    return runOnce(
            () -> {
              // Reset encoders at the start of the command
              m_leftEncoder.setPosition(0.0);
              m_rightEncoder.setPosition(0.0);
              
            })
        // Drive forward at specified speed
        .andThen(run(() -> m_drive.arcadeDrive(speed, 0)))
        // End command when we've traveled the specified distance
        .until(
            () ->
                Math.max(m_leftEncoder.getPosition(), m_rightEncoder.getPosition())
                    >= distanceMeters)
        // Stop the drive when the command ends
        .finallyDo(interrupted -> m_drive.stopMotor());
  }
}


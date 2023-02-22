// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ZeroIntake;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANSparkMax m_tiltMotor = new CANSparkMax(Constants.kTiltCANID, MotorType.kBrushless);
  private CANSparkMax m_shootMotor = new CANSparkMax(Constants.kCubeIntakeCANID, MotorType.kBrushless);
  public Solenoid m_Solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.kSolenoidChannel);
  private RelativeEncoder m_tiltEncoder;
  private Timer m_intakeTimer;
  private Timer m_autonTimer;
  private RelativeEncoder m_shootEncoder;
  //private SparkMaxPIDController m_IntegratedtiltPID;
  private PIDController m_tiltPID;
  private SparkMaxPIDController m_shootPID;
  private RelativeEncoder m_tiltShaftEncoder;
  private DigitalInput m_shooterIR;

  private boolean m_shooterIRWasPreviouslyTriggered;

  public Intake() {
    m_tiltEncoder = m_tiltMotor.getEncoder();
    m_shootEncoder = m_shootMotor.getEncoder();
    //m_IntegratedtiltPID = m_tiltMotor.getPIDController();
    m_tiltPID = new PIDController(0.05, 0.0, 0.0);

    m_shootPID = m_shootMotor.getPIDController();
    m_tiltShaftEncoder = m_tiltMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    //m_IntegratedtiltPID.setFeedbackDevice(m_tiltEncoder);
    m_tiltShaftEncoder.setInverted(true);
    m_tiltShaftEncoder.setPositionConversionFactor(360.0);
    m_tiltShaftEncoder.setPosition(0);
    // m_tiltPID.setFeedbackDevice(m_tiltShaftEncoder);
    // BE() CAREFUL WITH ALTERNATE ENCODER m_tiltMotor.restoreFactoryDefaults(); BE
    // CAREFUL WITH ALTERNATE ENCODER()
    m_shootMotor.restoreFactoryDefaults();
    m_shootMotor.setIdleMode(IdleMode.kCoast);
    m_tiltMotor.setIdleMode(IdleMode.kBrake);
    m_shooterIR = new DigitalInput(2);
    m_shootPID.setP(0.00025);
    m_shootPID.setI(0.0000015);
    m_shootPID.setIZone(200);
    m_shootPID.setD(0);

    m_tiltMotor.setInverted(false);

    // m_IntegratedtiltPID.setP(0.01);
    // m_IntegratedtiltPID.setI(0.0);
    // m_IntegratedtiltPID.setIZone(0);
    // m_IntegratedtiltPID.setD(0);

    setTiltMotorPositionToZero();

    m_intakeTimer = new Timer();
    m_autonTimer = new Timer();
    m_shooterIRWasPreviouslyTriggered = false;
    m_shootMotor.enableVoltageCompensation(10.0);
    m_tiltMotor.enableVoltageCompensation(10.0);
    setSoftLimits();
    // m_shootPID.setFF(1 / 5400);

  }

  public void setSoftLimits() {
    m_tiltMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.kTiltOutSoftLimit);
    m_tiltMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_tiltMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.kTiltInSoftLimit);
    m_tiltMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

  }

  public double tiltShaftEncoderCountsToDegrees(double encoderCounts) {
    // double degrees = ((122 /-0.388) * encoderCounts) + 130;
    // return degrees;
    return encoderCounts;
  }

  public double getTiltSoftLimit() {
    return m_tiltMotor.getSoftLimit(SoftLimitDirection.kReverse);
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

  public CommandBase tiltDoNothingCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          m_tiltMotor.set(0);
        });
  }

  public void setTiltMotorPositionToZero() {
    m_tiltShaftEncoder.setInverted(true);
    m_tiltShaftEncoder.setPosition(150.0);
    m_tiltShaftEncoder.setPositionConversionFactor(360.0);
  }

  public CommandBase zeroTiltMotorEncoderCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_tiltEncoder.setPosition(0.0);
        });
  }

  public CommandBase zeroTiltMotorCommand() {
    return new ZeroIntake(this);
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

  public boolean isRunningSlowly() {
    return Math.abs(m_tiltEncoder.getVelocity()) < 1;
  }

  public void setTiltMotorPower(double power) {
    m_tiltMotor.set(power);
  }

  public double getTiltAngleDegrees(){
    return m_tiltShaftEncoder.getPosition();
  }

  public double getTiltAngleRadians(){
    return Math.toRadians(getTiltAngleDegrees());
  }

  public CommandBase tiltToDegreesCommand(double targetDegrees) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          
          double feedForwardPercent = Math.cos(getTiltAngleRadians()) / 10.0;
          if (getTiltAngleDegrees() >= 85 && getTiltAngleDegrees() <= 95 ){
            feedForwardPercent = Math.cos(getTiltAngleRadians()) / 20.0;
          }
          double feedForwardVoltage = feedForwardPercent * Constants.kTiltVoltageCompensation;
          // m_tiltPID.setArbi
          // m_tiltPID.setReference(position, ControlType.kPosition, 0, feedForwardVoltage);
          double outputVoltage = m_tiltPID.calculate(getTiltAngleDegrees(), targetDegrees) + feedForwardVoltage;
          outputVoltage = MathUtil.clamp(outputVoltage, -3.0, 3.0);
          m_tiltMotor.setVoltage(outputVoltage);

         

        });
  }

  public void zeroTiltMotor() {
    m_tiltEncoder.setPosition(0.0);
  }

  public double getTiltMotorPosition() {
    return m_tiltEncoder.getPosition();
  }

  public CommandBase runTiltMotorCommand(double power) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          setTiltMotorPower(power);
        });
  }

  public CommandBase runTiltMotorCommand(DoubleSupplier power) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          SmartDashboard.putNumber("Tilt Motor Power", power.getAsDouble());
          setTiltMotorPower(power.getAsDouble());
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

    SmartDashboard.putNumber("tilt Position Degrees",
        tiltShaftEncoderCountsToDegrees(m_tiltShaftEncoder.getPosition()));
    SmartDashboard.putNumber("tilt position encoder counts", getTiltMotorPosition());
    SmartDashboard.putNumber("tilt motor alternate encoder", m_tiltShaftEncoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ZeroIntake;

public class Tilt extends SubsystemBase {

  private CANSparkMax m_tiltMotor = new CANSparkMax(Constants.kTiltCANID, MotorType.kBrushless);
  private RelativeEncoder m_tiltEncoder;
  private PIDController m_tiltPID;
  private RelativeEncoder m_tiltShaftEncoder;

  public Tilt() {
    m_tiltEncoder = m_tiltMotor.getEncoder();
    m_tiltPID = new PIDController(0.05, 0.0, 0.0);
    m_tiltShaftEncoder = m_tiltMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    m_tiltShaftEncoder.setInverted(true);
    m_tiltShaftEncoder.setPositionConversionFactor(360.0);
    m_tiltShaftEncoder.setPosition(Constants.kTiltFullyRetractedAngleDegrees);
    m_tiltMotor.setIdleMode(IdleMode.kBrake);
    m_tiltMotor.setInverted(false);
    m_tiltMotor.enableVoltageCompensation(10.0);
    setTiltMotorPositionToZero();
    setSoftLimits();

  }

  public void setSoftLimits() {
    m_tiltMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.kTiltOutSoftLimit);
    m_tiltMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_tiltMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.kTiltInSoftLimit);
    m_tiltMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
  }

  public double getTiltSoftLimit() {
    return m_tiltMotor.getSoftLimit(SoftLimitDirection.kReverse);
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
    m_tiltShaftEncoder.setPositionConversionFactor(360.0);
    m_tiltShaftEncoder.setPosition(Constants.kTiltFullyRetractedAngleDegrees);
  }


  public CommandBase zeroTiltMotorCommand() {
    return new ZeroIntake(this);
  }

  public boolean isRunningSlowly() {
    return Math.abs(m_tiltEncoder.getVelocity()) < 1;
  }

  public void setTiltMotorPower(double power) {
    m_tiltMotor.set(power);
  }

  public double getTiltAngleDegrees() {
    return m_tiltShaftEncoder.getPosition();
  }

  public double getTiltAngleRadians() {
    return Math.toRadians(getTiltAngleDegrees());
  }

  public CommandBase tiltToDegreesCommand(double targetDegrees, boolean checkDoneness) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    CommandBase thing = run(
        () -> {
          //System.out.println("Stuff" + targetDegrees);
          double feedForwardPercent = Math.cos(getTiltAngleRadians()) / 10.0;
          if (getTiltAngleDegrees() >= 85 && getTiltAngleDegrees() <= 95) {
            feedForwardPercent = Math.cos(getTiltAngleRadians()) / 20.0;
          }
          double feedForwardVoltage = feedForwardPercent * Constants.kTiltVoltageCompensation;
          // m_tiltPID.setArbi
          // m_tiltPID.setReference(position, ControlType.kPosition, 0,
          // feedForwardVoltage);
          double outputVoltage = m_tiltPID.calculate(getTiltAngleDegrees(), targetDegrees) + feedForwardVoltage;
          outputVoltage = MathUtil.clamp(outputVoltage, -3.0, 3.0);
          m_tiltMotor.setVoltage(outputVoltage);

        });
    if (checkDoneness) {
      thing = thing.until(() -> Math.abs(getTiltAngleDegrees() - targetDegrees) <= 10);
    }
    return thing;

  }

  public void zeroTiltMotor() {
    m_tiltShaftEncoder.setPosition(Constants.kTiltFullyRetractedAngleDegrees);
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

  public CommandBase runTiltMotorCommandUntil(double power, double timeout) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.

    return runTiltMotorCommand(power).withTimeout(timeout);
  }

  public double tiltShaftEncoderCountsToDegrees(double encoderCounts) {
    // double degrees = ((122 /-0.388) * encoderCounts) + 130;
    // return degrees;
    return encoderCounts;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("tilt Position Degrees",
        tiltShaftEncoderCountsToDegrees(m_tiltShaftEncoder.getPosition()));
    SmartDashboard.putNumber("tilt position encoder counts", getTiltMotorPosition());
    SmartDashboard.putNumber("tilt motor alternate encoder", m_tiltShaftEncoder.getPosition());

  }

}

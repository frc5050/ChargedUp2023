// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  CANSparkMax m_leftFront = new CANSparkMax(Constants.kLeftFrontCANID, MotorType.kBrushless);
  CANSparkMax m_leftRear = new CANSparkMax(Constants.kLeftRearCANID, MotorType.kBrushless);
  CANSparkMax m_leftMiddle = new CANSparkMax(Constants.kLeftMiddleCANID, MotorType.kBrushless);
  CANSparkMax m_rightFront = new CANSparkMax(Constants.kRightFrontCANID, MotorType.kBrushless);
  CANSparkMax m_rightRear = new CANSparkMax(Constants.kRightRearCANID, MotorType.kBrushless);
  CANSparkMax m_rightMiddle = new CANSparkMax(Constants.kRightMiddleCANID, MotorType.kBrushless);
  ProfiledPIDController m_driveStraightPID;
  PIDController m_turnPID;
  PIDController m_balancePID;
  RelativeEncoder m_leftEncoder;
  RelativeEncoder m_rightEncoder;
  double adjustedDistanceMeters;
  double lStartingPosition;
  double desiredChange;
  double currentChange;
  double startingAngle;
  AHRS m_navX = new AHRS();
  Timer m_shimmyTimer = new Timer();

  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(
      m_leftFront,
      m_leftRear,
      m_leftMiddle);

  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(
      m_rightFront,
      m_rightRear,
      m_rightMiddle);

  private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0, 0);

  // private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors,
  // m_rightMotors);
  public static void configMotor(CANSparkMax motor, boolean isInverted) {
    motor.restoreFactoryDefaults();
    motor.setSmartCurrentLimit(Constants.kDriveSmartCurrentLimit);
    motor.setIdleMode(IdleMode.kBrake);
    motor.enableVoltageCompensation(12.0);
    motor.setInverted(isInverted);
  }

  public Drive() {
    configMotor(m_rightFront, true);
    configMotor(m_rightMiddle, true);
    configMotor(m_rightRear, true);
    configMotor(m_leftFront, false);
    configMotor(m_leftMiddle, false);
    configMotor(m_leftRear, false);

    m_leftEncoder = m_leftFront.getEncoder();
    m_rightEncoder = m_rightFront.getEncoder();

    m_rightRear.follow(m_rightFront);
    m_leftRear.follow(m_leftFront);
    m_rightMiddle.follow(m_rightFront);
    m_leftMiddle.follow(m_leftFront);

    m_rightEncoder.setPosition(0);
    m_leftEncoder.setPosition(0);

    m_rightMotors.setInverted(false);
    m_leftMotors.setInverted(false);

    m_driveStraightPID = new ProfiledPIDController(Constants.kStraightP, 0.0, 0.0,
        new TrapezoidProfile.Constraints(1, 0.3));
    m_turnPID = new PIDController(0.01, 0, 0.0);
    m_turnPID.enableContinuousInput(-180, 180);

    m_turnPID.setTolerance(Constants.kTurnPIDPositionTolerance, Constants.kTurnPIDVelocityTolerance);

    m_balancePID = new PIDController(0.003, 0, 0.0);
    m_balancePID.setTolerance(Constants.kBalanceTolerance);

    m_leftEncoder.setPositionConversionFactor(Constants.kMotorRotationsToMeters);
    m_rightEncoder.setPositionConversionFactor(Constants.kMotorRotationsToMeters);
    m_leftEncoder.setVelocityConversionFactor(Constants.kVelocityConversionFactor);
    m_rightEncoder.setVelocityConversionFactor(Constants.kVelocityConversionFactor);

    startingAngle = 0.0;
  }

  public void resetNavX() {
    m_navX.reset();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  // public void controlBrake(boolean solenoidState){
  // m_brakeSolenoid.set(solenoidState);
  // }

  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public CommandBase shimmyCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_shimmyTimer.reset();

          m_shimmyTimer.start();
          startingAngle = m_navX.getAngle();
          System.out.println(startingAngle + "Shimmy");
        }).andThen(
            run(
                () -> {
                  double desiredTurnCounterClockwise = startingAngle + 15;
                  double desiredTurnClockwise = startingAngle - 15;
                  turnToAbsoluteAngleCommand(desiredTurnCounterClockwise);
                  turnToAbsoluteAngleCommand(desiredTurnClockwise);
                }))
        .until(() -> m_shimmyTimer.hasElapsed(0.5));
  }

  public CommandBase zeroDriveEncoderCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_rightEncoder.setPosition(0);
          m_leftEncoder.setPosition(0);
        });
  }

  public CommandBase balanceRollCommand(double setPoint) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          m_balancePID.setSetpoint(setPoint);
          double beginningRoll = m_navX.getRoll();
          double motorCommand = MathUtil.clamp(m_balancePID.calculate(beginningRoll), -0.25, 0.25);
          motorCommand = motorCommand + Math.copySign(Constants.kBalanceFeedForward, motorCommand);
          double leftPower = motorCommand;
          double rightPower = motorCommand;
          if (m_balancePID.atSetpoint()){
            leftPower = 0.0;
            rightPower = 0.0;
          }
          DifferentialDrive.WheelSpeeds spds = DifferentialDrive.tankDriveIK(rightPower, leftPower, false);
          // m_drive.tankDrive(rightPower,leftPower);
          m_leftMotors.set(spds.left);
          m_rightMotors.set(spds.right);


        });
  }

  public void setMotorPower(double speed) {
    setMotorSpeeds(speed/1.02, speed);
  }

  public void arcadeDrive(double fwd, double rot, boolean squareInputs) {
    DifferentialDrive.WheelSpeeds spds = DifferentialDrive.arcadeDriveIK(fwd, rot, squareInputs);
    setMotorSpeeds(spds.left, spds.right);
  }

  public void setMotorSpeeds(double left, double right) {
    SmartDashboard.putNumber("L Speed", left);
    SmartDashboard.putNumber("R Speed", right);
    m_leftMotors.set(left);
    m_rightMotors.set(right);

  }

  public CommandBase arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          arcadeDrive(fwd.getAsDouble(), rot.getAsDouble(), true);
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
    // This method will be called once per scheduler run
    // System.out.println("compressor: " + m_PneumaticHub.getCompressor());
    SmartDashboard.putNumber("Left Velocity", m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Velocity", m_rightEncoder.getVelocity());
    SmartDashboard.putNumber("Velocity Ratio", m_leftEncoder.getVelocity() / m_rightEncoder.getVelocity());
    SmartDashboard.putNumber("Angle", m_navX.getAngle());
    SmartDashboard.putNumber("Roll", m_navX.getRoll());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // This command takes distance but it also Math.abs's it so the sign of speed
  // determines the direction rather than distance meters

// driveDistanceStraightCommand(-5, 0.5, 0.0) => drive 5m backwards with max abs speed of 0.5 m/s
// driveDistanceStraightCommand(+5, 0.5, 0.0) => drive 5m forwards with max abs speed of 0.5 m/s



  public CommandBase driveDistanceCommand(double distanceMeters, double timSpeed, double rotation, double timAccel, double positionTolerance) {
    // double speed = m_driveStraightPID.calculate(speed, rotation)
    return runOnce(
        () -> {
          lStartingPosition = m_leftEncoder.getPosition();
          m_driveStraightPID.reset(lStartingPosition);
          m_driveStraightPID.setConstraints(new TrapezoidProfile.Constraints(Math.abs(timSpeed), timAccel));
          m_driveStraightPID.setTolerance(positionTolerance, 0.2);
          adjustedDistanceMeters = lStartingPosition + distanceMeters;
          m_driveStraightPID.setGoal(adjustedDistanceMeters);
          System.out.println("starting drive distance command" + adjustedDistanceMeters);
          desiredChange = distanceMeters;
          currentChange = 0;
        }).andThen(

            run(() -> {
              double currentPosition = m_leftEncoder.getPosition();
              double whereweshudbe = m_driveStraightPID.calculate(currentPosition);
              double velocity = m_driveStraightPID.getSetpoint().velocity + whereweshudbe;
              final double maxAbsSpeed = Math.abs(timSpeed) * 1.1;
              if (Math.abs(velocity) > maxAbsSpeed) {
                if (velocity >= 0) {
                  velocity = Math.abs(maxAbsSpeed);
                } else {
                  velocity = -Math.abs(maxAbsSpeed);
                }
              }
              double speed = velocity / 2;
              SmartDashboard.putNumber("Velocity over Speed", m_leftEncoder.getVelocity()/speed);
              System.out.println("left position: " + lStartingPosition);

              System.out.println("desired change: " + desiredChange);
              SmartDashboard.putNumber("Desired Change", desiredChange);
              System.out.println("current change: " + currentChange);
              currentChange = currentPosition - lStartingPosition;
              System.out.println("Speed: " + speed);
              System.out.println("Current Position: " + currentPosition);
              SmartDashboard.putNumber("Current Position", currentPosition);
              System.out.println("Adjusted Distance (Goal): " + adjustedDistanceMeters);
              SmartDashboard.putNumber("Drive Distance Goal", adjustedDistanceMeters);
              System.out.println("Error:" + m_driveStraightPID.getPositionError());
              SmartDashboard.putNumber("Error", m_driveStraightPID.getPositionError());
              arcadeDrive(speed, rotation, false);
            })
                // End command when we've traveled the specified distance
                .until(
                  // () -> (currentChange / desiredChange) >= 1)
                  () -> m_driveStraightPID.atGoal())
                  //() -> m_driveStraightPID.atSetpoint())
                  // Stop the drive when the command ends
                .finallyDo(interrupted -> {
                  m_leftMotors.stopMotor();
                  m_rightMotors.stopMotor();
                  System.out.println("interrupted: " + interrupted);
                  System.out.println("motor power" + m_leftMotors.get());
                })

    );

  }

  public CommandBase driveDistanceCommand(double distanceMeters, double timSpeed, double rotation) {
    return driveDistanceCommand(distanceMeters, timSpeed, rotation, Constants.kDriveTimAccel, Constants.kDriveDistanceTolerance);
  }



  public CommandBase turnToAbsoluteAngleCommand(double angleDegrees) {
    return runOnce(
        () -> {
          System.out.println("starting turn to angle command");
          m_turnPID.reset();
          m_turnPID.setSetpoint(angleDegrees);
        }).beforeStarting(
            run(
                () -> {
                  m_turnPID.setSetpoint(angleDegrees);
                  double beginningAngle = -m_navX.getAngle();
                  double angleCommand = MathUtil.clamp(m_turnPID.calculate(beginningAngle), -0.2, 0.2);
                  double leftPower = -angleCommand;
                  double rightPower = angleCommand;
                  setMotorSpeeds(leftPower, rightPower);
                })
                .until(() -> m_turnPID.atSetpoint())
                .finallyDo(interrupted -> {
                  m_leftMotors.stopMotor();
                  m_rightMotors.stopMotor();
                })
                );

  }

  public CommandBase zeroYawCommand (){
    return runOnce (
      () -> {
        m_navX.zeroYaw();
      }
    );
  }

  public CommandBase turnToRelativeAngleCommand(double angleDegrees) {
    return runOnce(
        () -> {
          System.out.println("starting turn to angle command");
          m_turnPID.reset();
          m_turnPID.setSetpoint(angleDegrees - m_navX.getAngle());
        }).beforeStarting(
            run(
                () -> {
                  m_turnPID.setSetpoint(angleDegrees);
                  m_navX.zeroYaw();
                  double beginningAngle = -m_navX.getAngle();
                  double angleCommand = MathUtil.clamp(m_turnPID.calculate(beginningAngle), -0.2, 0.2);
                  double leftPower = -angleCommand;
                  double rightPower = angleCommand;
                  setMotorSpeeds(leftPower, rightPower);
                })
                .until(() -> m_turnPID.atSetpoint())
                .finallyDo(interrupted -> {
                  m_leftMotors.stopMotor();
                  m_rightMotors.stopMotor();
                })
                );

  }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDController2;

public class Drive extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);
  CANSparkMax m_leftFront = new CANSparkMax(Constants.kLeftFrontCANID, MotorType.kBrushless);
  CANSparkMax m_leftRear =  new CANSparkMax(Constants.kLeftRearCANID, MotorType.kBrushless);
  CANSparkMax m_leftMiddle = new CANSparkMax(Constants.kLeftMiddleCANID, MotorType.kBrushless);
  CANSparkMax m_rightFront = new CANSparkMax(Constants.kRightFrontCANID, MotorType.kBrushless);
  CANSparkMax m_rightRear = new CANSparkMax(Constants.kRightRearCANID, MotorType.kBrushless);
  CANSparkMax m_rightMiddle = new CANSparkMax(Constants.kRightMiddleCANID, MotorType.kBrushless);
  PneumaticHub m_PneumaticHub = new PneumaticHub(Constants.kPHCANID);
  SparkMaxPIDController m_rightPID;
  SparkMaxPIDController m_leftPID;
  PIDController m_turnPID;
  PIDController m_tiltPID;
  RelativeEncoder m_leftEncoder;
  RelativeEncoder m_rightEncoder;
  double adjustedDistanceMeters;
  double lStartingPosition; 
  double desiredChange;
  double currentChange;
  double startingAngle;
  AHRS m_navX = new AHRS();
  Timer m_shimmyTimer = new Timer();

  private final MotorControllerGroup m_leftMotors = 
    new MotorControllerGroup( 
        m_leftFront,
        m_leftRear,
        m_leftMiddle);

  private final MotorControllerGroup m_rightMotors = 
    new MotorControllerGroup(
        m_rightFront,
        m_rightRear,
        m_rightMiddle);

  private Solenoid m_brakeSolenoid = new Solenoid(Constants.kPHCANID,PneumaticsModuleType.REVPH, 7);
  
  private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0, 0);

  // private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  public Drive() {

  
    m_PneumaticHub.enableCompressorDigital();
    m_compressor.enableDigital();

    m_leftFront.restoreFactoryDefaults();
    m_leftMiddle.restoreFactoryDefaults();
    m_leftRear.restoreFactoryDefaults();
    m_rightFront.restoreFactoryDefaults();
    m_rightMiddle.restoreFactoryDefaults();
    m_rightRear.restoreFactoryDefaults();

    m_leftEncoder = m_leftFront.getEncoder();
    m_rightEncoder = m_rightFront.getEncoder();

    m_rightRear.follow(m_rightFront);
    m_leftRear.follow(m_leftFront);
    m_rightMiddle.follow(m_rightFront);
    m_leftMiddle.follow(m_leftFront);

    m_rightEncoder.setPosition(0);
    m_leftEncoder.setPosition(0);

    m_rightFront.setInverted(true);
    m_rightRear.setInverted(true);
    m_rightMiddle.setInverted(true);
    m_rightMotors.setInverted(false);
    m_leftMotors.setInverted(false);

    m_leftFront.enableVoltageCompensation(12.0);
    m_leftMiddle.enableVoltageCompensation(12.0);
    m_leftRear.enableVoltageCompensation(12.0);
    m_rightFront.enableVoltageCompensation(12.0);
    m_rightMiddle.enableVoltageCompensation(12.0);
    m_rightRear.enableVoltageCompensation(12.0);

    m_rightPID = m_rightFront.getPIDController();
    m_leftPID = m_leftFront.getPIDController();
    m_turnPID = new PIDController(0.02, 0, 0.0001);
    m_turnPID.enableContinuousInput(-180, 180);
    
    m_turnPID.setTolerance(2.0, 1.0);

    m_tiltPID = new PIDController(0.08, 0, 0);
    m_tiltPID.setTolerance(5);

    m_leftEncoder.setPositionConversionFactor(Constants.kMotorRotationsPerWheelRotations);
    m_rightEncoder.setPositionConversionFactor(Constants.kMotorRotationsPerWheelRotations);

    m_rightFront.setIdleMode(IdleMode.kCoast);
    m_rightMiddle.setIdleMode(IdleMode.kCoast);
    m_rightRear.setIdleMode(IdleMode.kCoast);
    m_leftFront.setIdleMode(IdleMode.kCoast);
    m_leftMiddle.setIdleMode(IdleMode.kCoast);
    m_leftRear.setIdleMode(IdleMode.kCoast);

    startingAngle = 0.0;
  }

  public void resetNavX(){
    m_navX.reset();
  }


  /**
   * Example command factory method.
   *
   * @return a command
   */

   public void controlBrake(boolean solenoidState){
    m_brakeSolenoid.set(solenoidState);
   }
   
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
      }
      ).andThen(
        run (
        () -> {
          double desiredTurnCounterClockwise = startingAngle + 15;
          double desiredTurnClockwise = startingAngle - 15;
          turnToAngleCommand(desiredTurnCounterClockwise);
          turnToAngleCommand(desiredTurnClockwise);
        })).until(()->m_shimmyTimer.hasElapsed(0.5));
  }


  public CommandBase zeroDriveEncoderCommand( ) {
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
          m_tiltPID.setSetpoint(setPoint);
          double beginningRoll = m_navX.getRoll();
          double motorCommand =  MathUtil.clamp(m_tiltPID.calculate(beginningRoll), -0.25, 0.25);
          double leftPower = motorCommand;
          double rightPower = motorCommand; 
          DifferentialDrive.WheelSpeeds spds = DifferentialDrive.tankDriveIK(rightPower, leftPower, true);
          // m_drive.tankDrive(rightPower,leftPower);
          m_leftMotors.set(spds.left);
          m_rightMotors.set(spds.right);
        });
  }

  public void setMotorPower(double speed){

    m_leftMotors.set(speed);
    m_rightMotors.set(speed);
  }





  public CommandBase controlBrakeCommand(boolean solenoidState) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> 
          m_brakeSolenoid.set(solenoidState)
        );
  }

  public void arcadeDrive(double fwd, double rot, boolean squareInputs){
    DifferentialDrive.WheelSpeeds spds = DifferentialDrive.arcadeDriveIK(fwd, rot, true);
         setMotorSpeeds(spds.left, spds.right);
  }
  public void setMotorSpeeds(double left, double right) {

    m_leftMotors.set(left / 1.06);
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
    System.out.println("compressor: " + m_PneumaticHub.getCompressor());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  //This command takes distance but it also Math.abs's it so the sign of speed determines the direction rather than distance meters
  public CommandBase driveDistanceCommand(double distanceMeters, double speed, double rotation) {
    return runOnce(
            () -> {
              System.out.println("starting drive distance command");
              lStartingPosition = m_leftEncoder.getPosition() / Constants.kMetersToWheelRotations;
              adjustedDistanceMeters = lStartingPosition + distanceMeters;
              desiredChange = distanceMeters;
              currentChange = 0;
            }
    ).andThen(
    
    run(()  -> {

    double currentPosition =   m_leftEncoder.getPosition() / Constants.kMetersToWheelRotations;

            System.out.println("left position: " + lStartingPosition);
              
              System.out.println("desired change: " + desiredChange);
              System.out.println("current change: " + currentChange);
              currentChange = currentPosition - lStartingPosition;


                arcadeDrive(speed, rotation, true);
  })
        // End command when we've traveled the specified distance
        .until(
            () ->
            (currentChange / desiredChange) >= 1)
        // Stop the drive when the command ends
        .finallyDo(interrupted -> {
               m_leftMotors.stopMotor();
               m_rightMotors.stopMotor();
               System.out.println("interrupted: " + interrupted);
               System.out.println("motor power" + m_leftMotors.get());   })

        
  );

  }

  public CommandBase turnToAngleCommand (double angleDegrees) {
    return runOnce(
              () -> {
                System.out.println("starting turn to angle command");
                m_turnPID.reset();
                m_turnPID.setSetpoint(angleDegrees);
              }
    ).beforeStarting(
      run(
          () -> {
            m_turnPID.setSetpoint(angleDegrees);
            double beginningAngle = -m_navX.getAngle();
            double angleCommand = MathUtil.clamp(m_turnPID.calculate(beginningAngle), -0.2, 0.2);
            
            double leftPower = -angleCommand;
            double rightPower = angleCommand;
            setMotorSpeeds(leftPower, rightPower);
          }).until(()->m_turnPID.atSetpoint()));
    
    
    
  }



}


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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDController2;

public class Drive extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  CANSparkMax m_leftFront = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax m_leftRear =  new CANSparkMax(6, MotorType.kBrushless);
  CANSparkMax m_rightFront = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax m_rightRear = new CANSparkMax(7, MotorType.kBrushless);
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
  AHRS m_navX = new AHRS();

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

    m_rightRear.follow(m_rightFront);
    m_leftRear.follow(m_leftFront);

    m_rightEncoder.setPosition(0);
    m_leftEncoder.setPosition(0);

    m_rightFront.setInverted(true);
    m_rightRear.setInverted(true);
    m_rightMotors.setInverted(false);
    m_leftMotors.setInverted(false);

    m_leftFront.enableVoltageCompensation(12.0);
    m_leftRear.enableVoltageCompensation(12.0);
    m_rightFront.enableVoltageCompensation(12.0);
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
          m_drive.tankDrive(rightPower,leftPower);
     
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
  public CommandBase arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> m_drive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble(), true)         
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


                m_drive.arcadeDrive(speed, rotation);
  })
        // End command when we've traveled the specified distance
        .until(
            () ->
            (currentChange / desiredChange) >= 1)
        // Stop the drive when the command ends
        .finallyDo(interrupted -> {
               m_drive.stopMotor();
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
            m_drive.tankDrive(leftPower, rightPower, false);
  
            
          }).until(()->m_turnPID.atSetpoint()));
    
    
    
  }



}


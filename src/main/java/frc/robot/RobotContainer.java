// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.autos.DriveAuto;
import frc.robot.autos.LongSideStartRetrieveAndPark;
import frc.robot.autos.MiddleStartAndPark;
import frc.robot.autos.PickUpCube;
import frc.robot.autos.SideStartAndPark;
import frc.robot.autos.SideStartNeverGiveUp;
import frc.robot.autos.shimmy;
import frc.robot.autos.zeroTest;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.HighConeCommand;
import frc.robot.commands.ZeroIntake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lifter;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public Drive m_drive = new Drive();
  public Intake m_intake = new Intake();
  public static DriveAuto m_autos = new DriveAuto();
  public Lifter m_lifter = new Lifter();
  CommandJoystick m_joystick = new CommandJoystick(1);


  public void periodic(){
    m_intake.m_Solenoid.set(true);
  }


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    SmartDashboard.putData(m_drive);
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    m_drive.setDefaultCommand(  
            m_drive.arcadeDriveCommand(
          () -> -m_joystick.getY(), () -> -m_joystick.getX()) );  

    m_intake.setDefaultCommand(m_intake.tiltDoNothingCommand());

    //m_intake.setDefaultCommand(m_intake.runTiltMotorCommand(m_driverController.getLeftX()));

  
    

    m_lifter.setDefaultCommand(m_lifter.elevatorDoNothingCommand());

    m_joystick.button(8).whileTrue(m_drive.controlBrakeCommand(false));
    m_joystick.button(8).whileFalse(m_drive.controlBrakeCommand(true));

    //tilt 
    m_driverController.povUp().whileTrue(m_intake.runTiltMotorCommand(-0.5));
    m_driverController.povCenter().whileTrue(m_intake.runTiltMotorCommand(0.0));
    m_driverController.povDown().whileTrue(m_intake.runTiltMotorCommand(0.5));

    //intake
    m_driverController.x().whileTrue(m_intake.runShootMotorCommand(0.5));
    m_driverController.x().whileFalse(m_intake.runShootMotorCommand(0));

    //outtake 
    m_driverController.b().whileTrue(m_intake.runShootMotorCommand(-0.5));
    m_driverController.b().whileFalse(m_intake.runShootMotorCommand(0));

    //medium height shot
    m_joystick.button(5).whileTrue(m_intake.runShootMotorCommand(-0.5));
    m_joystick.button(5).whileFalse(m_intake.runShootMotorCommand(0.0));
    //m_driverController.leftBumper().whileTrue(m_intake.runShootMotorCommandwithPID(-9000, ControlType.kVelocity));
    //m_driverController.leftBumper().whileTrue(m_intake.shootPopCommand(true));
    //m_driverController.leftBumper().whileFalse(m_intake.shootPopCommand(false));


    //high
    m_joystick.button(6).whileTrue(m_intake.runShootMotorCommand(-0.7));
    m_joystick.button(6).whileFalse(m_intake.runShootMotorCommand(0.0));
    //m_driverController.rightBumper().whileTrue(m_intake.runShootMotorCommandwithPID(-14000, ControlType.kVelocity));
    //m_driverController.rightBumper().whileTrue(m_intake.shootPopCommand(true));
    //m_driverController.rightBumper().whileFalse(m_intake.shootPopCommand(false));


    //shoot pneumatic (which we don't use anymore)
    //m_driverController.leftTrigger().whileTrue(m_intake.shootPopCommand(true));
    //m_driverController.leftTrigger().whileFalse(m_intake.shootPopCommand(false));

    // //cone intake
     m_driverController.a().whileTrue(m_lifter.coneIntakeCommand(1.0));
     m_driverController.a().whileFalse(m_lifter.coneIntakeCommand(0.0));

    // //and cone outtake
     m_driverController.y().whileTrue(m_lifter.coneIntakeCommand(-1.0));
     m_driverController.y().whileFalse(m_lifter.coneIntakeCommand(0.0));

    // //elevator down
     m_driverController.rightStick().whileTrue(m_lifter.runElevatorCommand(0.7));
    m_driverController.rightStick().whileFalse(m_lifter.runElevatorCommand(0));

    //tilt to position
    m_driverController.rightTrigger().whileTrue(m_intake.tiltToPositionCommand(Constants.kTiltConePickUpHumanPlayerPosition));
    m_driverController.rightTrigger().whileFalse(m_intake.getDefaultCommand());
   

    // //elevator up
       m_driverController.leftStick().whileTrue(m_lifter.runElevatorCommand(-0.7));
       m_driverController.leftStick().whileFalse(m_lifter.runElevatorCommand(0.0));



    //m_joystick.button(12).whileTrue(m_drive.turnToAngleCommand(90));
    //m_joystick.button(12).whileFalse(m_drive.getDefaultCommand());

    m_joystick.button(7).whileTrue(m_drive.balanceRollCommand(0.0));
    m_joystick.button(7).whileFalse(m_drive.getDefaultCommand());

    m_joystick.button(12).whileTrue(m_intake.zeroTiltMotorCommand());
    m_joystick.button(12).whileFalse(m_intake.getDefaultCommand());

    m_joystick.button(10).whileTrue(HighConeCommand.HighConeConfigurationCommand(m_intake, m_lifter));
    m_joystick.button(10).whileFalse(m_lifter.getDefaultCommand());

    m_joystick.button(9).whileTrue(m_lifter.elevatorPIDCommand(Constants.kElevatorMiddlePosition));
    m_joystick.button(9).whileFalse(m_lifter.getDefaultCommand());

   

  

    

    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


   //sendable chooser goes here to implement multiple autons 
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return SideStartNeverGiveUp.sideStartNeverGiveUpCommand(m_drive, m_intake);
    //MiddleStartAndPark.middleStartAndParkCommand(m_drive, m_intake);
    //SideStartRetrieveAndPark.sideStartRetrieveAndParkCommand(m_drive, m_intake);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.autos.DoStartUpTasks;
import frc.robot.autos.DriveAuto;
import frc.robot.autos.MiddleStartConeAndPark;
import frc.robot.autos.MiddleStartConeLeaveAndPark;
import frc.robot.autos.MiddleStartCubeAndPark;
import frc.robot.autos.MiddleStartCubeLeaveAndPark;
import frc.robot.autos.SideStartPlaceConePickUpCone;
import frc.robot.autos.SideStartPlaceConePickUpConeTurn;
import frc.robot.autos.SideStartPlaceConePickUpCube;
import frc.robot.autos.SideStartPlaceConePickUpCubeTurn;
import frc.robot.autos.SideStartShootCubePickUpCone;
import frc.robot.autos.SideStartShootCubePickUpCube;
import frc.robot.autos.doNothing;
import frc.robot.commands.FeederConeIntakeCommand;
import frc.robot.commands.HighConeCommand;
import frc.robot.commands.HighFeederConeIntakeCommand;
import frc.robot.commands.LEDTestCommand;
import frc.robot.commands.MediumConeCommand;
import frc.robot.subsystems.Brake;
import frc.robot.subsystems.ConeIntake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Tilt;
import frc.robot.subsystems.LED;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public Drive m_drive = new Drive();
  public Intake m_intake = new Intake();
  public static DriveAuto m_autos = new DriveAuto();
  public Lifter m_lifter = new Lifter();
  public Brake m_brake = new Brake();
  public Tilt m_tilt = new Tilt();
  public LED m_led = new LED();
  public ConeIntake m_coneIntake = new ConeIntake();
  CommandJoystick m_joystick = new CommandJoystick(1);

  private final Command m_middleStartConeLeaveAndPark = MiddleStartConeLeaveAndPark.middleStartAndParkCommand(m_drive, m_intake,
      m_lifter, m_tilt, m_coneIntake, m_brake);
  private final Command m_middleStartCubeLeaveAndPark = MiddleStartCubeLeaveAndPark.middleStartAndParkCommand(m_drive, m_intake,
      m_tilt, m_brake);
  private final Command m_doNothing = doNothing.doNothingCommand();
  private final Command m_doStartUpTasks = DoStartUpTasks.doStartUpTasksCommand(m_tilt, m_drive, m_brake);
  private final Command m_sideStartPlaceConePickUpCube = SideStartPlaceConePickUpCube
      .sideStartNeverGiveUpCommand(m_drive, m_intake, m_brake, m_tilt, m_lifter, m_coneIntake);
  private final Command m_sideStartShootCubePickUpCube = SideStartShootCubePickUpCube
      .sideStartNeverGiveUpCommand(m_drive, m_intake, m_brake, m_tilt);

  private final Command m_sideStartShootCubePickUpCone = SideStartShootCubePickUpCone.sideStartShootCubePickUpConeCommand(m_drive, m_intake, m_brake, m_tilt, m_coneIntake);
  private final Command m_sideStartPlaceConePickUpCone = SideStartPlaceConePickUpCone.sideStartNeverGiveUpCommand(m_drive, m_intake, m_brake, m_tilt, m_lifter, m_coneIntake);

    private final Command m_middleStartCubeAndPark = MiddleStartCubeAndPark.middleStartAndParkCommand(m_drive, m_intake, m_tilt, m_brake);

    private final Command m_middleStartConeAndPark = MiddleStartConeAndPark.middleStartAndParkCommand(m_drive, m_intake, m_lifter, m_tilt, m_coneIntake, m_brake);
    private final Command m_sideStartPlaceConePickUpConeTurn = SideStartPlaceConePickUpConeTurn.sideStartNeverGiveUpCommand(m_drive, m_intake, m_brake, m_tilt, m_lifter, m_coneIntake);
    private final Command m_sideStartPlaceConePickUpCubeTurn = SideStartPlaceConePickUpCubeTurn.sideStartNeverGiveUpCommand(m_drive, m_intake, m_brake, m_tilt, m_lifter, m_coneIntake);
  SendableChooser<Command> m_chooser = new SendableChooser<>();



  public void periodic() {
  }

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(Constants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    SmartDashboard.putData(m_drive);

    m_chooser.setDefaultOption("Default", m_doStartUpTasks);
    m_chooser.addOption("DoNothing", m_doNothing);
    m_chooser.addOption("MiddleStartCubeLeaveAndPark", m_middleStartCubeLeaveAndPark);
    m_chooser.addOption("MiddleStartConeLeaveAndPark", m_middleStartConeLeaveAndPark);
    m_chooser.addOption("SideStartShootCubePickUpCube", m_sideStartShootCubePickUpCube);
    m_chooser.addOption("SideStartPlaceConePickUpCube", m_sideStartPlaceConePickUpCube);
    m_chooser.addOption("MiddleStartCubeAndPark", m_middleStartCubeAndPark);
    m_chooser.addOption("MiddleStartConeAndPark" , m_middleStartConeAndPark);
    m_chooser.addOption("SideStartShootCubePickUpCone" , m_sideStartShootCubePickUpCone);
    m_chooser.addOption("SideStartPlaceConePickUpCone" , m_sideStartPlaceConePickUpCone);
    m_chooser.addOption("SideStartPlaceConePickUpCubeTurn", m_sideStartPlaceConePickUpCubeTurn);
    m_chooser.addOption("SideStartPlaceConePickUpConeTurn", m_sideStartPlaceConePickUpConeTurn);
    SmartDashboard.putData(m_chooser);
    CameraServer.startAutomaticCapture();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_drive.setDefaultCommand(
        m_drive.arcadeDriveCommand(
            () -> -m_joystick.getY(), () -> -m_joystick.getX()));

    m_intake.setDefaultCommand(m_intake.intakeDoNothingCommand());

    m_led.setDefaultCommand(m_led.controlLEDCommand(m_intake, m_tilt, m_coneIntake));

    m_brake.setDefaultCommand(m_brake.setBrakeCommand(m_joystick.button(8)));

    m_coneIntake.setDefaultCommand(m_coneIntake.coneIntakeDoNothingCommand());

    // m_tilt.setDefaultCommand(m_tilt.runTiltMotorCommand(() ->
    // m_driverController.getLeftY()));

    m_lifter.setDefaultCommand(m_lifter.elevatorDoNothingCommand());

    // tilt
    m_driverController.povUp().whileTrue(m_tilt.runTiltMotorCommand(Constants.kTiltMotorInMotorPower));
    m_driverController.povCenter().whileTrue(m_tilt.runTiltMotorCommand(0.0));
    m_driverController.povDown().whileTrue(m_tilt.runTiltMotorCommand(Constants.kTiltMotorOutMotorPower));

    // runTi
    m_tilt.setDefaultCommand(
        m_tilt.runTiltMotorCommand(
            () -> {
              if (m_driverController.povDown().getAsBoolean() || m_driverController.povDownLeft().getAsBoolean()
                  || m_driverController.povDownRight().getAsBoolean()) {
                return Constants.kTiltMotorInMotorPower;
              } else if (m_driverController.povUp().getAsBoolean() || m_driverController.povUpLeft().getAsBoolean()
                  || m_driverController.povUpRight().getAsBoolean()) {
                return Constants.kTiltMotorOutMotorPower;
              }
              return 0.0;
            }));

    // intake
    m_driverController.x().whileTrue(m_intake.runShootMotorCommand(0.5, m_led));

    // outtake
    m_driverController.b().whileTrue(m_intake.runShootMotorCommand(-0.5, m_led));

    // medium height shot
    m_joystick.button(5).whileTrue(m_intake.runShootMotorCommand(Constants.kMidShotMotorPower, m_led));

    // high shot
    m_joystick.button(6).whileTrue(m_intake.runShootMotorCommand(Constants.kHighShotMotorPower, m_led));

    // //cone intake
    m_driverController.a().whileTrue(m_coneIntake.coneIntakeCommand());

    // //and cone outtake
    m_driverController.y().whileTrue(m_coneIntake.coneOuttakeCommand(false));

    // //elevator down
    m_driverController.rightStick().whileTrue(m_lifter.runElevatorCommand(0.7));

    // tilt to position out
    // TODO
    m_driverController.rightTrigger().whileTrue(m_tilt.tiltToDegreesCommand(Constants.kTiltConeOutPosition, false));

    // tilt to position in
    // TODO
    m_driverController.leftTrigger().whileTrue(m_tilt.tiltToDegreesCommand(Constants.kTiltConeInPosition, false));

    // //elevator up
    m_driverController.leftStick().whileTrue(m_lifter.runElevatorCommand(-0.7));

    m_joystick.button(7).whileTrue(m_drive.balanceRollCommand(0.0));

    //m_joystick.button(12).whileTrue(m_tilt.zeroTiltMotorCommand());
    m_joystick.button(12).whileTrue(HighFeederConeIntakeCommand.HighConeConfigurationCommand(m_tilt, m_lifter));
    m_joystick.button(10).whileTrue(HighConeCommand.HighConeConfigurationCommand(m_tilt, m_lifter));

    m_joystick.button(9).whileTrue(MediumConeCommand.MediumConeConfigurationCommand(m_tilt, m_lifter));

    m_joystick.button(11).whileTrue(m_lifter.elevatorPIDTeleopCommand(Constants.kElevatorDownPosition));

    m_joystick.button(1).whileTrue(m_tilt.tiltToDegreesCommand(90, false));

    m_joystick.button(2).whileTrue(m_tilt.tiltToDegreesCommand(75, false));

    m_driverController.rightBumper().whileTrue(FeederConeIntakeCommand.intakeConeCommand(m_tilt, m_intake, m_coneIntake, m_led));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  // sendable chooser goes here to implement multiple autons
  public Command getAutonomousCommand() {

    return m_chooser.getSelected();

  }
}

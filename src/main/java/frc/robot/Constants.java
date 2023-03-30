// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // CAN IDs
  public static final int kPHCANID = 1;
  public static final int kLeftFrontCANID = 10;
  public static final int kLeftRearCANID = 14;
  public static final int kLeftMiddleCANID = 11;
  public static final int kRightFrontCANID = 3;
  public static final int kRightRearCANID = 21; // 21 is 1 on pdh
  public static final int kRightMiddleCANID = 2;
  public static final int kCubeIntakeCANID = 16;
  public static final int kTiltCANID = 13;
  public static final int kElevatorCANID = 20; // twenty is actually zero on pdh
  public static final int kConeIntakeCANID = 19;

  // controller ports
  public static final int kDriverControllerPort = 0;

  // driving

  // Note: makes turning slower
  public static final int kDriveSmartCurrentLimit = 35;

  public static final double kMotorRotationsPerWheelRotations = 1 / 8.45;
  public static final double kMetersToWheelRotations = 100.0 / (2.54 * 6.0 * Math.PI);
  public static final double kMotorRotationsToMeters = kMotorRotationsPerWheelRotations / kMetersToWheelRotations;
  public static final double kVelocityConversionFactor = (1.0 / 60.0) * kMotorRotationsToMeters;
  public static final int kSolenoidChannel = 5;
  public static final double kStraightP = 0.6;
  public static final double kDriveDistanceTolerance = 0.1;
  public static final double kBalancingAutonTimAccel = 0.7;
  public static final double kDriveTimAccel = 0.3;
  public static final double kNavXRollOffset = -1.75;
  public static final double kBalanceTolerance = 5;
  public static final double kBalanceFeedForward = 0.01;
  public static final double kDistanceOverStation = -4.0;
  public static final double kDistanceToStation = -1.9;
  public static final double kDistanceBackToStation = 1.5;
  public static final double kTurnPIDPositionTolerance = 2.0;
  public static final double kTurnPIDVelocityTolerance = 0.3;

  // shooting
  public static final double kShootingTimeOut = 1.0;
  public static final double kHighShotMotorPower = -0.95;
  public static final double kMidShotMotorPower = -0.58;
  public static final double kVeryLowShootPower = -0.4;

  // intake
  public static final double kIntakePower = 0.5;
  public static final double kAutoIntakeTiltOutPower = -0.3;
  public static final double kIntakeTiltZeroPower = 0.2;
  public static final float kTiltOutSoftLimit = -10;
  public static final float kTiltInSoftLimit = -2;
  public static final double kMidConePosition = 10.0;
  public static final double kIntakeIRDelay = 0.2;
  public static final double kTiltConeOutPosition = 10;
  public static final double kTiltConeInPosition = 140;
  public static final double kTiltConeHighFeederPositionDegrees = 5.0;
  public static final double kTiltConeHighPositionDegrees = 20; //Should be 25ish
  public static final double kTiltFeederPositionDegrees = 85;
  public static final double kTiltVoltageCompensation = 10.0;
  public static final double kConeOuttakeMotorPower = -1.0;
  public static final double kConeIntakeMotorPower = 1.0;
  public static final double kTiltMotorInMotorPower = 0.2;
  public static final double kTiltMotorOutMotorPower = -0.2;
  public static final double kTiltFullyRetractedAngleDegrees = 150.0;
  public static final double kAutoReturnTiltTimeout = 0.5;
  public static final double kIntakeTimeout = 1.0;
  public static final double kTiltHighCubePosition = 75.0 ;

  // elevator
  public static final float kElevatorTopSoftLimit = -126.6f;
  public static final float kElevatorBottomSoftLimit = 0;
  public static final double kElevatorHighConePosition = -126.58;
  public static final double kElevatorMiddlePosition = -96;
  public static final double kElevatorDownPosition = 0.0;
  public static final double kElevatorTolerance = 0.15;
  public static final double kAutonHeightWait = 45;
  public static final double kElevatorHighCubePosition = -126.58;

  //LEDS
  public static final int LEDPort = 0; 
  public static final int LED2Port = 1; 
  public static final int LEDCount = 20; 
  /* 
  this one is purple
  led.setLEDColor(25, 0, 29);
  this one is Yellow
  led.setLEDColor(106, 76, 0);
  This one is green
  led.setLEDColor(0, 110, 0);
  HAWT Pink Color Code
  m_led.setLEDColorCommand(200,11,37); 
  */
  

}

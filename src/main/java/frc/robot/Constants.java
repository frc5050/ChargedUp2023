// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  //CAN IDs
  public static final int kPHCANID = 1;
  public static final int kLeftFrontCANID = 10;
  public static final int kLeftRearCANID = 14;
  public static final int kLeftMiddleCANID = 11;
  public static final int kRightFrontCANID = 3;
  public static final int kRightRearCANID = 21; //21 is 1 on pdh
  public static final int kRightMiddleCANID = 2;
  public static final int kCubeIntakeCANID = 16;
  public static final int kTiltCANID = 13;
  public static final int kElevatorCANID = 20; // twenty is actually zero on pdh
  public static final int kConeIntakeCANID = 19;



  
  //controller ports
    public static final int kDriverControllerPort = 0;

  //driving
    
    // Note: makes turning slower
    public static final int kDriveSmartCurrentLimit = 25;

    public static final double kMotorRotationsPerWheelRotations = 1/8.45;
    public static final double kMetersToWheelRotations = 100.0 / (2.54 * 6.0 * Math.PI);
    public static final double kMotorRotationsToMeters = kMotorRotationsPerWheelRotations / kMetersToWheelRotations;
    public static final double kVelocityConversionFactor = (1.0/60.0) * kMotorRotationsToMeters;
    public static final int kSolenoidChannel = 5;
    public static final double kStraightP = 0.2;

    //shooting
    public static final double kShootingTimeOut = 1.0;
    public static final double kHighShotMotorPower = -1.0;

    //intake 
    public static final double kIntakePower = 0.5;
    public static final double kIntakeTiltOutPower = -0.3;
    public static final float kTiltOutSoftLimit = -32;
    public static final float kTiltInSoftLimit = -8;
    public static final double kConePosition = -29.6;
    public static final double kIntakeIRDelay = 0.03;
    public static final double kTiltConePickUpHumanPlayerPosition = -20.0;
    public static final double kTiltConeHighPosition = -28;


    //elevator
    public static final float kElevatorTopSoftLimit = -126.6f;
    public static final float kElevatorBottomSoftLimit = 0;
    public static final double kElevatorHighPosition = -126.58;
    public static final double kElevatorMiddlePosition = -96;

}

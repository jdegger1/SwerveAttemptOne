// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ModuleConstants {
    public static double kWheelDiameterMeters = Units.inchesToMeters(4);

    public static final double kDriveMotorGearRatio = 1/6.75;
    public static final double kTurningMotorGearRatio = (1/(150.0/7));
    
    
    //
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    
    
    //Used as position conversion factor
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter/60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad/60;
    //wtf this do
    
    //its our p term for the pid for turning - J
    public static final double kPTurning = 0.3;
    
  }

  public static class DriveConstants{
    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(15.5);
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(26.0);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(-kTrackWidth / 2, kWheelBase / 2),
      new Translation2d(kTrackWidth/ 2, kWheelBase / 2),
      new Translation2d(-kTrackWidth / 2, -kWheelBase / 2),
      new Translation2d(kTrackWidth / 2, -kWheelBase / 2)
      );



    public static final double kPhysicalMaxSpeedMetersPerSecond = 4.47;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
            kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    
    
    public static final int kFrontLeftDriveMotorPort = 11;
    public static final int kBackLeftDriveMotorPort = 3; 
    public static final int kFrontRightDriveMotorPort = 4;
    public static final int kBackRightDriveMotorPort = 10;

    public static final int kFrontLeftTurningMotorPort = 13;
    public static final int kBackLeftTurningMotorPort = 5; 
    public static final int kFrontRightTurningMotorPort = 14;
    public static final int kBackRightTurningMotorPort = 8;

    //Try messing with these reversed/not reversed values some more
    //look at what the shuffleboard values are vs what you want them to be
    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kBackLeftTurningEncoderReversed = false; 
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kBackRightTurningEncoderReversed = false;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 21;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 20;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 23; 
    public static final int kBackRightDriveAbsoluteEncoderPort = 22;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;
    //ZERO CANCODERS USING PHOENIX TUNER X INSTEAD
    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0; //21
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0; //20
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0; //23
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;  //22
  }

  public static final class OIConstants {
    public static final int kDriverControllerOnePort = 0;
    public static final int kDriverControllerTwoPort = 1;

    //CHECK CONTROLLER VALUES
    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxisXbox = 4;
    public static final int kDriverRotAxisJoystick = 2;


    ///////////////////////buttons\\\\\\\\\\\\\\\\\
    //////////////////////////Controller One\\\\\\\\\\\\\\\\\\\\\
    public static final int kDriverFieldOrientedButtonIdx =1;
    //x
    public static final int kDriverResetArmButton = 2;
    //B
    public static final int kRestGyrobutton = 3;
    //Y
    public static final int kShootSequenceButton = 4;
    //right Trigger
    public static final int kExtendLiftButton = 6;
   //left trigger
    public static final int kRetractLiftButton = 5;
    /////////////////////////////////Controller Two\\\\\\\\\\\\\\
    //Y
    public static final int kFlyWheelFwdButton = 4;
    //A
    public static final int kFlyWheelBwdButton = 1;
    //right trigger
    public static final int kArmForwardButton = 5;
    //left trigger
    public static final int kArmBackwardButton = 6;
    //B
    public static final int kHerderInButton = 3;
    //X
    public static final int kHerderOutButton = 2;

    //Plus up (POV button in degrees)
    public static final int kArmHerdButton = 0;
    //Plus Down
    public static final int kArmCloseSpeakerButton = 180;
    //plus right
    public static final int kArmAmpButton = 90;
    //plus left
    public static final int kArmFarSpeakerButton = 270;
    
    
    


    

    public static final double kDeadband = 0.15;
}
  
}
//https://software-metadata.revrobotics.com/REVLib-2025.json
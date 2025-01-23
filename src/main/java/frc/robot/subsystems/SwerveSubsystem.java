// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */

   private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort, 
        DriveConstants.kFrontLeftTurningMotorPort, 
        DriveConstants.kFrontLeftDriveEncoderReversed, 
        DriveConstants.kFrontLeftTurningEncoderReversed, 
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, 
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
        );
    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort, 
        DriveConstants.kFrontRightTurningMotorPort, 
        DriveConstants.kFrontRightDriveEncoderReversed, 
        DriveConstants.kFrontRightTurningEncoderReversed, 
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort, 
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
        );
    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort, 
        DriveConstants.kBackLeftTurningMotorPort, 
        DriveConstants.kBackLeftDriveEncoderReversed, 
        DriveConstants.kBackLeftTurningEncoderReversed, 
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort, 
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
        );
    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort, 
        DriveConstants.kBackRightTurningMotorPort, 
        DriveConstants.kBackRightDriveEncoderReversed, 
        DriveConstants.kBackRightTurningEncoderReversed, 
        DriveConstants.kBackRightDriveAbsoluteEncoderPort, 
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad, 
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed
        );

         //creates a navX gyro to use in da calcs
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, AHRS.NavXUpdateRate.k50Hz);
    
    // private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,new Rotation2d(0),
        new SwerveModulePosition[]{frontLeft.getPosition(),frontRight.getPosition(),backLeft.getPosition(),backRight.getPosition()});
    
  public SwerveSubsystem() {
    new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }
    //gets the heading returned as the gyro reading remainder after being divided by 360
    //that way it always reads from 0 to 360
    //or 0 to -360
    public double getHeading() {
        return -Math.IEEEremainder(gyro.getAngle(), 360);
    }
    //returns as radians?
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
    //returns Pose with x,y, and theta coordinates of robot
    public Pose2d getPose(){
        return odometer.getPoseMeters();
    }
    //reset the odometer current theta, module positions
    public void resetOdometry(Pose2d pose){
        odometer.resetPosition(getRotation2d(), 
        new SwerveModulePosition[]{frontLeft.getPosition(),frontRight.getPosition(),backLeft.getPosition(),backRight.getPosition()},
         pose);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

     //updates odometer based on new positions which take into account turn encoder and drive encoder along with position on robot
     odometer.update(getRotation2d(),new SwerveModulePosition[]{frontLeft.getPosition(),frontRight.getPosition(),backLeft.getPosition(),backRight.getPosition()} );
     SmartDashboard.putString("frontLeft", frontLeft.getPosition().toString());
     SmartDashboard.putString("frontRight", frontRight.getPosition().toString());
     SmartDashboard.putString("backLeft", backLeft.getPosition().toString());
     SmartDashboard.putString("backRight", backRight.getPosition().toString());
     SmartDashboard.putString("Robot Heading", getRotation2d().toString());

     SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
}

public void setModuleStates(SwerveModuleState[] desiredStates) {
  SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
  frontLeft.setDesiredState(desiredStates[0]);
  frontRight.setDesiredState(desiredStates[1]);
  backLeft.setDesiredState(desiredStates[2]);
  backRight.setDesiredState(desiredStates[3]);
}

}

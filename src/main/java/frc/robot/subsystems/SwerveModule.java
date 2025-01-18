// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */

  private SparkMax driveMotor;
  private SparkMax turningMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder turningEncoder;

  private CANcoder absoluteEncoder;

  private boolean absoluteEncoderReversed;

  private double absoluteEncoderOffsetRad;
  private int turningMotorId;
  private int absoluteEncoderId;

  private SparkMaxConfig config;

  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
      config = new SparkMaxConfig();
      this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
      this.absoluteEncoderReversed = absoluteEncoderReversed;

      this.absoluteEncoder = new CANcoder(absoluteEncoderId);

      driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
      turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
      this.turningMotorId = turningMotorId;
      this.absoluteEncoderId=absoluteEncoderId;

      driveEncoder = driveMotor.getEncoder();
      turningEncoder = turningMotor.getEncoder();

      config
        .inverted(driveMotorReversed)
        .idleMode(IdleMode.kCoast);
      config.encoder
        .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
        .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
      
      driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  

      config
        .inverted(turningMotorReversed)
        .idleMode(IdleMode.kCoast);
      config.encoder
        .positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
        .velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
      config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ModuleConstants.kPTurning, 0, 0);
      turningMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
      }

      //gets drive encoder position in meters
    public double getDrivePosition(){
        return driveEncoder.getPosition();
    }
    //gets turning encoder position in radians
    public double getTurningPosition(){
        return turningEncoder.getPosition();
    }
    //gets that swerve module position idk
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(),new Rotation2d(getTurningPosition()));
    }
    //gets drive encoder velocity in m/s
    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }
    //gets turning encoder velocity in rad/s
    public double getTurningVelocity(){
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad(){
      //getAbsolutePosition returns value from -0.5 to 0.5 (think cicle again (-0.5 is right next to 0.5))
      double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
      //boom angle is now in radians
      angle *= (2*Math.PI);
      //offsets the angle according to typed constants
      //pro tip: use Pheonix Tuner X to zero the absolute encoders manually instead
      //EVERY TIME THE ABSOLUTE ENCODERS ARE UNPLUGGED THEY NEED TO BE RE-ZEROED
      angle -= absoluteEncoderOffsetRad;
      //if they are reversed turn that john negative
      if(absoluteEncoderReversed){
          return angle*(-1.0);
      }else{
          return angle;
      }
  }
  public double getAbsoluteEncoderReading(){
      //literally just returns the method above this one
      //ik its basically useless but im not changing it now
      return getAbsoluteEncoderRad();
  }

  public void resetEncoders(){
        //zeros drive encoder
        driveEncoder.setPosition(0);
        //sets the relative turning encoder to it's absolute encoder's value in radians 
        //(remember we converted absolute to radians by multiplying by 2pi and 
        //relative to radians by using our gear ratio to create a conversion factor)
        turningEncoder.setPosition(getAbsoluteEncoderRad());

        
    }
    //gets absolute position of absolute encoders on a range of -0.5 to 0.5 (raw data with no conversions)
    public double getAbsolutePos(){
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    } 
    //gets the state of the module 
    public SwerveModuleState getState(){
        //creates a current state from the current drive velocity and turning position
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));    
    }
    
    
    //stop
    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

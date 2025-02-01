// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LadderConstants;

public class LadderSubsystem extends SubsystemBase {
  /** Creates a new LadderSubsystem. */

  //coded as brushless can code as brushed if neccesary.
  private final SparkMax liftMotor = new SparkMax(LadderConstants.kLiftMotorPort, MotorType.kBrushless);
  private final RelativeEncoder liftEncoder = liftMotor.getEncoder();
  
  //private final SparkMax liftMotor = new SparkMax(LadderConstants.kLiftMotorPort, MotorType.kBrushed);
  //private final RelativeEncoder liftEncoder = liftMotor.getAbsoluteEncoder();
  public LadderSubsystem() {}

  public void driveLift(double speed){
    liftMotor.set(speed);
  }

  public double getLiftEncoder(){return liftEncoder.getPosition();}

  public void setLiftEncoder(double val){
    liftEncoder.setPosition(val);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Height", getLiftEncoder());
  }
}

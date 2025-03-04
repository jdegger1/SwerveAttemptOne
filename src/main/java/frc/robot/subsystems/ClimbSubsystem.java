// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  private final SparkMax climbMotor;
  private final RelativeEncoder climbEncoder;
  
  public ClimbSubsystem() {
    climbMotor = new SparkMax(ClimbConstants.kClimbMotorPort, MotorType.kBrushless);
    climbEncoder = climbMotor.getEncoder();
  }

  public double getClimbEncoder(){
    return climbEncoder.getPosition();
  }

  public void resetClimbEncoder(){
    climbEncoder.setPosition(0);
  }

  public void setSpeed(double speed){
    climbMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climb Encoder", climbEncoder.getPosition());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */

  private SparkMax driveMotor;
  private SparkMax turningMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder turningEncoder;

  private PIDController turningPIDController;

  //private  absoluteEncoder;

  private boolean absoluteEncoderReversed;

  private double absoluteEncoderOffsetRad;
  private int turningMotorId;
  private int absoluteEncoderId;

  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
      this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
      this.absoluteEncoderReversed = absoluteEncoderReversed;

      //this.absoluteEncoder = new 

      dr
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

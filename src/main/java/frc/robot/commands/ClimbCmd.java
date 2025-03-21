// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbCmd extends Command {
  /** Creates a new ClimbCmd. */
  private ClimbSubsystem climbSubsystem;
  private double speed;
  private double setPoint;
  private boolean isAbove;
  public ClimbCmd(ClimbSubsystem climbSubsystem, double speed , double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climbSubsystem = climbSubsystem;
    this.speed = speed;
    isAbove = setPoint < climbSubsystem.getClimbEncoder();

    addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbSubsystem.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
    if((isAbove && climbSubsystem.getClimbEncoder() <= setPoint)
    || (!isAbove && climbSubsystem.getClimbEncoder() >= setPoint)){
      return true;
    }
     */
    return false;
  }
}

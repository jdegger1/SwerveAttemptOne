// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbHoldCmd extends Command {
  /** Creates a new ClimbHoldCmd. */
  private final ClimbSubsystem climbSubsystem;
  private final double setPoint;  
  private final PIDController m_PidController;
  public ClimbHoldCmd(ClimbSubsystem climbSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climbSubsystem = climbSubsystem;

    setPoint = climbSubsystem.getClimbEncoder();
    m_PidController = new PIDController(ClimbConstants.kClimbPVal, ClimbConstants.kClimbIVal, ClimbConstants.kclimbDVal);
    m_PidController.setSetpoint(setPoint);
    addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double speed = m_PidController.calculate(climbSubsystem.getClimbEncoder());

    climbSubsystem.setSpeed(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

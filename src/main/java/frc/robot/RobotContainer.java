// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LadderConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.LadderJoystickCmd;
import frc.robot.commands.LadderMove;
import frc.robot.commands.LadderMoveAuto;
import frc.robot.commands.ResetLadderEncoder;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LadderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here
  // 
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final LadderSubsystem ladderSubsystem = new LadderSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final SendableChooser<Command> autoChooser;

  private final Joystick driverJoystickOne = new Joystick(OIConstants.kDriverControllerOnePort);
  private final Joystick driverJoystickTwo = new Joystick(OIConstants.kDriverControllerTwoPort);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
   

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
              swerveSubsystem,
              () -> -driverJoystickOne.getRawAxis(OIConstants.kDriverYAxis),
              () -> driverJoystickOne.getRawAxis(OIConstants.kDriverXAxis),
              () -> driverJoystickOne.getRawAxis(OIConstants.kDriverRotAxisXbox),
              () -> !driverJoystickOne.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
              () -> driverJoystickOne.getRawButton(OIConstants.kFineTurningButton)));
    
    ladderSubsystem.setDefaultCommand(new LadderJoystickCmd(ladderSubsystem, () -> driverJoystickTwo.getRawAxis(OIConstants.kDriverYAxis) ));
    
    //Creates all named commands for pathPlanner
    NamedCommands.registerCommand("LadderL1", new LadderMoveAuto(ladderSubsystem, LadderConstants.kLiftTroughSetPoint));
    NamedCommands.registerCommand("LadderRecieve", new LadderMoveAuto(ladderSubsystem, LadderConstants.kLiftRecieveSetPoint));
    NamedCommands.registerCommand("LadderL2", new LadderMoveAuto(ladderSubsystem, LadderConstants.kLiftLowSetPoint));
    NamedCommands.registerCommand("LadderL3", new LadderMoveAuto(ladderSubsystem, LadderConstants.kLiftMidSetPoint));
    NamedCommands.registerCommand("LadderL4", new LadderMoveAuto(ladderSubsystem, LadderConstants.kLiftHighSetPoint));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser",autoChooser);

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {

    //Ladder now on controller two with joystick to manually control height.
    //toggle on true to make the robot stay at a setpoint until another command is given.
    new JoystickButton(driverJoystickTwo, OIConstants.kLiftHighButton).toggleOnTrue(new LadderMove(ladderSubsystem, LadderConstants.kLiftHighSetPoint));
    new JoystickButton(driverJoystickTwo, OIConstants.kLiftMidButton).toggleOnTrue(new LadderMove(ladderSubsystem, LadderConstants.kLiftMidSetPoint));
    new JoystickButton(driverJoystickTwo, OIConstants.kLiftLowButton).toggleOnTrue(new LadderMove(ladderSubsystem, LadderConstants.kLiftLowSetPoint));
    new JoystickButton(driverJoystickTwo, OIConstants.kliftTroughButton).toggleOnTrue(new LadderMove(ladderSubsystem, LadderConstants.kLiftTroughSetPoint));
    new JoystickButton(driverJoystickTwo, OIConstants.kLiftRecieveButton).toggleOnTrue(new LadderMove(ladderSubsystem, LadderConstants.kLiftRecieveSetPoint));

    new JoystickButton(driverJoystickTwo, OIConstants.kLiftResetEncoderButton).whileTrue(new ResetLadderEncoder(ladderSubsystem));

    //these two button move the ladder without a setPoint
    //new JoystickButton(driverJoystickTwo, OIConstants.kliftSpeedUpButton).whileTrue(new LadderShift(ladderSubsystem, LadderConstants.kLiftSpeedUp));
    //new JoystickButton(driverJoystickTwo, OIConstants.kliftSpeedDownButton).whileTrue(new LadderShift(ladderSubsystem, LadderConstants.kliftSpeedDown));

    new JoystickButton(driverJoystickOne, OIConstants.kIntakeInButton).whileTrue(new SpinIntake(intakeSubsystem, IntakeConstants.kIntakeSpeed));
    new JoystickButton(driverJoystickOne, OIConstants.kIntakeOutButton).whileTrue(new SpinIntake(intakeSubsystem, -IntakeConstants.kIntakeSpeed));  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
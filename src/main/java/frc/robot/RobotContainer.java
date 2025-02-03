// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.LadderConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LadderMove;
import frc.robot.commands.LadderShift;
import frc.robot.commands.ResetLadderEncoder;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.oldLadderCommands.LadderHigh;
import frc.robot.commands.oldLadderCommands.LadderLow;
import frc.robot.commands.oldLadderCommands.LadderMid;
import frc.robot.commands.oldLadderCommands.LadderRecieve;
import frc.robot.commands.oldLadderCommands.LadderTrough;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LadderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final LadderSubsystem ladderSubsystem = new LadderSubsystem();
  private final SendableChooser<String> m_chooser;

  private final Joystick driverJoystickOne = new Joystick(OIConstants.kDriverControllerOnePort);
  private final Joystick driverJoystickTwo = new Joystick(OIConstants.kDriverControllerTwoPort);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_chooser = new SendableChooser<>();

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
              swerveSubsystem,
              () -> -driverJoystickOne.getRawAxis(OIConstants.kDriverYAxis),
              () -> driverJoystickOne.getRawAxis(OIConstants.kDriverXAxis),
              () -> driverJoystickOne.getRawAxis(OIConstants.kDriverRotAxisXbox),
              () -> !driverJoystickTwo.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
    
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    /*
    new JoystickButton(driverJoystickOne, OIConstants.kLiftHighButton).whileTrue(new LadderHigh(ladderSubsystem, LadderConstants.kLiftHighSetPoint));
    new JoystickButton(driverJoystickOne, OIConstants.kLiftMidButton).whileTrue(new LadderMid(ladderSubsystem, LadderConstants.kLiftMidSetPoint));
    new JoystickButton(driverJoystickOne, OIConstants.kLiftLowButton).whileTrue(new LadderLow(ladderSubsystem, LadderConstants.kLiftLowSetPoint));
    new JoystickButton(driverJoystickOne, OIConstants.kliftTroughButton).whileTrue(new LadderTrough(ladderSubsystem, LadderConstants.kLiftTroughSetPoint));
    new JoystickButton(driverJoystickOne, OIConstants.kLiftRecieveButton).whileTrue(new LadderRecieve(ladderSubsystem, LadderConstants.kLiftRecieveSetPoint));
    */

    //Ladder is on controller one for testing, will move to controller two for final controls.
    new JoystickButton(driverJoystickOne, OIConstants.kLiftHighButton).whileTrue(new LadderMove(ladderSubsystem, LadderConstants.kLiftHighSetPoint));
    new JoystickButton(driverJoystickOne, OIConstants.kLiftMidButton).whileTrue(new LadderMove(ladderSubsystem, LadderConstants.kLiftMidSetPoint));
    new JoystickButton(driverJoystickOne, OIConstants.kLiftLowButton).whileTrue(new LadderMove(ladderSubsystem, LadderConstants.kLiftLowSetPoint));
    new JoystickButton(driverJoystickOne, OIConstants.kliftTroughButton).whileTrue(new LadderMove(ladderSubsystem, LadderConstants.kLiftTroughSetPoint));
    new JoystickButton(driverJoystickOne, OIConstants.kLiftRecieveButton).whileTrue(new LadderMove(ladderSubsystem, LadderConstants.kLiftRecieveSetPoint));

    new JoystickButton(driverJoystickOne, OIConstants.kLiftResetEncoderButton).onTrue(new ResetLadderEncoder(ladderSubsystem));

    //these two button move the ladder without a setPoint
    new JoystickButton(driverJoystickTwo, OIConstants.kliftSpeedUpButton).whileTrue(new LadderShift(ladderSubsystem, LadderConstants.kLiftSpeedUp));
    new JoystickButton(driverJoystickTwo, OIConstants.kliftSpeedDownButton).whileTrue(new LadderShift(ladderSubsystem, LadderConstants.kliftSpeedDown));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An example command will be run in autonomous
  //  return Autos.exampleAuto(m_exampleSubsystem);
  //}
}
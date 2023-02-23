// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutonomousRightOfStation1BallPad;
import frc.robot.commands.AutonomousRoutine1;
import frc.robot.commands.DrivebaseCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.GrabberCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivebaseSubsystem drivebaseSubsystem = new DrivebaseSubsystem();
  private final DrivebaseCommand drivebasecommand = new DrivebaseCommand(drivebaseSubsystem);

  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final LimelightCommand limelightcommand = new LimelightCommand(limelightSubsystem);

  private final GrabberSubsystem grabberSubsystem = new GrabberSubsystem();
  private final GrabberCommand grabbercommand = new GrabberCommand(grabberSubsystem);

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ElevatorCommand elevatorcommand = new ElevatorCommand(elevatorSubsystem);

  private final AutonomousRoutine1 routine1 = new AutonomousRoutine1(drivebaseSubsystem);
  private final AutonomousRightOfStation1BallPad autonomousrightofstation1ballpad = new AutonomousRightOfStation1BallPad(drivebaseSubsystem);

  public static XboxController xboxcontroller = new XboxController(Constants.xboxcontroller);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_chooser.setDefaultOption("Test Routine/Routine 1", routine1);
    m_chooser.addOption("Autonomous Right Of Station 1 Ball + Station", autonomousrightofstation1ballpad);
    SmartDashboard.putData("Autonomous Routines", m_chooser);
    // Configure the trigger bindings
    configureBindings();
    drivebaseSubsystem.setDefaultCommand(drivebasecommand);
    grabberSubsystem.setDefaultCommand(grabbercommand);
    elevatorSubsystem.setDefaultCommand(elevatorcommand);
  }

  private void configureBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();    
  }
}

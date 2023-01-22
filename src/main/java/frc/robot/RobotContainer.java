// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DrivebaseCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.GrabberCommand;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj.XboxController;
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
  private final DrivebaseSubsystem drivebasesub = new DrivebaseSubsystem();
  private final DrivebaseCommand drivebasecommand = new DrivebaseCommand(drivebasesub);

  private final GrabberSubsystem grabbersubsystem = new GrabberSubsystem();
  private final GrabberCommand grabbercommand = new GrabberCommand(grabbersubsystem);

  private final ElevatorSubsystem elevatorsubsystem = new ElevatorSubsystem();
  private final ElevatorCommand elevatorcommand = new ElevatorCommand(elevatorsubsystem);

  public static XboxController xboxcontroller = new XboxController(Constants.xboxcontroller);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drivebasesub.setDefaultCommand(drivebasecommand);
    grabbersubsystem.setDefaultCommand(grabbercommand);
    elevatorsubsystem.setDefaultCommand(elevatorcommand);
  }

  private void configureBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
    // An example command will be run in autonomous
    
  }
}

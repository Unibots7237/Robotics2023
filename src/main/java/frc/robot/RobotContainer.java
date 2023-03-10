// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutonomousCenterStationOneBall;
import frc.robot.commands.AutonomousLeftOfStation;
import frc.robot.commands.AutonomousRightOfStation;
import frc.robot.commands.AutonomousNoStationOneBall;
import frc.robot.commands.AutonomousTest;
import frc.robot.commands.DrivebaseCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.GrabberCommand;
import frc.robot.commands.LimelightCommand;
import frc.robot.subsystems.ArmSubsystem;
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
  private final ArmSubsystem armsubsystem = new ArmSubsystem();

  // The robot's subsystems and commands are defined here...
  public final DrivebaseSubsystem drivebasesubsystem = new DrivebaseSubsystem();
  private final DrivebaseCommand drivebasecommand = new DrivebaseCommand(drivebasesubsystem);

  private final GrabberSubsystem grabbersubsystem = new GrabberSubsystem();
  private final GrabberCommand grabbercommand = new GrabberCommand(grabbersubsystem, armsubsystem);

  private final ElevatorSubsystem elevatorsubsystem = new ElevatorSubsystem();

  private final ElevatorCommand elevatorcommand = new ElevatorCommand(elevatorsubsystem, armsubsystem);
  public final ArmCommand armcommand = new ArmCommand(armsubsystem, elevatorsubsystem);

  private final LimelightSubsystem limelightsubsystem = new LimelightSubsystem();
  private final LimelightCommand limelightcommand = new LimelightCommand(drivebasesubsystem, grabbersubsystem, elevatorsubsystem, armsubsystem, limelightsubsystem);

  private final AutonomousTest autonomoustest = new AutonomousTest(drivebasesubsystem, grabbersubsystem, elevatorsubsystem, armsubsystem, limelightsubsystem);
  private final AutonomousRightOfStation autonomousrightofstation = new AutonomousRightOfStation(drivebasesubsystem, grabbersubsystem, elevatorsubsystem, armsubsystem, limelightsubsystem);
  private final AutonomousLeftOfStation autonomousleftofstation = new AutonomousLeftOfStation(drivebasesubsystem, grabbersubsystem, elevatorsubsystem, armsubsystem, limelightsubsystem);
  private final AutonomousNoStationOneBall autonomousnostationoneball = new AutonomousNoStationOneBall(drivebasesubsystem, grabbersubsystem, elevatorsubsystem, armsubsystem, limelightsubsystem);
  private final AutonomousCenterStationOneBall autonomouscenterstationoneball = new AutonomousCenterStationOneBall(drivebasesubsystem, grabbersubsystem, elevatorsubsystem, armsubsystem, limelightsubsystem);
  
  public static XboxController xboxcontroller = new XboxController(Constants.xboxcontroller);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_chooser.setDefaultOption("Test Routine", autonomoustest);

    m_chooser.addOption("Autonomous Right Of Station 1 Ball", autonomousrightofstation);
    m_chooser.addOption("Autonomous Left Of Station 1 Ball", autonomousleftofstation);
    m_chooser.addOption("Autonomous Center Station 1 Ball", autonomouscenterstationoneball);
    m_chooser.addOption("Autonomous One Ball No Station", autonomousnostationoneball);

    SmartDashboard.putData("Autonomous Routines", m_chooser);
    // Configure the trigger bindings
    configureBindings();
    drivebasesubsystem.setDefaultCommand(drivebasecommand);
    grabbersubsystem.setDefaultCommand(grabbercommand);
    elevatorsubsystem.setDefaultCommand(elevatorcommand);
    armsubsystem.setDefaultCommand(armcommand);
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

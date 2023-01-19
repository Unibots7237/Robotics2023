// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DrivebaseCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivebaseSubsystem drivebasesub;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DrivebaseCommand(DrivebaseSubsystem subsystem) {
    drivebasesub = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double move2 = Robot.m_robotContainer.xboxcontroller.getLeftY();
        double turn = Robot.m_robotContainer.xboxcontroller.getLeftX();

        double accelerate = Robot.m_robotContainer.xboxcontroller.getRightTriggerAxis();
        double deaccelerate = Robot.m_robotContainer.xboxcontroller.getLeftTriggerAxis();
        double move = 0;

        if (accelerate > 0.0) {
            move = accelerate;
        }
        if (deaccelerate > 0.0) {
            move = -deaccelerate;
        }

        this.drivebasesub.teleopDrive(move, turn);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

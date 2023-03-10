// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivebaseSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivebaseCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private static DrivebaseSubsystem drivebasesub;

  static boolean autobalance;

  public DrivebaseCommand(DrivebaseSubsystem drivebasesubsystem) {
    drivebasesub = drivebasesubsystem;
    addRequirements(drivebasesub);
  }

  @Override
  public void execute() {

    double move2 = Robot.m_robotContainer.xboxcontroller.getLeftY();
    double turn = Robot.m_robotContainer.xboxcontroller.getLeftX();

    double accelerate = Robot.m_robotContainer.xboxcontroller.getRightTriggerAxis();
    double deaccelerate = Robot.m_robotContainer.xboxcontroller.getLeftTriggerAxis();
    double move = 0;


    if (accelerate > 0.05) {
        move = accelerate;
        }
    if (deaccelerate > 0.05) {
        move = -deaccelerate;
       }

        this.drivebasesub.teleopDrive(move, turn);

    if (RobotContainer.xboxcontroller.getYButton()) {
      autobalance = this.drivebasesub.autoBalanceOnStation();
    }
    if (autobalance) {
      this.drivebasesub.autoBalanceOnStation();
    }
  }
}

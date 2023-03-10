// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AutonomousTest extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DrivebaseSubsystem drivebasesub;
  private GrabberSubsystem grabsub;
  private ElevatorSubsystem elevatorsub;
  private ArmSubsystem armsub;
  private LimelightSubsystem limelightsub;
  
  private static Timer timer = new Timer();

  boolean doingFirstMove = true;
  boolean doingSecondMove = false;
  boolean doingThirdMove = false;

  int currentMove = 1;

  public AutonomousTest(DrivebaseSubsystem subsystem, GrabberSubsystem grabbersubsystem, ElevatorSubsystem elevatorsubsystem, ArmSubsystem armsubsystem, LimelightSubsystem limelightsubsystem) {
    drivebasesub = subsystem;
    grabsub = grabbersubsystem;
    elevatorsub = elevatorsubsystem;
    armsub = armsubsystem;
    limelightsub = limelightsubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, grabsub, armsub, elevatorsub, limelightsub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivebasesub.gyro.reset();
    drivebasesub.encoderLeft.setQuadraturePosition(0, 0);
    drivebasesub.encoderRight.setQuadraturePosition(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* 
    if (currentMove == 1) {
      doingFirstMove = elevatorsub.raiseToLevel(Constants.SecondElevatorHeight);
      if (!doingFirstMove) {
        currentMove++;
      }
    }
    if (currentMove == 2) {
      timer.start();
      armsub.currentArmTargetEncoder = Constants.thirdLevelArmHeight;
      currentMove ++;
    }
    if (currentMove == 3 && timer.hasElapsed(0.75)) {
      
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebasesub.gyro.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

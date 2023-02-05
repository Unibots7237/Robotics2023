// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.DrivebaseSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AutonomousRightOfStation1BallPad extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivebaseSubsystem drivebasesub;

  boolean doneWithFirstMove = false;
  boolean doneWithSecondMove = false;
  boolean doneWithThirdMove = false;
  boolean doneWithFourthMove = false;
  boolean doneWithFifthMove = false;
  boolean doneWithSixthMove = false;

  int currentMove = 1;
  int lastMove = 6;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonomousRightOfStation1BallPad(DrivebaseSubsystem subsystem) {
    drivebasesub = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
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

    //score the point
    //reverse 200 inches
    //turn 90 left (so 90 in the method)
    //move forward ~50 inches
    //turn 90 left
    //move forward ~50 inches
    //start auto gyro adjust

    if (currentMove == 1) {
      doneWithFirstMove = drivebasesub.DriveXInches(-2);
      if (doneWithFirstMove) {currentMove++;}
    }
    if (currentMove == 2) {
      doneWithSecondMove = drivebasesub.turnXdegrees(90);
      if (doneWithSecondMove) {currentMove++;}
    }
    if (currentMove == 3) {
      doneWithThirdMove = drivebasesub.DriveXInches(5);
      if (doneWithThirdMove) {currentMove++;}
    }
    if (currentMove == 4) {
      doneWithFourthMove = drivebasesub.turnXdegrees(90);
      if (doneWithFourthMove) {currentMove++;}
    }
    if (currentMove == 5) {
      doneWithFifthMove = drivebasesub.DriveXInches(8);
      if (doneWithFifthMove) {currentMove++;}
    }
    if (currentMove == 6) {
      doneWithSixthMove = drivebasesub.autoBalanceOnStation();
      if (doneWithSixthMove) {currentMove++;}
    }
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

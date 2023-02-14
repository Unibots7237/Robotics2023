// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ElevatorCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem m_subsystem;

  private final Timer timer = new Timer();
  private boolean debounce = false;
  private boolean currentlyMovingElevator = false;

  private boolean movingfirst = false;
  private boolean movingsecond = false;
  private boolean movingthird = false;

  
  public ElevatorCommand(ElevatorSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* 
    if (RobotContainer.xboxcontroller.getRightBumper()) {
      m_subsystem.spinNeoMotors(true);
    }
    if (RobotContainer.xboxcontroller.getLeftBumper()) {
      m_subsystem.spinNeoMotors(false);
    }*/

    //left stick button = third level
    //right stick button = second level
    //left bumper = first level

    if (timer.hasElapsed(1)) {
      timer.stop();
      timer.reset();
      debounce = false;
    }

    if (movingthird) {
      movingthird = this.m_subsystem.raiseToLevel(Constants.ThirdElevatorHeight);
    }
    if (movingsecond) {
      movingsecond = this.m_subsystem.raiseToLevel(Constants.SecondElevatorHeight);
    }
    if (movingfirst) {
      movingfirst = this.m_subsystem.raiseToLevel(Constants.FirstElevatorHeight);
    }

    if (!debounce) {
      if (RobotContainer.xboxcontroller.getLeftStickButton()) {
        movingthird = true;
        debounce = true; 
        timer.start();
      } else if(RobotContainer.xboxcontroller.getRightStickButton()){
        movingsecond = true;
        debounce = true;
        timer.start();
      } else if (RobotContainer.xboxcontroller.getLeftBumper()) {
        movingfirst = true;
        debounce = true;
        timer.start();
      }
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

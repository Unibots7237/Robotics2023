// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     
    if (!debounce) {
      if (RobotContainer.xboxcontroller.getRightBumper()) {
        m_subsystem.spinNeoMotors(true);
        if (-(m_subsystem.rightneomotor.getEncoder().getPosition()) >= Constants.maxElevatorHeight) {
          m_subsystem.neos.stopMotor();
        }
      } else if(RobotContainer.xboxcontroller.getLeftBumper()) {
        m_subsystem.spinNeoMotors(false);
      } else{
        m_subsystem.neos.set(0);
      }
    }

    //dpad right = third level
    //dpad up  = second level
    //dpad left = first level

    if (!debounce) {
      movingthird = false;
      movingsecond = false;
      movingfirst = false;
    }

    if (debounce) {

      if (movingthird) {
        movingthird = this.m_subsystem.raiseToLevel(Constants.ThirdElevatorHeight);
        if (!movingthird) {
          debounce = false;
          m_subsystem.neos.stopMotor();
        }
      }
      if (movingsecond) {
        movingsecond = this.m_subsystem.raiseToLevel(Constants.SecondElevatorHeight);
        if (!movingsecond) {
          debounce = false;
          m_subsystem.neos.stopMotor();
        }
      }
      if (movingfirst) {
        movingfirst = this.m_subsystem.raiseToLevel(Constants.FirstElevatorHeight);
        if (!movingfirst) {
          debounce = false;
          m_subsystem.neos.stopMotor();
        }
      }
    }

    if (!debounce) {
      if (RobotContainer.xboxcontroller.getPOV() == Constants.DPADRight) {
        movingthird = true;
        debounce = true; 
      } else if(RobotContainer.xboxcontroller.getPOV() == Constants.DPADUp){
        movingsecond = true;
        debounce = true;
      } else if (RobotContainer.xboxcontroller.getPOV() == Constants.DPADLeft) {
        movingfirst = true;
        debounce = true;
      }
    }

    if (RobotContainer.xboxcontroller.getPOV() == 180) {
      m_subsystem.rightneomotor.getEncoder().setPosition(0);
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

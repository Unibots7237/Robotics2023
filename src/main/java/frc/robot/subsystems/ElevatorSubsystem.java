// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  

  //have to manually lower to level 1 (lowest elevator height)
  public int currentElevatorLevel = 1;

  public CANSparkMax leftneomotor = new CANSparkMax(Constants.leftneomotor, MotorType.kBrushless);
  public CANSparkMax rightneomotor = new CANSparkMax(Constants.rightneomotor, MotorType.kBrushless);

  public MotorControllerGroup neos = new MotorControllerGroup(leftneomotor, rightneomotor);

  public ElevatorSubsystem() {
    rightneomotor.getEncoder().setPosition(Constants.startingElevatorHeight);
  }

  public void spinNeoMotors(boolean up) {
    SmartDashboard.putNumber("right neo encoder", -rightneomotor.getEncoder().getPosition());
    if (up) {
      if (-rightneomotor.getEncoder().getPosition() <= Constants.maxElevatorHeight) {
        neos.set(-Constants.elevatormotorspeed);
      }
    } else{
      neos.set(Constants.elevatormotorspeed);
    } 
  }

  public boolean raiseToLevel(double height) {
    SmartDashboard.putNumber("right neo encoder", -rightneomotor.getEncoder().getPosition());
    double neoEncoder = -rightneomotor.getEncoder().getPosition();
    boolean goingDown = false;

    if (height == Constants.FirstElevatorHeight) {
      currentElevatorLevel = 1;
    }
    if (height == Constants.SecondElevatorHeight) {
      currentElevatorLevel = 2;
    }
    if (height == Constants.ThirdElevatorHeight) {
      currentElevatorLevel = 3;
    }

    if (height < neoEncoder) {
      goingDown = true;
    }

    if (goingDown) {
      if (neoEncoder > height) {
        neos.set(Constants.elevatormotorspeed);
      }
      if (height == 0) {
        if (neoEncoder <= 0.5) {
          neos.set(0);
          return false;
        }
      }
      if (neoEncoder <= height+0.5) {
        neos.set(0);
        return false;
      }
    }
    if (!goingDown) {
      if (neoEncoder < height) {
        neos.set(-Constants.elevatormotorspeed);
      }
      if (neoEncoder >= height-0.5) {
        neos.set(0);
        return false;
      }
    }
    return true;
  }

  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

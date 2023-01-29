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


public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem() {}

  public CANSparkMax leftneomotor = new CANSparkMax(Constants.leftneomotor, MotorType.kBrushless);
  public CANSparkMax rightneomotor = new CANSparkMax(Constants.rightneomotor, MotorType.kBrushless);

  public MotorControllerGroup neos = new MotorControllerGroup(leftneomotor, rightneomotor);

  public void spinNeoMotors(boolean up) {
    if (up) {
      neos.set(Constants.elevatormotorspeed);
    } else{
      neos.set(-Constants.elevatormotorspeed);
    }
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

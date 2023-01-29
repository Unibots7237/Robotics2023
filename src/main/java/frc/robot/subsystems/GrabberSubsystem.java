// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class GrabberSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public GrabberSubsystem() {}

  DoubleSolenoid grabbersolenoids = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
   Constants.solenoidforward, Constants.solanoidreverse);

  //Solenoid grabbersolenoids = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  public void ExtendGrabbers() {
    grabbersolenoids.set(Value.kForward);
    //grabbersolenoids.set(true);
  }

  public void RetractGrabbers() {
    grabbersolenoids.set(Value.kReverse);
    //grabbersolenoids.set(false);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
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

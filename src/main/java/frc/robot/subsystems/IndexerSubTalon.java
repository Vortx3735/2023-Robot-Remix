// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubTalon extends SubsystemBase {
  TalonSRX indexTalon;
  /** Creates a new ExampleSubsystem. */
  public IndexerSubTalon(int ID) {
    indexTalon = new TalonSRX(ID);
  }

  public void move(double percentSpeed){
    indexTalon.set(TalonSRXControlMode.PercentOutput, percentSpeed);
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

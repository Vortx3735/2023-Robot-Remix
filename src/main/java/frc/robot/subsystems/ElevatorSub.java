// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSub extends SubsystemBase {
  static CANSparkMax ElevatorNeo1;
  static CANSparkMax ElevatorNeo2;
  private PIDController hold;
  private int setpoint;
  private SendableChooser<Boolean> softLimitOn = new SendableChooser<>();
  private SendableChooser<Boolean> resetElevator = new SendableChooser<>();

  /** Creates a new ExampleSubsystem. */
  public ElevatorSub(int ID1, int ID2) {
    ElevatorNeo1 = new CANSparkMax(ID1, MotorType.kBrushless);
    ElevatorNeo2 = new CANSparkMax(ID2, MotorType.kBrushless);
    ElevatorNeo1.setIdleMode(IdleMode.kBrake);
    ElevatorNeo2.setIdleMode(IdleMode.kBrake);
    ElevatorNeo2.follow(ElevatorNeo1, true);

    

    // ElevatorNeo2.enableSoftLimit(SoftLimitDirection.kForward, true);
    // ElevatorNeo2.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // ElevatorNeo1.enableSoftLimit(SoftLimitDirection.kForward, true);
    // ElevatorNeo1.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // ElevatorNeo1.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)113);
    // ElevatorNeo1.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)3);
    // ElevatorNeo2.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)113);
    // ElevatorNeo2.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)3);
    hold = new PIDController(0.01, 0, 0);
    setpoint = 0;

    softLimitOn.setDefaultOption("on", true);
    softLimitOn.addOption("off", false);

    resetElevator.setDefaultOption("no", false);
    resetElevator.addOption("yes", true);



    
  }

  public void move(double percentSpeed){
    ElevatorNeo1.set(percentSpeed);
  }

  public void hold() {
    double pos = ElevatorNeo1.getEncoder().getPosition();
    ElevatorNeo1.set(hold.calculate(pos, setpoint));

    setpoint = (int)(pos);
  }

  public void setRotations(double numRotations) {
    ElevatorNeo1.getEncoder().setPosition(numRotations);
    ElevatorNeo1.getEncoder().setPosition(numRotations);
  }


  @Override
  public void periodic() {
    // ElevatorNeo2.enableSoftLimit(SoftLimitDirection.kForward, softLimitOn.getSelected());
    // ElevatorNeo2.enableSoftLimit(SoftLimitDirection.kReverse, softLimitOn.getSelected());
    // ElevatorNeo1.enableSoftLimit(SoftLimitDirection.kForward, softLimitOn.getSelected());
    // ElevatorNeo1.enableSoftLimit(SoftLimitDirection.kReverse, softLimitOn.getSelected());

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("elevator rotations", ElevatorNeo1.getEncoder().getPosition());
    SmartDashboard.putData("elevator soft limit", softLimitOn);
    SmartDashboard.putData("reset elevator", resetElevator);

    if ( resetElevator.getSelected() ) {
      setRotations(0);
    }
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

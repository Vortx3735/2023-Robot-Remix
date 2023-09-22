// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSub;
import edu.wpi.first.wpilibj.DigitalInput;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;



/** An example command that uses an example subsystem. */
public class ElevatorCom extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSub elevate;
  private DigitalInput limitSwitchTop = new DigitalInput(0);
  private DigitalInput limitSwitchBottom = new DigitalInput(1);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorCom(ElevatorSub inputElevate) {
    elevate = inputElevate;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevate);
  }

  public void startMotor() {
    if(limitSwitchTop.get()) {
      elevate.move(RobotContainer.con2.getRightTriggerAxis());
    } else {
      elevate.hold();
    }
  }

  public void stopMotor() {
    elevate.move(0);
  }

  public void reverseMotor() {
    if(limitSwitchBottom.get()) {
      elevate.move(RobotContainer.con2.getLeftTriggerAxis());
    } else {
      elevate.hold();
    }
  }

  public void moveTest() {
    if (MathUtil.applyDeadband(-RobotContainer.con2.getRightY(), 0.1) < 0 && !limitSwitchBottom.get())
    {
      elevate.move(0);
    } 
    else if (MathUtil.applyDeadband(-RobotContainer.con2.getRightY(), 0.1) > 0 && !limitSwitchTop.get())
    {
      elevate.hold();
    } 
    else if (MathUtil.applyDeadband(-RobotContainer.con2.getRightY(), 0.1) > 0)
    {
      if (RobotContainer.clawsub.phClawDoubleSolenoid.get() == kReverse) 
      {
        elevate.move(MathUtil.applyDeadband(-RobotContainer.con2.getRightY()*0.7, 0.1));
      } 
      else
      {
        elevate.hold();
      }
    } 
    else if (MathUtil.applyDeadband(-RobotContainer.con2.getRightY(), 0.1) == 0)
    {
      elevate.hold();
    } 
    else
    {
      elevate.move(MathUtil.applyDeadband(-RobotContainer.con2.getRightY()*0.5, 0.1));
    }
  }

  public void moveMotor(double speed) {
    elevate.move(speed);
  }

  public void hold() {
    elevate.hold();
  }

  // public void goTop() {
  //   new ParallelRaceGroup(
  //     new RunCommand(
  //       moveMotor(.7),
  //       this)
  //   )
  // }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubTalon;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;


/** An example command that uses an example subsystem. */
public class IntakeComTalon extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubTalon intake;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeComTalon(IntakeSubTalon inputIntake) {
    intake = inputIntake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  public void startIntake() {
    intake.move(-0.8);
  }

  public void stopIntake() {
    intake.move(0);
  }

  public void rev() {
    intake.move(0.8);
  }

  public void push() {
    if (intake.phIntakeDoubleSolenoid.get() == kForward)
    {
      if(RobotContainer.clawsub.phClawDoubleSolenoid.get() == kReverse)
      {
        intake.closeIntake();
      }
    } else {
      intake.openIntake();
    }
    // intake.toggleIntake();
  }

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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.subsystems.IndexerSubTalon;
import edu.wpi.first.wpilibj2.command.CommandBase;



/** An example command that uses an example subsystem. */
public class IndexerComTalon extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IndexerSubTalon indexer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IndexerComTalon(IndexerSubTalon inputIndex) {
    indexer = inputIndex;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
  }

  public void start() {
    indexer.move(-0.65);
  }

  public void stop() {
    indexer.move(0);
  }

  public void rev() {
    indexer.move(0.65);
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

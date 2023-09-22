package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.torquelib.swerve.TorqueSwerveModule2022.SwerveConfig;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public final class Auton {
  private static HashMap<String, Command> eventMap;
  private static SwerveAutoBuilder autoBuilder;

  public static PathPlannerTrajectory straightPath;
  
  public static List<PathPlannerTrajectory> straightPathGroup; 
  public static Command straightPathCommand;
  public static DriveSubsystem swerveDrive;

  public static void init() {
    eventMap = buildEventMap();
    RobotContainer.swerve.zeroGyroscope();
    swerveDrive = RobotContainer.swerve;
    final SwerveConfig config = SwerveConfig.defaultConfig;


    straightPathGroup = PathPlanner.loadPathGroup("Straight", new PathConstraints(2, 1));
    
    autoBuilder = new SwerveAutoBuilder(
        swerveDrive::getPose, 
        swerveDrive::resetPose, 
        new PIDConstants(config.drivePGain, config.driveIGain, config.driveDGain), 
        new PIDConstants(config.turnPGain, config.turnIGain, config.turnDGain), 
        swerveDrive::autonDrive, 
        eventMap,
        swerveDrive
    );

    eventMap.put("Pick Cube", pickCube());
    eventMap.put("Score Cube", scoreCube());
    straightPathCommand = autoBuilder.fullAuto(straightPathGroup);
  }

    public static Command pickCube() {
        return new ParallelRaceGroup(
        new ParallelCommandGroup(
            new RunCommand(
                RobotContainer.intake::startIntake,
                RobotContainer.intakesub
            ),
            new RunCommand(
                RobotContainer.indexer::start,
                RobotContainer.indexersub
            )
        ),
        new WaitCommand(2)
        );
    }

  public static Command scoreCube() {
    return new ParallelRaceGroup(
        new ParallelCommandGroup(
          new RunCommand(
            RobotContainer.intake::rev,
            RobotContainer.intakesub
          ),
          new RunCommand(
            RobotContainer.indexer::rev,
            RobotContainer.indexersub
          )
        ),
        new WaitCommand(2)
    );
  }

  public static Command straightPath() {
    return straightPathCommand;
  }

  public static CommandBase none() {
    return Commands.none();
  }

  private static HashMap<String, Command> buildEventMap() {
    return new HashMap<>(
        Map.ofEntries(
            Map.entry("event1", Commands.print("event1")),
            Map.entry("event2", Commands.print("event2"))));
  }
}
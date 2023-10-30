// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.Constants.OIConstants;
import static frc.robot.subsystems.DriveSubsystem.speedScale;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Compressor;

// ask ethan what this is
// import javax.print.attribute.standard.JobHoldUntil;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    
    public static VorTXControllerXbox con1 = new VorTXControllerXbox(0);
    //  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    
    public static VorTXControllerXbox con2 = new VorTXControllerXbox(1);

    public static IntakeSubTalon intakesub = new IntakeSubTalon(13);
    public static IntakeComTalon intake = new IntakeComTalon(intakesub);
    
    
    // public static IntakeSubTalon intakesub = new IntakeSubTalon(25);
    // public static IntakeComTalon intake = new IntakeComTalon(intakesub);
    
    public static IndexerSubTalon indexersub = new IndexerSubTalon(26);
    public static IndexerComTalon indexer = new IndexerComTalon(indexersub);
    

    public static ClawSubTalon clawsub = new ClawSubTalon();
    public static ClawComTalon claw = new ClawComTalon(clawsub);

    public static ElevatorSub elevatorsub = new ElevatorSub(1, 2);
    public static ElevatorCom elevator = new ElevatorCom(elevatorsub);

    // public static PhotonSub limelight = new PhotonSub("ur mother");
    //public static PigeonWrapper gyro = new PigeonWrapper(0);

    public static Compressor phCompressor = new Compressor(11, PneumaticsModuleType.CTREPCM);

    // public static Auto test = new Auto("Straight");
    public static DriveSubsystem swerve = new DriveSubsystem();
    public static AutoBalance balance = new AutoBalance();
    
    private static final SendableChooser<Command> autoChooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        phCompressor.enableDigital();
        //phCompressor.disable();


        Auton.init(); 

        autoChooser.setDefaultOption("Straight", Auton.straightPath());
        autoChooser.addOption("None", Auton.none());


        SmartDashboard.putData("Autonomous Mode", autoChooser);

        // Configure the button bindings
        configureButtonBindings();

        indexersub.setDefaultCommand(
            new RunCommand(
                indexer::stop,
                indexersub
            ));

        intakesub.setDefaultCommand(
            new RunCommand(
                intake::stopIntake,
                intakesub
            )
        );
        
        elevatorsub.setDefaultCommand(
            new RunCommand(
                elevator::moveTest,
                elevatorsub
            )
        );

        swerve.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                () -> swerve.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        //1,0,0,
                        MathUtil.applyDeadband(con1.getLeftY(), 0.1)*speedScale,
                        MathUtil.applyDeadband(con1.getLeftX(), 0.1)*speedScale,
                        MathUtil.applyDeadband(con1.getRightX(), 0.1)*1.75*speedScale, 
                        DriveSubsystem.getGyroscopeRotation())
                    ),
                    swerve
            )
        );

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {


        // R1 changes speed to as fast as possible
        // L1 changes speed to really slow
        // if speed is set to anything > 4, motor controllers set to brake 
        con1.rb.onTrue(
            new InstantCommand(
                () -> {
                    swerve.changeSpeed(10);
                },
                swerve
            )
        );
        con1.lb.onTrue(
            new InstantCommand(
                () -> {
                    swerve.changeSpeed(1.5);
                },
                swerve
            )
        );

        con1.menu.onTrue(
           new InstantCommand(
                swerve::zeroGyroscope,
                swerve
           ) 
        );

        // //  index and intake
        con2.lb.whileTrue(
            new ParallelCommandGroup(
                new RunCommand(
                    indexer::start,
                    indexersub
                )
                ,
                new RunCommand(
                    intake::startIntake,
                    intakesub
                )
                // ,
                // new InstantCommand(
                //     claw::intakeClaw,
                //     clawsub
                // )
            )
        );

        // // outtake
        con2.bButton.whileTrue(
            new ParallelCommandGroup(
                new RunCommand(
                    indexer::rev,
                    indexersub
                )
                ,
                new RunCommand(
                    intake::rev,
                    intakesub
                )
            )
        );


        // con2.lt.whileTrue(
        //     new RunCommand(
        //         elevator::reverseMotor,
        //         elevatorsub
        //     )
        // );

        // con2.rt.whileTrue(
        //     new RunCommand(
        //         elevator::startMotor,
        //         elevatorsub
        //     )
        // );


        con2.rb.onTrue(
            new InstantCommand(
                intake::push,
                intakesub
            )                
        );

        con2.aButton.onTrue(
            new InstantCommand(
                claw::grab,
                clawsub
            )
        );

    }

    

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        swerve.zeroGyroscope();
        // An ExampleCommand will run in autonomous
        
        // return test;
    
        // return autoChooser.getSelected();
        
        //return new RunCommand(
        //    () -> swerve.drive(
        //        new ChassisSpeeds(
        //            -balance.scoreAndBalance(), 
        //            0,
        //            0
        //        )
        //    ),
        //    swerve
        //);


        //secondary hardcoded auton
        // return new SequentialCommandGroup(
        //     new InstantCommand(
        //         intake::push,
        //         intakesub
        //     ),
        //     new ParallelRaceGroup(
        //         new ParallelCommandGroup(
        //             new RunCommand(
        //                 indexer::rev,
        //                 indexersub
        //             ),
        //             new RunCommand(
        //                 intake::rev,
        //                 intakesub
        //             )
        //         ),
        //         new WaitCommand(1)
        //     ),
        //     new ParallelRaceGroup(
        //         new RunCommand(
        //             () -> swerve.drive(
        //                 new ChassisSpeeds(
        //                     2,
        //                     0,
        //                     0
        //                 )
        //             ),
        //         swerve
        //         ),
        //         new WaitCommand(2.2)
        //     )
        // );

        //HARD-CODED AUTON, DON'T DELETE JUST COMMENT
        return new SequentialCommandGroup(
            new ParallelRaceGroup(
                new RunCommand(
                    () -> swerve.drive(
                        new ChassisSpeeds(
                            2,
                            0,
                            0
                        )
                    ),
                swerve
                ),
                new WaitCommand(0.35)
            ),
            new ParallelRaceGroup(
                new RunCommand(
                    () -> swerve.drive(
                        new ChassisSpeeds(
                            -1,
                            0,
                            0
                        )
                    ),
                swerve
                ),
                new WaitCommand(4.4)
            )
        );
    }
}
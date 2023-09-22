// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
//import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.torquelib.swerve.TorqueSwerveModule2022;
import frc.robot.subsystems.torquelib.swerve.TorqueSwerveModule2022.SwerveConfig;
import frc.robot.subsystems.torquelib.swerve.TorqueSwerveModule2022.SwervePorts;
//import frc.robot.RobotContainer;
import com.ctre.phoenix.sensors.Pigeon2Configuration;

import static frc.robot.Constants.*;

public class DriveSubsystem extends SubsystemBase {
    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is
     * useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;
    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
    // pi
    // By default this value is setup for a Mk3 standard module using Falcon500s to
    // drive.
    // An example of this constant for a Mk4 L2 module with NEOs to drive is:
    // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880 / 60.0 *
            SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
            SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;
    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.


    public static double speedScale;

    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

    public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

        

    //private final static AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
    public static Pigeon2 pigeon;
    public static Pigeon2Configuration pConfig;

    // These are our modules. We initialize them in the constructor.
    private final TorqueSwerveModule2022 m_frontLeftModule;
    private final TorqueSwerveModule2022 m_frontRightModule;
    private final TorqueSwerveModule2022 m_backLeftModule;
    private final TorqueSwerveModule2022 m_backRightModule;
    private final TorqueSwerveModule2022[] modules;
    
    private Pose2d m_pose;
    private SwerveDriveOdometry m_odometry;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public DriveSubsystem() {
        //ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        // There are 4 methods you can call to create your swerve modules.
        // The method you use depends on what motors you are using.
        //
        // Mk3SwerveModuleHelper.createFalcon500(...)
        // Your module has two Falcon 500s on it. One for steering and one for driving.
        //
        // Mk3SwerveModuleHelper.createNeo(...)
        // Your module has two NEOs on it. One for steering and one for driving.
        //
        // Mk3SwerveModuleHelper.createFalcon500Neo(...)
        // Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving
        // and the NEO is for steering.
        //
        // Mk3SwerveModuleHelper.createNeoFalcon500(...)
        // Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the
        // Falcon 500 is for steering.
        //
        // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper
        // class.

        
        // By default we will use Falcon 500s in standard configuration. But if you use
        // a different configuration or motors
        // you MUST change it. If you do not, your code will crash on startup.
        // FIXME Setup motor configuration


        SwervePorts FL_MOD = new SwervePorts(FRONT_LEFT_MODULE_DRIVE_MOTOR, FRONT_LEFT_MODULE_STEER_MOTOR, FRONT_LEFT_MODULE_STEER_ENCODER);
        SwervePorts FR_MOD = new SwervePorts(FRONT_RIGHT_MODULE_DRIVE_MOTOR, FRONT_RIGHT_MODULE_STEER_MOTOR, FRONT_RIGHT_MODULE_STEER_ENCODER);
        SwervePorts BL_MOD = new SwervePorts(BACK_LEFT_MODULE_DRIVE_MOTOR, BACK_LEFT_MODULE_STEER_MOTOR, BACK_LEFT_MODULE_STEER_ENCODER);
        SwervePorts BR_MOD = new SwervePorts(BACK_RIGHT_MODULE_DRIVE_MOTOR, BACK_RIGHT_MODULE_STEER_MOTOR, BACK_RIGHT_MODULE_STEER_ENCODER);


       final SwerveConfig config = SwerveConfig.defaultConfig;

       pigeon = new Pigeon2(18);
       pConfig = new Pigeon2Configuration();
       pConfig.MountPosePitch = MOUNT_PITCH;
       pConfig.MountPoseRoll = MOUNT_ROLL;
       pConfig.MountPoseYaw = MOUNT_YAW;
       pigeon.configAllSettings(pConfig);
       zeroGyroscope();

       config.maxVelocity = 4.5;
       config.maxAcceleration = 3;
       config.maxAngularVelocity = Math.PI;
       config.maxAngularAcceleration = Math.PI;

        m_frontLeftModule = new TorqueSwerveModule2022("frontLeft", FL_MOD, FRONT_LEFT_MODULE_STEER_OFFSET -1.174990368758747, config);

        // We will do the same for the other modules
        m_frontRightModule = new TorqueSwerveModule2022("frontRight", FR_MOD, FRONT_RIGHT_MODULE_STEER_OFFSET -2.647602763765622, config);

        m_backLeftModule = new TorqueSwerveModule2022("backLeft", BL_MOD, BACK_LEFT_MODULE_STEER_OFFSET + 1.029346505702506, config);

        m_backRightModule = new TorqueSwerveModule2022("backRight", BR_MOD, BACK_RIGHT_MODULE_STEER_OFFSET -2.391440058458112, config);
        speedScale = 10;

        modules = new TorqueSwerveModule2022[] {
            m_frontLeftModule,
            m_frontRightModule,
            m_backLeftModule,
            m_backRightModule
        };

        m_odometry = new SwerveDriveOdometry(
            m_kinematics, getGyroscopeRotation(),
            new SwerveModulePosition[] {
                m_frontLeftModule.getPosition(),
                m_frontRightModule.getPosition(),
                m_backLeftModule.getPosition(),
                m_backRightModule.getPosition()
            }, new Pose2d(5.0, 13.5, new Rotation2d()));

        for ( TorqueSwerveModule2022 module : modules ) {
            module.setIsBrake(true);
        }

    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        pigeon.setYaw(0);
    }

    public Pose2d getPose() {
        return m_pose;
    }

    public void resetPose(Pose2d pose) {
        pose = getPose();
        m_odometry.resetPosition(
            getGyroscopeRotation(), 
            new SwerveModulePosition[] {
                m_frontLeftModule.getPosition(),
                m_frontRightModule.getPosition(),
                m_backLeftModule.getPosition(),
                m_backRightModule.getPosition() 
            }, pose
        );        
    }

    // R1 changes speed to triple
    // RS changes speed to normal
    // LS changes speed to half
    public void changeSpeed(double speed) {
        speedScale = speed;
        boolean isBrakeMode = speed < 3;
        for ( TorqueSwerveModule2022 module : modules ) {
            module.setIsBrake(isBrakeMode);
        }
    }

    /*
    * Method that returns adjusted yaw based off of drift we've observed
    * Usually drifs around 5 degrees per rotation, or 5 degrees per 360 degrees
    * Magic number ehehehehehee
    * @return The yaw adjusted bu our observed drift
    */
    //byebye stage
    // public static double getAdjustedYaw() {
    //     double magic = 360.0/365;
    //     double raw_angle = m_navx.getAngle();
    //     double adjusted_angle = raw_angle * magic;
    //     double adjusted_orientation = adjusted_angle % 360;
    //     if(adjusted_orientation > 180) {
    //         adjusted_orientation -= 360;
    //     }
    //     return adjusted_orientation;
    // }
    
    public static Rotation2d getGyroscopeRotation() {
        // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 + pigeon.getYaw());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    public void autonDrive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            chassisSpeeds.vyMetersPerSecond*0.1,
            chassisSpeeds.vxMetersPerSecond*0.1,
            chassisSpeeds.omegaRadiansPerSecond*0.5,
            DriveSubsystem.getGyroscopeRotation());
    }

    @Override
    public void periodic() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        m_pose = m_odometry.update(getGyroscopeRotation(),
            new SwerveModulePosition[] {
                m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(),
                m_backLeftModule.getPosition(), m_backRightModule.getPosition()
            }
        );

        m_frontLeftModule.setDesiredState(states[0]);
        m_frontRightModule.setDesiredState(states[1]);
        m_backLeftModule.setDesiredState(states[2]);
        m_backRightModule.setDesiredState(states[3]);

        SmartDashboard.putNumber("Yaw", pigeon.getYaw());
        //SmartDashboard.putNumber("Angle", m_navx.getAngle());
        // SmartDashboard.putNumber("DriftAdjusted", getAdjustedYaw());
        SmartDashboard.putNumber("Pitch", pigeon.getPitch());
        SmartDashboard.putNumber("Roll", pigeon.getRoll());

    }
}
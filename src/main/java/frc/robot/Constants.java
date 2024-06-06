// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.pid.FPID;
import org.frc5587.lib.subsystems.SwerveBase.SwerveConstants;
import org.frc5587.lib.subsystems.SwerveModuleBase.SwerveModuleConstants;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.util.swervelib.util.COTSFalconSwerveConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class ArmConstants {
    //motor info
   public static final int LEFT_MOTOR_ID = 20;
   public static final int RIGHT_MOTOR_ID = 21;
   public static final boolean LEFT_MOTOR_INVERTED = false;
   public static final boolean RIGHT_MOTOR_INVERTED = true;
    
    public static final double AMP_SETPOINT = Units.degreesToRadians(83);
    public static final double RESTING_SETPOINT = Units.degreesToRadians(0);
    public static final double TRAVEL_SETPOINT = Units.degreesToRadians(6);
    public static final double CLIMB_SETPOINT = Units.degreesToRadians(40);
    public static final double FERRY_SETPOINT = Units.degreesToRadians(25);
    
    public static final double GEARING_MOTOR_TO_ARM = 180.;
    public static final Rotation2d[] SOFT_LIMITS = {Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(87)};
    public static final Rotation2d ZERO_OFFSET = new Rotation2d();
    public static final int ENCODER_CPR = 1;
    public static final Constraints DEFAULT_CONSTRAINTS = new Constraints(Math.PI, Math.PI / 2);
    public static final Constraints CLIMB_CONSTRAINTS = new Constraints(Math.PI / 2, Math.PI / 4);
    public static final ProfiledPIDController PID = new ProfiledPIDController(20, 0.0, 0.01, DEFAULT_CONSTRAINTS);
    public static final ProfiledPIDController CLIMB_PID = new ProfiledPIDController(23.529, 0.0, 0.31304, DEFAULT_CONSTRAINTS);
    public static final ArmFeedforward FF = new ArmFeedforward(0.12522, 0.305, 0.10237, 0.006812);
    public static final int STALL_LIMIT = 40;
    public static final int FREE_LIMIT = 40;
    public static final double ARM_LENGTH_METERS = 0.525;
    public static final Rotation2d ANGLE_TO_SHOOTER = Rotation2d.fromDegrees(52.);
    public static final double SHOOTER_HEIGHT_METERS = 0.395;
  }
    public static final class DrivetrainConstants {
        public static final boolean INVERT_GYRO = true; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants CHOSEN_MODULE = 
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = 0.553;
        public static final double WHEEL_BASE = 0.553;
        public static final double WHEEL_CIRCUMFERENCE_METERS = CHOSEN_MODULE.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        public static final int DRIVE_ENCODER_CPR = 1;
        public static final int ANGLE_ENCODER_CPR = 1;
        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

        /* Motor Inverts */
        public static final boolean DRIVE_MOTOR_INVERTED = CHOSEN_MODULE.driveMotorInvert;
        public static final boolean ANGLE_MOTOR_INVERTED = CHOSEN_MODULE.angleMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean CANCODER_INVERTED = CHOSEN_MODULE.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int DRIVE_CONT_LIMIT = 35;
        public static final int DRIVE_PEAK_LIMIT = 40;
        public static final double DRIVE_PEAK_DURATION = 0.1;
        public static final boolean DRIVE_LIMIT_ENABLED = true;
        public static final boolean DRIVE_PEAK_LIMIT_ENABLED = true;

        public static final int ANGLE_CONT_LIMIT = 30;
        public static final int ANGLE_PEAK_LIMIT = 35;
        public static final double ANGLE_PEAK_DURATION = 0.1;
        public static final boolean ANGLE_LIMIT_ENABLED = true;
        public static final boolean ANGLE_PEAK_LIMIT_ENABLED = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double OPEN_LOOP_RAMP = 0.5;
        public static final double CLOSED_LOOP_RAMP = 0.08;

        /* Drive Motor PID Values */
        public static final FPID DRIVE_FPID = new FPID(
                0.0038, 0.0, 0.0, 0.0);

        /* Angle Motor PID Values */
        public static final FPID ANGLE_FPID = new FPID(
                CHOSEN_MODULE.angleKF, CHOSEN_MODULE.angleKP, CHOSEN_MODULE.angleKI, CHOSEN_MODULE.angleKD);

        /*
         * Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         */
        public static final double DRIVE_KS = 0.22087;
        public static final double DRIVE_KV = 5.599;
        public static final double DRIVE_KA = 0.25493;
        public static final SimpleMotorFeedforward DRIVE_FF = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 5;
        public static final double CRAWL_SPEED = 0.75;
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 3 * Math.PI;
        public static final double CRAWL_ANGULAR_VELOCITY = Math.PI / 3;

        /* Neutral Modes */
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_ID = 10;
            public static final int ANGLE_ID = 15;
            public static final int CANCODER_ID = 50;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(155.188+68.291);
            public static final boolean ENCODER_INVERTED = false;
            public static final SwerveModuleConstants MODULE_CONSTANTS = new SwerveModuleConstants(
                    0, WHEEL_CIRCUMFERENCE_METERS, MAX_SPEED, ANGLE_ENCODER_CPR, DRIVE_ENCODER_CPR, ANGLE_GEAR_RATIO,
                    DRIVE_GEAR_RATIO);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int DRIVE_ID = 11;
            public static final int ANGLE_ID = 16;
            public static final int CANCODER_ID = 51;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(-192.845);
            public static final boolean ENCODER_INVERTED = false;
            public static final SwerveModuleConstants MODULE_CONSTANTS = new SwerveModuleConstants(
                    1, WHEEL_CIRCUMFERENCE_METERS, MAX_SPEED, ANGLE_ENCODER_CPR, DRIVE_ENCODER_CPR, ANGLE_GEAR_RATIO,
                    DRIVE_GEAR_RATIO);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_ID = 12;
            public static final int ANGLE_ID = 17;
            public static final int CANCODER_ID = 52;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(-261.453);
            public static final boolean ENCODER_INVERTED = false;
            public static final SwerveModuleConstants MODULE_CONSTANTS = new SwerveModuleConstants(
                    2, WHEEL_CIRCUMFERENCE_METERS, MAX_SPEED, ANGLE_ENCODER_CPR, DRIVE_ENCODER_CPR, ANGLE_GEAR_RATIO,
                    DRIVE_GEAR_RATIO);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int DRIVE_ID = 13;
            public static final int ANGLE_ID = 18;
            public static final int CANCODER_ID = 53;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(53.287 - 127.792);
            public static final boolean ENCODER_INVERTED = false;
            public static final SwerveModuleConstants MODULE_CONSTANTS = new SwerveModuleConstants(
                    3, WHEEL_CIRCUMFERENCE_METERS, MAX_SPEED, ANGLE_ENCODER_CPR, DRIVE_ENCODER_CPR, ANGLE_GEAR_RATIO,
                    DRIVE_GEAR_RATIO);
        }
        
        public static final SwerveModuleConstants[] ALL_MODULE_CONSTANTS = {Mod0.MODULE_CONSTANTS, Mod1.MODULE_CONSTANTS, Mod2.MODULE_CONSTANTS, Mod3.MODULE_CONSTANTS};
        public static final SwerveConstants SWERVE_CONSTANTS = new SwerveConstants(ALL_MODULE_CONSTANTS, SWERVE_KINEMATICS, INVERT_GYRO, MAX_SPEED);
    }
  
  public static final class ClimbConstants {
    public static final int LEFT_MOTOR_ID = 50;
    public static final int RIGHT_MOTOR_ID = 51;
    
    public static final boolean RIGHT_MOTOR_INVERTED = false;
    public static final boolean LEFT_MOTOR_INVERTED = true;
    
    public static final int STALL_LIMIT = 40;
    public static final int FREE_LIMIT = 35;

    public static final double GEARING = 12.5;
    public static final double SPOOL_DIAMETER_METERS = Units.inchesToMeters(0.5); // spool is 1.645 in, hex is 0.5 in
    public static final double SPOOL_CIRCUMFERENCE_METERS = Math.PI * SPOOL_DIAMETER_METERS;
    public static final double TOP_POSITION = 0;
    public static final double BOTTOM_POSITION = Units.inchesToMeters(7.5);
    public static final double MIDDLE_POSITION = BOTTOM_POSITION / 2;
    
    public static final double[] SOFT_LIMITS = {0, Units.inchesToMeters(8.)};
    public static final TrapezoidProfile.Constraints CONSTRAINTS = new Constraints(0.2, 0.1);
    public static final ProfiledPIDController PID = new ProfiledPIDController(196.28, 0, 0, CONSTRAINTS);
  }

  public static final class IntakeConstants {
    public static final int MOTOR_ID = 40;
    public static final boolean MOTOR_INVERTED = true;

    public static final int STALL_LIMIT = 35;
    public static final int FREE_LIMIT = 25;

    public static final double FORWARD_THROTTLE = .75;
    public static final double REVERSE_THROTTLE = 0.4;
    public static final double WHEEL_RADIUS = 2.25/2;
    public static final double GEARING = 12.;
    public static final SimpleMotorFeedforward FF = new SimpleMotorFeedforward(0., 0.1, 0.);
    public static final PIDController PID = new PIDController(0.15, 0, 0.0);
    public static final double MINIMUM_VELOCITY = 30;
    public static final double SWERVE_VELOCITY_OFFSET = 1;
  }

  public static final class ShooterConstants {
    // motor ports
    public static final int LEFT_MOTOR_ID = 30;
    public static final int RIGHT_MOTOR_ID = 31;

    public static final boolean RIGHT_MOTOR_INVERTED = false;
    public static final boolean LEFT_MOTOR_INVERTED = true;
    // motor limits
    public static final int STALL_LIMIT = 30;
    public static final int FREE_LIMIT = 35;
    // motor speeds
    public static final double FORWARD_THROTTLE = 0.65;
    public static final double REVERSE_THROTTLE = .5;
    public static final double AMP_THROTTLE = 9;
    public static final double IDLE_SPEED = FORWARD_THROTTLE / 4;

    public static final double MAX_MOTOR_SPEED_RPS = 71.8;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    public static final TrapezoidProfile.Constraints CONSTRAINTS = new Constraints(80, 80);
    public static final ProfiledPIDController PID = new ProfiledPIDController(0.10737, 0, 0, CONSTRAINTS);
    public static final ProfiledPIDController LEFT_PID = new ProfiledPIDController(0.086865, 0, 0.0, CONSTRAINTS);
    public static final ProfiledPIDController RIGHT_PID = new ProfiledPIDController(0.060734, 0, 0.0, CONSTRAINTS);
    
    public static final SimpleMotorFeedforward FF = new SimpleMotorFeedforward(0.14804, 0.39004, 0.16621);
    public static final SimpleMotorFeedforward LEFT_FF = new SimpleMotorFeedforward(0.16486, 0.39292, 0.12948);
    public static final SimpleMotorFeedforward RIGHT_FF = new SimpleMotorFeedforward(0.16883, 0.39896, 0.15285);


    public static final double RadiansPerMeter = Units.degreesToRadians(3);
    
  }

  public static final class FieldConstants {
    public static final Translation3d BLUE_SPEAKER_OPENING_TRANSLATION = new Translation3d(0.01, 5.556, 3.267);
    public static final Pose2d BLUE_SUBWOOFER_FRONT_POSE = new Pose2d(1.35, 5.556, new Rotation2d());
    
    public static final Translation3d RED_SPEAKER_OPENING_TRANSLATION = new Translation3d(16.53, 5.556, 3.267);
    public static final Pose2d RED_SUBWOOFER_FRONT_POSE = new Pose2d(15.20, 5.556, Rotation2d.fromDegrees(180.));

    public static final Pose2d BLUE_AMP_POSE = new Pose2d(1.82, 7.66, Rotation2d.fromDegrees(180));
    public static final Pose2d RED_AMP_POSE = new Pose2d(14.70, 7.66, Rotation2d.fromDegrees(180));
  }

    public static final class LimelightConstants {
        public static final Rotation2d NOTE_DETECTOR_MOUNT_ANGLE = Rotation2d.fromDegrees(-22); // Relative to arm angle
        public static final double DISTANCE_TO_ARM_PIVOT = Units.inchesToMeters(29.407);
        public static final double INITIAL_LENS_HEIGHT = Units.inchesToMeters(9); 
        public static final double GOAL_HEIGHT = Units.inchesToMeters(0);
        public static final double DISTANCE_OFFSET = 0;
    }

    public static final class AutoConstants {
        public static final double MAX_SPEED_MPS = 4.5; // in m/s  
        public static final double MAX_ACCEL_MPS_2 = 3.5; // in m/s^2 
        public static final double MAX_ANGULAR_SPEED_R_S = Math.PI; // in radians/s 
        public static final double MAX_ANGULAR_ACCEL_R_S_2 = Math.PI; // in radians/s^2 

        public static final double ROTATION_KP = 1.25;
        public static final double ROTATION_KI = 0;
        public static final double ROTATION_KD = 0.05;

        public static final double TRANSLATION_KP = 4.5;
        public static final double TRANSLATION_KI = 0;
        public static final double TRANSLATION_KD = 0.0;

        public static final double DRIVE_BASE_RADIUS = 0.6095; // in m, middle to corner
        public static final PathConstraints CONSTRAINTS = new PathConstraints(MAX_SPEED_MPS, MAX_ACCEL_MPS_2, MAX_ANGULAR_SPEED_R_S, MAX_ANGULAR_ACCEL_R_S_2);
        public static final PathConstraints PATHFIND_CONSTRAINTS = new PathConstraints(0.85, 0.75, Math.PI / 4, Math.PI / 4);
        
    }
}

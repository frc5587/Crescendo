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
    
   //Values TBD, placeholders for now
    public static final double SPEAKER_SETPOINT = Units.degreesToRadians(40);
    public static final double AMP_SETPOINT = Units.degreesToRadians(83);
    public static final double RESTING_SETPOINT = Units.degreesToRadians(1);
    public static final double STAGE_SETPOINT = Units.degreesToRadians(89);
    
    public static final double GEARING_MOTOR_TO_ARM = 180.;
    public static final double GEARING_ARM_TO_THROUGHBORE = 16./64.;
    public static final double GEARING_THROUGHBORE_TO_MOTOR = 1. / (GEARING_MOTOR_TO_ARM * GEARING_ARM_TO_THROUGHBORE);
    public static final Rotation2d[] SOFT_LIMITS = {Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(87)};
    public static final Rotation2d ZERO_OFFSET = new Rotation2d();
    public static final Rotation2d THROUGHBORE_ZERO_OFFSET = Rotation2d.fromRotations(0.5); // TODO: Replace this placeholder
    public static final int ENCODER_CPR = 1;
    public static final Constraints DEFAULT_CONSTRAINTS = new Constraints(Math.PI, Math.PI);
    public static final ProfiledPIDController PID = new ProfiledPIDController(7., 0.0, 0.85, DEFAULT_CONSTRAINTS);
    public static final Constraints CLIMB_CONSTRAINTS = new Constraints(Math.PI / 2, Math.PI / 4);
    public static final ArmFeedforward FF = new ArmFeedforward(0.35, 0.25, 1.5, 0.);
    public static final int STALL_LIMIT = 35;
    public static final int FREE_LIMIT = 40;

  }
    public static final class DrivetrainConstants {
        public static final boolean INVERT_GYRO = true; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants CHOSEN_MODULE = 
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(27); // distance from left wheel to right wheel
        public static final double WHEEL_BASE = Units.inchesToMeters(27); // distance from front wheel to back wheel
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

        public static final double SLEW_RATE = 3; // m/s^2

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
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.25;

        /* Drive Motor PID Values */
        public static final FPID DRIVE_FPID = new FPID(
                0.02, 0.1, 0, 0);

        /* Angle Motor PID Values */
        public static final FPID ANGLE_FPID = new FPID(
                CHOSEN_MODULE.angleKF, CHOSEN_MODULE.angleKP, CHOSEN_MODULE.angleKI, CHOSEN_MODULE.angleKD);

        /*
         * Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         */
        public static final double DRIVE_KS = (0.32 / 12);
        public static final double DRIVE_KV = (1.51 / 12);
        public static final double DRIVE_KA = (0.27 / 12);
        public static final SimpleMotorFeedforward DRIVE_FF = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 5;
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = Math.PI;

        /* Neutral Modes */
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_ID = 10;
            public static final int ANGLE_ID = 15;
            public static final int CANCODER_ID = 50;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(158.688);
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
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(53.287);
            public static final boolean ENCODER_INVERTED = false;
            public static final SwerveModuleConstants MODULE_CONSTANTS = new SwerveModuleConstants(
                    3, WHEEL_CIRCUMFERENCE_METERS, MAX_SPEED, ANGLE_ENCODER_CPR, DRIVE_ENCODER_CPR, ANGLE_GEAR_RATIO,
                    DRIVE_GEAR_RATIO);
        }
        
        public static final SwerveModuleConstants[] ALL_MODULE_CONSTANTS = {Mod0.MODULE_CONSTANTS, Mod1.MODULE_CONSTANTS, Mod2.MODULE_CONSTANTS, Mod3.MODULE_CONSTANTS};
        public static final SwerveConstants SWERVE_CONSTANTS = new SwerveConstants(ALL_MODULE_CONSTANTS, SWERVE_KINEMATICS, INVERT_GYRO, MAX_SPEED);
    }

  public static final class IntakeConstants {
    public static final int MOTOR_ID = 40;
    public static final boolean MOTOR_INVERTED = true;

    public static final int STALL_LIMIT = 25;
    public static final int FREE_LIMIT = 20;

    public static final double FORWARD_THROTTLE = .75;
    public static final double REVERSE_THROTTLE = 0.25;
    public static final double WHEEL_RADIUS = 1;
    public static final double GEARING = 12;
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
    public static final double FORWARD_THROTTLE = .8;
    public static final double REVERSE_THROTTLE = .8;
  }

  public static final class FieldConstants {
        public static final Translation3d BLUE_SPEAKER_OPENING_TRANSLATION = new Translation3d(0.01, 5.556, 3.267);
        public static final Pose2d BLUE_SUBWOOFER_FRONT_POSE = new Pose2d(1.35, 5.556, new Rotation2d());
        
        public static final Translation3d RED_SPEAKER_OPENING_TRANSLATION = new Translation3d(16.53, 5.556, 3.267);
        public static final Pose2d RED_SUBWOOFER_FRONT_POSE = new Pose2d(15.20, 5.556, new Rotation2d());

    public static final Pose2d BLUE_AMP_POSE = new Pose2d(1.82, 7.66, Rotation2d.fromDegrees(90));
    public static final Pose2d RED_AMP_POSE = new Pose2d(14.70, 7.66, Rotation2d.fromDegrees(90));

    public static final Translation2d[] BLUE_AUTO_TRACK_BOUNDS = {
      new Translation2d(),
      new Translation2d()
    };
    public static final Translation2d[] RED_AUTO_TRACK_BOUNDS = {
      new Translation2d(),
      new Translation2d()
    };
    }

    public static final class LimelightConstants {
        public static final double MOUNT_ANGLE = 30;
        public static final double LENS_HEIGHT = Units.inchesToMeters(10); 
        public static final double GOAL_HEIGHT = Units.inchesToMeters(0); // not needed
        public static final double DISTANCE_OFFSET = 0; // not needed
    }

    public static final class AutoConstants {
        public static final double MAX_SPEED_MPS = 5.;  // in m/s 
        public static final double MAX_ACCEL_MPS_2 = 2.5; // 3. // in m/s^2 
        public static final double MAX_ANGULAR_SPEED_R_S = Math.PI / 4.; // Math.PI / 4.; // in radians/s 
        public static final double MAX_ANGULAR_ACCEL_R_S_2 = Math.PI / 8.; // Math.PI / 4.; // in radians/s^2 

        // TODO set rotation + translation PID values
        public static final double ROTATION_KP = .5;
        public static final double ROTATION_KI = 0;
        public static final double ROTATION_KD = 0;

        public static final double TRANSLATION_KP = 5.5;
        public static final double TRANSLATION_KI = 0;
        public static final double TRANSLATION_KD = 0;

        public static final double DRIVE_BASE_RADIUS = 0.6095; // in m, middle to corner
        public static final PathConstraints CONSTRAINTS = new PathConstraints(MAX_SPEED_MPS, MAX_ACCEL_MPS_2, MAX_ANGULAR_SPEED_R_S, MAX_ANGULAR_ACCEL_R_S_2);
        
    }
}

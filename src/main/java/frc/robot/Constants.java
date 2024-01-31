// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import org.frc5587.lib.pid.FPID;
import org.frc5587.lib.subsystems.SwerveBase.SwerveConstants;
import org.frc5587.lib.subsystems.SwerveModuleBase.SwerveModuleConstants;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
   public static final int LEFT_MOTOR_ID = 30;
   public static final int RIGHT_MOTOR_ID = 31;
   public static final boolean LEFT_MOTOR_INVERTED = false;
   public static final boolean RIGHT_MOTOR_INVERTED = true;
    
   //Values TBD, placeholders for now
    public static final double SPEAKER_SETPOINT = Math.toRadians(40);
    public static final double AMP_SETPOINT = Math.toRadians(-10);
    public static final double RESTING_SETPOINT = Math.toRadians(0);
    
    public static final double GEARING = 25;
    public static final double[] SOFT_LIMITS = {Math.toRadians(-10), Math.toRadians(20)};
    public static final double ZERO_OFFSET = 0.0;
    public static final int ENCODER_CPR = 1;
    public static final int[] SWITCH_PORTS = {2, 3};
    public static final boolean[] SWITCH_INVERTIONS = {false, false};
    public static final ProfiledPIDController PID = new ProfiledPIDController(3.8528, 0, 0.28713, new Constraints(1, 0.5));
    public static final ArmFeedforward FF = new ArmFeedforward(0.46656, 0.22857, 0.45468, 0.01122);
    public static final int STALL_LIMIT = 35;
    public static final int FREE_LIMIT = 40;

  }
    public static final class DrivetrainConstants {
        public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

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
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0));

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
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(231.943);
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
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(278.086);
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
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(346.113);
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
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(339.082);
            public static final boolean ENCODER_INVERTED = false;
            public static final SwerveModuleConstants MODULE_CONSTANTS = new SwerveModuleConstants(
                    3, WHEEL_CIRCUMFERENCE_METERS, MAX_SPEED, ANGLE_ENCODER_CPR, DRIVE_ENCODER_CPR, ANGLE_GEAR_RATIO,
                    DRIVE_GEAR_RATIO);
        }
        
        public static final SwerveModuleConstants[] ALL_MODULE_CONSTANTS = {Mod0.MODULE_CONSTANTS, Mod1.MODULE_CONSTANTS, Mod2.MODULE_CONSTANTS, Mod3.MODULE_CONSTANTS};
        public static final SwerveConstants SWERVE_CONSTANTS = new SwerveConstants(ALL_MODULE_CONSTANTS, SWERVE_KINEMATICS, INVERT_GYRO, MAX_SPEED);
    }

    public static final class LimelightConstants {
        public static final double MOUNT_ANGLE = 30;
        public static final double LENS_HEIGHT = Units.inchesToMeters(10); 
        public static final double GOAL_HEIGHT = Units.inchesToMeters(0); // not needed
        public static final double DISTANCE_OFFSET = 0; // not needed
    }

    public static final class AutoConstants {
        public static final double MAX_SPEED_MPS = 0.5; // 3.  // in m/s 
        public static final double MAX_ACCEL_MPS_2 = 0.25; // 3. // in m/s^2 
        public static final double MAX_ANGULAR_SPEED_R_S = Math.PI / 4.; // Math.PI / 4.; // in radians/s 
        public static final double MAX_ANGULAR_ACCEL_R_S_2 = Math.PI / 8.; // Math.PI / 4.; // in radians/s^2 

        // TODO set rotation + translation PID values
        public static final double ROTATION_KP = 0.05;
        public static final double ROTATION_KI = 0;
        public static final double ROTATION_KD = 0;

        public static final double TRANSLATION_KP = 0.05;
        public static final double TRANSLATION_KI = 0;
        public static final double TRANSLATION_KD = 0;

        public static final double DRIVE_BASE_RADIUS = 0.47;
        
    }
}
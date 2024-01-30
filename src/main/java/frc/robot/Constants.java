// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ArmConstants {
    public static class WristConstants {
        public static final Constraints CONSTRAINTS = new Constraints(5, 5, 0);
        public static final ProfiledPIDController PID_CONTROLLER = new ProfiledPIDController(5.5, 0, 0, CONSTRAINTS); //12pm // kD 1.9861
        public static final ArmFeedforward FF_CONTROLLER = new ArmFeedforward(0.7635, 2.0434, 0.074894);
        public static final ArmFeedforward HIGH_FF = new ArmFeedforward(0.55044, 2.2877, 0.3263);
        public static final double GEARING = 1;
        public static final int ENCODER_CPR = 1;
        public static final boolean LEFT_INVERTED = true;
        public static final boolean RIGHT_INVERTED = false;
        public static final int SWITCH_PORT = 0;
    }

    public static final int LEADER_MOTOR = 0;
    public static final int FOLLOWER_MOTOR = 0;
}

package frc.robot.subsystems;

import org.frc5587.lib.subsystems.SwerveBase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;

public class Swerve extends SwerveBase {
     private static SwerveModule[] swerveModules = {
            new SwerveModule(DrivetrainConstants.Mod0.MODULE_CONSTANTS, new TalonFX(10, "canivore"),
                    new TalonFX(15, "canivore"), new CANcoder(50, "canivore"), DrivetrainConstants.Mod0.ANGLE_OFFSET),
            new SwerveModule(DrivetrainConstants.Mod1.MODULE_CONSTANTS, new TalonFX(11, "canivore"),
                    new TalonFX(16, "canivore"), new CANcoder(51, "canivore"), DrivetrainConstants.Mod1.ANGLE_OFFSET),
            new SwerveModule(DrivetrainConstants.Mod2.MODULE_CONSTANTS, new TalonFX(12, "canivore"),
                    new TalonFX(17, "canivore"), new CANcoder(52, "canivore"), DrivetrainConstants.Mod2.ANGLE_OFFSET),
            new SwerveModule(DrivetrainConstants.Mod3.MODULE_CONSTANTS, new TalonFX(13, "canivore"),
                    new TalonFX(18, "canivore"), new CANcoder(53, "canivore"), DrivetrainConstants.Mod3.ANGLE_OFFSET)
    };

   

    public Swerve() {
        super(DrivetrainConstants.SWERVE_CONSTANTS, swerveModules);

        // Auto Config
            AutoBuilder.configureHolonomic(
                this::getOdometryPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(AutoConstants.TRANSLATION_KP, AutoConstants.TRANSLATION_KI, AutoConstants.TRANSLATION_KD), // Translation PID constants TODO set
                        new PIDConstants(AutoConstants.ROTATION_KP, AutoConstants.ROTATION_KI, AutoConstants.ROTATION_KD), // Rotation PID constants TODO set
                        AutoConstants.MAX_SPEED_MPS, // Max module speed, in m/s
                        AutoConstants.DRIVE_BASE_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

 
    
    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw().getDegrees());
        SmartDashboard.putNumber("Yaw Offset", gyro.getYawZeroOffset().getDegrees());
        if(SmartDashboard.getBoolean("Zero Yaw", true)) {
            gyro.zeroYaw();
        }
        SmartDashboard.putBoolean("Zero Yaw", false);
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

        for (int i = 0; i < swerveModules.length; i++) {
            SmartDashboard.putNumber("M"+i+" Raw CANCoder", swerveModules[i].getNonZeroedAbsoluteEncoderValue().getDegrees());
            SmartDashboard.putNumber("M" + i + " Adjusted CANCoder", swerveModules[i].getAbsoluteEncoderValue().getDegrees());
            SmartDashboard.putNumber("M" + i + " Relative", swerveModules[i].getAngle().getDegrees());
        }

        SmartDashboard.putData("Field", field);
    }
}
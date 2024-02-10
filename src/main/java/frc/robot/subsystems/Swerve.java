package frc.robot.subsystems;

import org.frc5587.lib.subsystems.SwerveBase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        for (int i = 0; i < swerveModules.length; i++) {
            SmartDashboard.putNumber("M"+i+" Raw CANCoder", swerveModules[i].getNonZeroedAbsoluteEncoderValue().getDegrees());
            SmartDashboard.putNumber("M" + i + " Adjusted CANCoder", swerveModules[i].getAbsoluteEncoderValue().getDegrees());
            SmartDashboard.putNumber("M" + i + " Relative", swerveModules[i].getAngle().getDegrees());
        }
        

    }
}
package frc.robot.subsystems;

import org.frc5587.lib.subsystems.SwerveBase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrivetrainConstants;

public class Swerve extends SwerveBase {
     private static SwerveModule[] swerveModules = {
            new SwerveModule(DrivetrainConstants.Mod0.MODULE_CONSTANTS, new TalonFX(10, "canivore"),
                    new TalonFX(15, "canivore"), new CANcoder(50)),
            new SwerveModule(DrivetrainConstants.Mod1.MODULE_CONSTANTS, new TalonFX(11, "canivore"),
                    new TalonFX(16, "canivore"), new CANcoder(51)),
            new SwerveModule(DrivetrainConstants.Mod2.MODULE_CONSTANTS, new TalonFX(12, "canivore"),
                    new TalonFX(17, "canivore"), new CANcoder(52)),
            new SwerveModule(DrivetrainConstants.Mod3.MODULE_CONSTANTS, new TalonFX(13, "canivore"),
                    new TalonFX(18, "canivore"), new CANcoder(53))
    };

    public Swerve() {
        super(DrivetrainConstants.SWERVE_CONSTANTS, swerveModules);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("CANcoder val", swerveModules[0].getAbsoluteEncoderValue().getDegrees());
        SmartDashboard.putNumber("Non-Converted Val", swerveModules[0].getAngleMotorEncoderPosition().getDegrees());
        SmartDashboard.putNumber("Converted Val", swerveModules[0].getAngle().getDegrees());
        SmartDashboard.putNumber("Gyro yaw", gyro.getYaw().getDegrees());
        SmartDashboard.putNumber("Yaw offset", gyro.getYawZeroOffset().getDegrees());
        if(SmartDashboard.getBoolean("Zero Yaw", true)) {
                gyro.zeroYaw();
        }
        SmartDashboard.putBoolean("Zero Yaw", false);
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }
}
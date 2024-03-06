package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;

public class CharacterizationManager {
    private final ArmCharacterization armCharacterization;
    private final SwerveCharacterization swerveCharacterization;

    public CharacterizationManager(Arm arm, Swerve swerve) {
        this.armCharacterization = new ArmCharacterization(arm);
        this.swerveCharacterization = new SwerveCharacterization(swerve);
    }

    public boolean armCharIsRunning() {
        return armCharacterization.isRunning();
    }

    public boolean swerveCharIsRunning() {
        return swerveCharacterization.isRunning();
    }

    public void characterizationPeriodic() {
        SmartDashboard.putNumber(null, 0);
        SmartDashboard.putBoolean("Arm Characterization Running", armCharIsRunning());
        SmartDashboard.putBoolean("Swerve Characterization Running", swerveCharIsRunning());
    }

    public ArmCharacterization getArmChar() {
        return this.armCharacterization;
    }
    
    public SwerveCharacterization getSwerveChar() {
        return this.swerveCharacterization;
    }
}

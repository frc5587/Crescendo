package frc.robot.commands;

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
    }

    public ArmCharacterization getArmChar() {
        return this.armCharacterization;
    }
    
    public SwerveCharacterization getSwerveChar() {
        return this.swerveCharacterization;
    }
}

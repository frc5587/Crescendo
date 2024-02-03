package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;

public class CharacterizationManager {
    private final ArmCharacterization armCharacterization;

    public CharacterizationManager(Arm arm, Swerve swerve) {
        this.armCharacterization = new ArmCharacterization(arm);
    }

    public boolean charIsRunning() {
        return armCharacterization.isRunning();
    }

    public void characterizationPeriodic() {
        SmartDashboard.putNumber(null, 0);
    }
}

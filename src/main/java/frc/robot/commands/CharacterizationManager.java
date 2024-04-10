package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class CharacterizationManager {
    private final ArmCharacterization armCharacterization;
    private final SwerveCharacterization swerveCharacterization;
    private final ShooterCharacterization shooterCharacterization;
    private final LeftShooterCharacterization leftShooterCharacterization;
    private final RightShooterCharacterization rightShooterCharacterization;
    private final ClimbCharacterization climbCharacterization;

    public CharacterizationManager(Arm arm, Swerve swerve, Shooter shooter, Climb climb) {
        this.armCharacterization = new ArmCharacterization(arm);
        this.swerveCharacterization = new SwerveCharacterization(swerve, arm);
        this.shooterCharacterization = new ShooterCharacterization(shooter);
        this.leftShooterCharacterization = new LeftShooterCharacterization(shooter);
        this.rightShooterCharacterization = new RightShooterCharacterization(shooter);
        this.climbCharacterization = new ClimbCharacterization(climb);
    }

    public boolean armCharIsRunning() {
        return armCharacterization.isRunning();
    }

    public boolean swerveCharIsRunning() {
        return swerveCharacterization.isRunning();
    }

    public boolean shooterCharIsRunning() {
        return shooterCharacterization.isRunning();
    }

    public boolean climbCharIsRunning() {
        return climbCharacterization.isRunning();
    }

    public ArmCharacterization getArmChar() {
        return this.armCharacterization;
    }
    
    public SwerveCharacterization getSwerveChar() {
        return this.swerveCharacterization;
    }

    public ShooterCharacterization getShooterChar() {
        return this.shooterCharacterization;
    }
    public LeftShooterCharacterization getLeftShooterChar() {
        return this.leftShooterCharacterization;
    }
    public RightShooterCharacterization getRightShooterChar() {
        return this.rightShooterCharacterization;
    }

    public ClimbCharacterization getClimbChar() {
        return this.climbCharacterization;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frc5587.lib.control.DeadbandCommandXboxController;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.CharacterizationManager;
import frc.robot.commands.DualStickSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  protected final Swerve swerve = new Swerve();
  protected final Arm arm = new Arm();
  private final CharacterizationManager charManager = new CharacterizationManager(arm, swerve);

  private final DeadbandCommandXboxController xbox =
      new DeadbandCommandXboxController(0);

      private final DualStickSwerve driveCommand = new DualStickSwerve(swerve, xbox::getLeftY, xbox::getLeftX,
           () -> {return -xbox.getRightX();}, () -> xbox.rightBumper().negate().getAsBoolean());

    private final SendableChooser<Command> charChooser = new SendableChooser<Command>();
  
  public RobotContainer() {
    // Configure the trigger bindings
    swerve.setDefaultCommand(driveCommand);
    configureBindings();
    charChooser.setDefaultOption("Arm Q Fwd", charManager.getArmChar().quasistatic(SysIdRoutine.Direction.kForward));
    charChooser.addOption("Arm Q Bwd", charManager.getArmChar().quasistatic(SysIdRoutine.Direction.kReverse));
    charChooser.addOption("Arm D Fwd", charManager.getArmChar().dynamic(SysIdRoutine.Direction.kForward));
    charChooser.addOption("Arm D Bwd", charManager.getArmChar().dynamic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("Char", charChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

return charChooser.getSelected();
  }
}

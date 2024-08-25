package frc.robot.subsystems;

import org.frc5587.lib.subsystems.LimelightBase;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends LimelightBase {
    SendableChooser<LedValues> ledChooser = new SendableChooser<LedValues>();

    public Limelight() {
        super(0, Units.inchesToMeters(30), 0, 0);

        ledChooser.setDefaultOption("DEFAULT", LedValues.DEFAULT);
        ledChooser.addOption("ON", LedValues.ON);
        ledChooser.addOption("OFF", LedValues.OFF);
        ledChooser.addOption("BLINK", LedValues.BLINK);
        SmartDashboard.putData("Limelight LEDs", ledChooser);

    }

    public void periodic() {
        setLEDs(ledChooser.getSelected());
    }

}

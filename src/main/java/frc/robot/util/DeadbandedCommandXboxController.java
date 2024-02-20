package frc.robot.util;

import org.frc5587.lib.control.DeadbandCommandXboxController;

import edu.wpi.first.wpilibj.XboxController;

public class DeadbandedCommandXboxController extends DeadbandCommandXboxController {
    public DeadbandedCommandXboxController(int channel) {
        super(channel);
    }

    public DeadbandedCommandXboxController(int channel, double deadbandCutoff) {
        super(channel, deadbandCutoff);
    }

    @Override
    public double getLeftX() {
        return getRawAxis(XboxController.Axis.kLeftX.value);
    }
    @Override
    public double getLeftY() {
        return getRawAxis(XboxController.Axis.kLeftY.value);
    }
    @Override
    public double getRightX() {
        return getRawAxis(XboxController.Axis.kRightX.value);
    }
}
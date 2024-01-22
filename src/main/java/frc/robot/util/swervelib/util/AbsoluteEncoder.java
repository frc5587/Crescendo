package frc.robot.util.swervelib.util;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AbsoluteEncoder extends DutyCycleEncoder {
    private final boolean encoderReversed;

    /**
     * Creates an AbsoluteEncoder that can be used for duty cycle/PWM encoders, including the SRX Mag Encoder.
     * <p>
     * Defaults DutyCycleRange to {@code 1.0/4096.0} and {@code 4095.0/4096.0} {@link DutyCycleEncoder#setDutyCycleRange(double, double)}
     * @param channel the channel to attach to
     * @param offset set the position offset in a range from 0-1
     */
    public AbsoluteEncoder(int channel, double offset, boolean encoderReversed) {
        super(channel);
        setPositionOffset(offset);
        setDutyCycleRange(1.0/4096.0, 4095.0/4096.0);

        this.encoderReversed = encoderReversed;
    }

    /**
     * Creates an AbsoluteEncoder that can be used for duty cycle/PWM encoders, including the SRX Mag Encoder.
     * @param channel the channel to attach to
     * @param offset set the position offset in a range from 0-1
     * @param dutyCycleRange 
     */
    public AbsoluteEncoder(int channel, double offset, boolean encoderReversed, double[] dutyCycleRange) {
        super(channel);
        setPositionOffset(offset);
        setDutyCycleRange(dutyCycleRange[0], dutyCycleRange[1]);

        this.encoderReversed = encoderReversed;
    }

    double angle;

    public double getValue() {
        angle = getAbsolutePosition() * (encoderReversed ? -1.0 : 1.0);

        return angle;
    }
}
package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public abstract class CharacterizationBase {
    protected enum MechanismType {
        Linear, Rotational
    }
    protected final MechanismType type;
    protected SysIdRoutine routine;
    protected Config routineConfig;
    protected final Subsystem subsystem;
    protected Mechanism armMechanism;
    protected MutableMeasure<Velocity<Voltage>> quasistaticRamp = MutableMeasure
            .mutable(Units.Volts.per(Units.Second).of(0.25));
    protected MutableMeasure<Voltage> dynamicVoltage = MutableMeasure.mutable(Units.Volts.of(1.5));

    protected MutableMeasure<Voltage> outputVoltage = MutableMeasure.mutable(Units.Volts.of(0));
    protected MutableMeasure<Angle> mechanismAngle = MutableMeasure.mutable(Units.Rotations.of(0));
    protected MutableMeasure<Distance> mechanismDistance = MutableMeasure.mutable(Units.Meters.of(0));
    protected MutableMeasure<Velocity<Angle>> angularVelocity = MutableMeasure.mutable(Units.RotationsPerSecond.of(0));
    protected MutableMeasure<Velocity<Distance>> linearVelocity = MutableMeasure.mutable(Units.MetersPerSecond.of(0));
    protected MutableMeasure<Velocity<Velocity<Angle>>> angularAccel = MutableMeasure
            .mutable(Units.RotationsPerSecond.per(Units.Second).of(0));
    protected MutableMeasure<Velocity<Velocity<Distance>>> linearAccel = MutableMeasure
            .mutable(Units.MetersPerSecondPerSecond.of(0));

    public CharacterizationBase(MechanismType type, Subsystem subsystem) {
        this.type = type;
        this.subsystem = subsystem;
        this.armMechanism = new Mechanism((volts) -> setVoltage(volts.in(Units.Volts)), log(), subsystem);
        this.routineConfig = new Config(quasistaticRamp, dynamicVoltage, Units.Seconds.of(10));
        this.routine = new SysIdRoutine(routineConfig, armMechanism);
    }

    private Consumer<SysIdRoutineLog> logRotational() {
        return (log) -> {log.motor("motor")
                .voltage(outputVoltage
                        .mut_replace(Units.Volts.of(getMotorVoltage())))
                .angularPosition(
                        mechanismAngle.mut_replace(Units.Rotations.of(getMechanismPosition())))
                .angularVelocity(angularVelocity
                        .mut_replace(Units.RotationsPerSecond.of(getMechanismVelocity())))
                .angularAcceleration(angularAccel.mut_replace(Units.RotationsPerSecond.per(Units.Second)
                        .of(getMechanismAcceleration())));
                        SmartDashboard.putNumber("Voltage", outputVoltage.magnitude());
                        SmartDashboard.putNumber("Position", mechanismAngle.magnitude());
                        SmartDashboard.putNumber("Velocity", angularVelocity.magnitude());
                        SmartDashboard.putNumber("Acceleration", angularAccel.magnitude());
                };
    }

    private Consumer<SysIdRoutineLog> logLinear() {
        return (log) -> log.motor("motor")
                .voltage(outputVoltage
                        .mut_replace(Units.Volts.of(getMotorVoltage())))
                .linearPosition(
                        mechanismDistance.mut_replace(Units.Meters.of(getMechanismPosition())))
                .linearVelocity(linearVelocity
                        .mut_replace(Units.MetersPerSecond.of(getMechanismVelocity())))
                .linearAcceleration(linearAccel.mut_replace(Units.MetersPerSecondPerSecond
                        .of(getMechanismAcceleration())));
    }

    private Consumer<SysIdRoutineLog> log() {
        return this.type == MechanismType.Rotational ? logRotational() : logLinear();
    }

    /**
     * Sets the motor's output voltage
     */
    public abstract void setVoltage(double volts);

    /**
     * @return the motor's output voltage
     */
    public abstract double getMotorVoltage();

    /**
     * @return the position of the mechanism. If type is set to Rotational, this should
     * be in rotations. If type is Linear, this should be in meters.
     */
    public abstract double getMechanismPosition();

    /**
     * @return the velocity of the mechanism. If type is set to Rotational, this should
     * be in rotations per second. If type is Linear, this should be in meters per second.
     */
    public abstract double getMechanismVelocity();

    /**
     * @return the acceleration of the mechanism. If type is set to Rotational, this should
     * be in rotations per second squared. If type is Linear, this should be in meters per
     * second squared.
     */
    public abstract double getMechanismAcceleration();

    public Command preRoutine() {
        return Commands.none();
    };

    public Command postRoutine() {
        return Commands.none();
    }

    public Command quasistatic(Direction direction) {
        return preRoutine().andThen(routine.quasistatic(direction)).andThen(postRoutine());
    }

    public Command dynamic(Direction direction) {
        return preRoutine().andThen(routine.dynamic(direction)).andThen(postRoutine());
    }

    public boolean isRunning() {
        return (this.quasistatic(Direction.kForward).isScheduled() || this.quasistatic(Direction.kReverse).isScheduled()
                || this.dynamic(Direction.kForward).isScheduled() || this.dynamic(Direction.kReverse).isScheduled());
    }
}

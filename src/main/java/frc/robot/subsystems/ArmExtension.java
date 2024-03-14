package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class ArmExtension extends ProfiledPIDSubsystem {

    CANSparkMax armExtension = new CANSparkMax(0, MotorType.kBrushless);
    CANSparkMax armExtension_Follower = new CANSparkMax(0, MotorType.kBrushless);

    RelativeEncoder armEncoder = armExtension.getEncoder();
    SysIdRoutine armRoutine = new SysIdRoutine(new Config(), new Mechanism((Measure<Voltage> volts) -> {
        armExtension.set(volts.in(Units.Volts) / armExtension.getBusVoltage());
    }, (SysIdRoutineLog log) -> {
        log.motor("armExtension")
            .voltage(Units.Volts.of(armExtension.get()*armExtension.getBusVoltage()))
            .linearPosition(Units.Meters.of(armEncoder.getPosition()))
            .linearVelocity(Units.MetersPerSecond.of(armEncoder.getVelocity()));
    }, this));

    public ArmExtension() {
        super(new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)));

        armExtension.restoreFactoryDefaults();
        armExtension_Follower.restoreFactoryDefaults();
        armExtension_Follower.follow(armExtension, true);
    }

    // TODO: Implement
    @Override
    protected void useOutput(double output, State setpoint) {
        // TODO: implement
    }

    @Override
    protected double getMeasurement() {
        return 0;
        // TODO: implement
    }

    public Command getQuasistaticSysidCommand(Direction direction){
        return armRoutine.quasistatic(direction).finallyDo(()->{armExtension.set(0);});
    }

    public Command getDynamicSysidCommand(Direction direction){
        return armRoutine.dynamic(direction).finallyDo(()->{armExtension.set(0);});
    }
}

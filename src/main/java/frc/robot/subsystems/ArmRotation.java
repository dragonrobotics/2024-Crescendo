package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class ArmRotation extends ProfiledPIDSubsystem {

    CANSparkMax armRotation = new CANSparkMax(20, MotorType.kBrushless);
    CANSparkMax armRotation_Follower = new CANSparkMax(24, MotorType.kBrushless);

    RelativeEncoder armEncoder = armRotation.getEncoder();
    int callCount = 0;

    public ArmRotation() {
        super(
            new ProfiledPIDController(.64, 0, 0, new Constraints(60, 100))
            );
        SmartDashboard.putData("PidController", getController());

        armRotation.restoreFactoryDefaults();
        armRotation_Follower.restoreFactoryDefaults();

        armRotation.setIdleMode(IdleMode.kBrake);
        armRotation_Follower.setIdleMode(IdleMode.kBrake);

        armEncoder.setPositionConversionFactor(90.0 / 34.0);

        armEncoder.setVelocityConversionFactor((90.0 / 34.0) / 60.0);

        armRotation_Follower.follow(armRotation, true);

        setGoal(0);
enable();
        SmartDashboard.putData("Arm Rotation Subsystem", this);
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        callCount++;
        SmartDashboard.putNumber("Call Count", callCount);
        armRotation.set(.2);
    }

    @Override
    protected double getMeasurement() {
        return armEncoder.getPosition();
    }

    public boolean atGoal() {
        return getController().atGoal();
    }

    public Command rotateToPosition(double angle) {
        SmartDashboard.putNumber("TAngle", angle);
        return sequence(
                new PrintCommand("Set Angle: " + angle),
                runOnce(() -> {
                    setGoal(angle);
                }),
                waitUntil(this::atGoal));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Angle", getMeasurement());
        SmartDashboard.putBoolean("Arm At Goal", atGoal());
    }
}

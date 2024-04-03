package frc.robot.subsystems;

import static java.lang.Math.abs;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Intake extends ProfiledPIDSubsystem {
    private DigitalInput beamBreak = new DigitalInput(8);
    private CANSparkMax intakePull = new CANSparkMax(10, MotorType.kBrushless);
    private CANSparkMax intakePull2 = new CANSparkMax(16, MotorType.kBrushless);

    private CANSparkMax intakeToggle = new CANSparkMax(12, MotorType.kBrushless);
    private CANSparkMax intakeToggle2 = new CANSparkMax(17, MotorType.kBrushless);

    private RelativeEncoder toggleEncoder = intakeToggle2.getEncoder();

    public enum intakePosition {
        up(0),
        none(0),
        down(135);

        public double angle;

        private intakePosition(double value) {
            this.angle = value;
        }
    }

    public Intake() {
        super(new ProfiledPIDController(.14, 0, 0, new Constraints(1000, 1000)));
        SmartDashboard.putData("Intake Controller", getController());
        intakePull.restoreFactoryDefaults();
        intakePull2.restoreFactoryDefaults();
        intakeToggle.restoreFactoryDefaults();
        intakeToggle2.restoreFactoryDefaults();
        intakeToggle2.follow(intakeToggle, true);
        intakePull.follow(intakePull2);
        getController().setTolerance(2);
        intakeToggle.setIdleMode(IdleMode.kCoast);
        intakeToggle2.setIdleMode(IdleMode.kCoast);

        intakeToggle.setInverted(false);

        toggleEncoder.setPositionConversionFactor(135.0/28.0);
        toggleEncoder.setPosition(0);
        intakeToggle.burnFlash();
        intakeToggle2.burnFlash();
    }

    public double GetIntakeAngle() {
        return toggleEncoder.getPosition();
    }

    public intakePosition GetIntakePosition() {
        if (GetIntakeAngle() > 120 && GetIntakeAngle() < 300) {
            return intakePosition.down;
        } else if (GetIntakeAngle() < 15 || GetIntakeAngle() > 300) {
            return intakePosition.up;
        } else
            return intakePosition.none;
    }

    public boolean IntakeStopped() {
        return abs(toggleEncoder.getVelocity()) < 5;
    }

    public boolean hasNote() {
        return !beamBreak.get();
    }

    public Command intakeNote(CommandXboxController controller) {
        return sequence(
                SetIntakePosition(intakePosition.down),
                runOnce(() -> {
                    setVoltage(6);
                }),
                waitUntil(() -> {
                    return hasNote();
                }),
                runOnce(() -> {
                    stopIntake();
                }),
                runOnce(() -> {
                    controller.getHID().setRumble(RumbleType.kBothRumble, 1);
                }),
                SetIntakePosition(intakePosition.up))
                .finallyDo(() -> {
                    controller.getHID().setRumble(RumbleType.kBothRumble, 0);
                    stopIntake();
                });
    }

    public Command SetIntakePosition(intakePosition position) {
        return runOnce(() -> {
            setGoal(position.angle);
        }).andThen(waitUntil(() -> {
            return getController().atGoal();
        }));
    }

    public void stopIntake() {
        setVoltage(0);
    }

    public void setVoltage(double voltage) {
        intakePull2.setVoltage(-voltage);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Intake Angle", toggleEncoder.getPosition());
        SmartDashboard.putBoolean("Intake Has Note", hasNote());
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        intakeToggle.setVoltage(output);
    }

    @Override
    protected double getMeasurement() {
        return toggleEncoder.getPosition();
    }
}

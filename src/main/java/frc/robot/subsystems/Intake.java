package frc.robot.subsystems;

import static java.lang.Math.abs;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Intake extends SubsystemBase {
    private DigitalInput beamBreak = new DigitalInput(2);
    private CANSparkMax intakePull = new CANSparkMax(22, MotorType.kBrushless);
    private CANSparkMax intakePull2 = new CANSparkMax(23, MotorType.kBrushless);
    
    private CANSparkMax intakeToggle = new CANSparkMax(12, MotorType.kBrushless);
    private CANSparkMax intakeToggle2 = new CANSparkMax(25, MotorType.kBrushless);
    
    private AbsoluteEncoder toggleEncoder = intakeToggle.getAbsoluteEncoder(Type.kDutyCycle);
    
    public enum intakePosition {
        up(5),
        none(0),
        down(133);
        
        public double angle;
        
        private intakePosition(double value) {
            this.angle = value;
        }
    }
    
    public Intake() {
        intakePull.restoreFactoryDefaults();
        intakePull2.restoreFactoryDefaults();
        intakeToggle2.follow(intakeToggle, true);
        intakePull.follow(intakePull2);

        intakeToggle.setIdleMode(IdleMode.kBrake);
        intakeToggle2.setIdleMode(IdleMode.kBrake);

        intakeToggle.setInverted(false);

        toggleEncoder.setPositionConversionFactor(360);
        toggleEncoder.setInverted(true);

    }

    public double GetIntakeAngle() {
        return toggleEncoder.getPosition();
    }

    public intakePosition GetIntakePosition() {
        if (GetIntakeAngle() > 120 && GetIntakeAngle() < 300) {
            return intakePosition.down;
        } else if (GetIntakeAngle() < 15 || GetIntakeAngle() >  300) {
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
        return SetIntakePosition(intakePosition.down).andThen(runOnce(() -> {
            setVoltage(4);
        })).andThen(waitUntil(() -> {
            return hasNote();
        })).andThen(
                sequence(
                    runOnce(() -> {
                        controller.getHID().setRumble(RumbleType.kBothRumble, 1);
                    })),
                        runOnce(() -> {
                            stopIntake();
                        }),
                        SetIntakePosition(intakePosition.up))
                .finallyDo(() -> {
                    controller.getHID().setRumble(RumbleType.kBothRumble, 0);
                    stopIntake();
                });
    }

    public Command SetIntakePosition(intakePosition position) {

        return runOnce(() -> {
            double movePower = 8;
            if (position == intakePosition.up) {
                movePower *= -4;
            }
            intakeToggle.setVoltage(movePower);
        }).andThen(waitUntil(() -> {
            return GetIntakePosition().angle == position.angle;
        })).finallyDo((boolean intr) -> {
            System.out.println("At point");
            stopRotate();
        }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public void stopIntake() {
        setVoltage(0);
    }

    public void setVoltage(double voltage) {
        intakePull2.setVoltage(-voltage);
    }

    public void setRotateVoltage(double voltage) {
        intakeToggle.setVoltage(voltage);
    }

    public void stopRotate() {
        intakeToggle.stopMotor();
    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("Intake Angle", toggleEncoder.getPosition());
        SmartDashboard.putBoolean("Intake Has Note", hasNote());
    }

    @Override
    public void initSendable(SendableBuilder builder){
    }
}

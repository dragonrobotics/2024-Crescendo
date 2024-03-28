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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class ArmRotation extends ProfiledPIDSubsystem {
    
    public enum RotationAngle {
        Down(0),
        Amp(90),
        Trap(110);

        public double angle;
        private RotationAngle(double angle){
            this.angle = angle;
        }
    }

    CANSparkMax armRotation = new CANSparkMax(20, MotorType.kBrushless);
    CANSparkMax armRotation_Follower = new CANSparkMax(24, MotorType.kBrushless);

    RelativeEncoder armEncoder = armRotation.getEncoder();
    RotationAngle currentTarget = RotationAngle.Amp;

    public ArmRotation() {
        super(//.64
            new ProfiledPIDController(1.4, 0, 0, new Constraints(1600, 800))
            );
        getController().setTolerance(1);

        armRotation.restoreFactoryDefaults();
        armRotation_Follower.restoreFactoryDefaults();

        armRotation.setIdleMode(IdleMode.kBrake);
        armRotation_Follower.setIdleMode(IdleMode.kBrake);

        armEncoder.setPositionConversionFactor(90.0 / 34.0);

        armEncoder.setVelocityConversionFactor((90.0 / 34.0) / 60.0);

        armRotation_Follower.follow(armRotation, true);

        setGoal(currentTarget.angle);
        enable();
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        armRotation.set(output/armRotation.getBusVoltage());
    }

    @Override
    protected double getMeasurement() {
        return armEncoder.getPosition();
    }

    public boolean atGoal() {
        return getController().atGoal();
    }

    public RotationAngle getRotation(){
        return currentTarget;
    }

    public Command rotateToPosition(RotationAngle angle) {
        return sequence(
                runOnce(() -> {
                    currentTarget = angle;
                    setGoal(angle.angle);
                }),
                waitUntil(this::atGoal));
    }

    

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Arm Angle", getMeasurement());
        SmartDashboard.putString("Arm Rotation Position", getRotation().name());
        SmartDashboard.putBoolean("Arm At Goal", atGoal());
    }
}

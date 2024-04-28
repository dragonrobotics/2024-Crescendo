package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class ArmRotation extends ProfiledPIDSubsystem {
    
    public enum RotationAngle {
        Down(0),
        Raised(13), // 11 // 22 // 23
        Amp(90),
        Trap(110),
        Custom(0);

        public double angle;
        private RotationAngle(double angle){
            this.angle = angle;
        }
    }

    CANSparkMax armRotation = new CANSparkMax(25, MotorType.kBrushless);
    CANSparkMax armRotation_Follower = new CANSparkMax(20, MotorType.kBrushless);

    RelativeEncoder armEncoder = armRotation.getEncoder();
    RotationAngle currentTarget = RotationAngle.Down;

    public ArmRotation() {
        super(//.64
            new ProfiledPIDController(.64, 0, 0, new Constraints(400, 200))
            );
        getController().setTolerance(1);
        SmartDashboard.putData("Arm Rotation PID", getController());
        armRotation.restoreFactoryDefaults();
        armRotation_Follower.restoreFactoryDefaults();

        armRotation.setIdleMode(IdleMode.kBrake);
        armRotation_Follower.setIdleMode(IdleMode.kBrake);

        double conversionFactor = (360/337.5);

        armEncoder.setPositionConversionFactor(conversionFactor);

        armEncoder.setVelocityConversionFactor((conversionFactor) / 60.0);
        armEncoder.setPosition(1.63);
        armRotation_Follower.follow(armRotation, true);
        armRotation.burnFlash();
        armRotation_Follower.burnFlash();
        setGoal(currentTarget.angle);
        enable();
    }

    public void climbSpeed(){
        getController().setConstraints(new Constraints(200, 50));
    }

    public void normalSpeed(){
        getController().setConstraints(new Constraints(400, 200));
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

    public void setBrake(boolean b) {

        armRotation.setIdleMode(b ? IdleMode.kBrake : IdleMode.kCoast);

        armRotation_Follower.setIdleMode(b ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public Command rotateToCustomAngle(double d) {
        currentTarget = RotationAngle.Custom;
        currentTarget.angle = d;
        return rotateToPosition(currentTarget);
    }
}

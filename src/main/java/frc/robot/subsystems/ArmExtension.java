package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class ArmExtension extends ProfiledPIDSubsystem {
    
    public enum ExtensionPosition {
        In(0),
        Amp(23),
        ClimbUp(Amp.distance),
        ClimbDown(0),
        Trap(40);

        public double distance;
        private ExtensionPosition(double distance){
            this.distance = distance;
        }
    }
    

    CANSparkMax armExtension = new CANSparkMax(14, MotorType.kBrushless);
    double target = 0;
    RelativeEncoder armEncoder = armExtension.getEncoder();
    int callCount = 0;
    ExtensionPosition currentTarget;
    
        final double climbSpeed = 100;
        final double normalSpeed = 200;
        final double normalAcceleration = 50;

    public ArmExtension() {
        super(
                new ProfiledPIDController(2, 0, 0, new Constraints(400, 100)));

        getController().setTolerance(2);
        armExtension.restoreFactoryDefaults();
        armExtension.setInverted(false);
        armExtension.setIdleMode(IdleMode.kBrake);
        armEncoder.setPositionConversionFactor(.47);

        armEncoder.setVelocityConversionFactor((1.5) / 60.0);

        armExtension.burnFlash();
        // setGoal(3);
        enable();
        currentTarget = ExtensionPosition.In;
        SmartDashboard.putData(getController());
    }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        armExtension.set(output / armExtension.getBusVoltage());
    }

    public void climbSpeed(){
        getController().setConstraints(new Constraints(25, 10));
    }

    public void normalSpeed(){
        getController().setConstraints(new Constraints(400, 100));
    }

    @Override
    protected double getMeasurement() {
        return armEncoder.getPosition();
    }

    public boolean atGoal() {
        return getController().atGoal();
    }

    public Command extendToPosition(ExtensionPosition position) {
        return sequence(
                runOnce(() -> {
                    currentTarget = position;
                    setGoal(position.distance);
                }),
                waitUntil(this::atGoal));
    }

    public ExtensionPosition getPosition(){
        return currentTarget;
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Arm Length", getMeasurement());
        SmartDashboard.putString("Arm Extension Position", currentTarget.name());
        SmartDashboard.putBoolean("Arm Extension at Goal", atGoal());
    }

    public void setBrake(boolean b) {
        armExtension.setIdleMode(b ? IdleMode.kBrake : IdleMode.kCoast);
    }
}

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtendoArm extends SubsystemBase {
    private CANSparkMax leftRotateMotor = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax rightRotateMotor = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax leftElevatorMotor = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax rightElevatorMotor = new CANSparkMax(0, MotorType.kBrushless);

    private AbsoluteEncoder angleEncoder = rightRotateMotor.getAbsoluteEncoder(Type.kDutyCycle);
    private AbsoluteEncoder elevatorEncoder = rightElevatorMotor.getAbsoluteEncoder(Type.kDutyCycle);

    private SparkPIDController angleController = rightRotateMotor.getPIDController();
    private double angleP = 0.1;
    private double angleI = 0.0;
    private double angleD = 0.0;

    private SparkPIDController elevatorController = rightElevatorMotor.getPIDController();
    private double elevatorP = 0.1;
    private double elevatorI = 0.0;
    private double elevatorD = 0.0;

    public ExtendoArm() {
        leftRotateMotor.follow(rightRotateMotor);
        angleController.setFeedbackDevice(angleEncoder);
        angleController.setP(angleP);
        angleController.setI(angleI);
        angleController.setD(angleD);
        angleEncoder.setPositionConversionFactor(360);

        leftElevatorMotor.follow(rightElevatorMotor);
        elevatorController.setFeedbackDevice(elevatorEncoder);
        elevatorController.setP(elevatorP);
        elevatorController.setI(elevatorI);
        elevatorController.setD(elevatorD);
        elevatorEncoder.setPositionConversionFactor(1); // TODO: Find actual value (unit: meters?)
    }

    public void setAngle(double angle) {
        // TODO: Add limit checking
        angleController.setReference(angle, ControlType.kPosition);
    }

    public double getAngle() {
        return angleEncoder.getPosition();
    }

    public void setExtension(double length) {
        // TODO: Add limits
        elevatorController.setReference(length, ControlType.kPosition);
    }

    public double getExtension() {
        return elevatorEncoder.getPosition();
    }

    public Command SetAngle(double angle) {
        /* TODO: Make sure it can't break by going down when it's extended */
        return runOnce(() -> {
            setAngle(angle);
        }).andThen(
                waitUntil(() -> {
                    return Math.abs(getAngle() - angle) < 5;
                    /* TODO: Find better way to deadzone it */}));
    }

    public Command SetExtension(double height) {
        /* TODO: Make sure it can't break itsself by extending when its up */
        return runOnce(() -> {
            setExtension(height);
        }).andThen(
                waitUntil(() -> {
                    return Math.abs(getExtension() - height) < 5;
                    /* TODO: Find better way to deadzone it */}));
    }

}

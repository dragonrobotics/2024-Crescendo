package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class ExtendoArm extends SubsystemBase {
    private CANSparkMax leftRotateMotor = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax rightRotateMotor = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax leftElevatorMotor = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax rightElevatorMotor = new CANSparkMax(0, MotorType.kBrushless);

    private RelativeEncoder angleEncoder = rightRotateMotor.getEncoder();
    private RelativeEncoder elevatorEncoder = rightElevatorMotor.getEncoder();

    private DigitalInput elevatorLimitSwitch = new DigitalInput(0); // TODO: get actual port

    private SparkPIDController angleController = rightRotateMotor.getPIDController();
    private double angleP = 0.1; // TODO: Tune pid
    private double angleI = 0.0;
    private double angleD = 0.0;

    private SparkPIDController elevatorController = rightElevatorMotor.getPIDController();
    private double elevatorP = 0.1; // TODO: Tune pid
    private double elevatorI = 0.0;
    private double elevatorD = 0.0;

    public ExtendoArm() {
        leftRotateMotor.follow(rightRotateMotor);
        angleController.setFeedbackDevice(angleEncoder);
        angleController.setP(angleP);
        angleController.setI(angleI);
        angleController.setD(angleD);
        angleEncoder.setPositionConversionFactor(360); // TODO: Find actual conversion factor (based on gear ratio, easy math.)
        angleEncoder.setPosition(0);
        leftElevatorMotor.follow(rightElevatorMotor);
        elevatorController.setFeedbackDevice(elevatorEncoder);
        elevatorController.setP(elevatorP);
        elevatorController.setI(elevatorI);
        elevatorController.setD(elevatorD);
        elevatorEncoder.setPositionConversionFactor(1); // TODO: Find actual value (unit: meters?)

        run(()->{setElevatorVoltage(-2);}).until(()->{return elevatorLimitSwitch.get();}).andThen(runOnce(()->{elevatorEncoder.setPosition(0);})).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).schedule();
    }

    public void setAngle(double angle) {
        // TODO: Add limit checking
        angleController.setReference(angle, ControlType.kPosition);
    }

    public double getAngle() {
        return angleEncoder.getPosition();
    }

    public boolean isAngleDown(){
        if(getAngle() < 2){ return true;}
        else{return false;}
    }

    public void setExtension(double length) {
        // TODO: Add limits
        elevatorController.setReference(length, ControlType.kPosition);
    }

    public double getExtension() {
        return elevatorEncoder.getPosition();
    }

    public boolean isExtendDown(){
        if(getExtension() < 2){return true;}
        else{return false;}
    }

    public Command SetAngle(double angle) {
        return runOnce(() -> {
        setAngle(angle);
        })
        .andThen(waitUntil(() -> {
            return Math.abs(getAngle() - angle) < 5;
            /* TODO: Find better way to deadzone it */}));
        }

    public Command SetExtension(double height) {
        return runOnce(() -> {
        setExtension(height);
        })
        .andThen(waitUntil(() -> {
            return Math.abs(getExtension() - height) < 5;
            /* TODO: Find better way to deadzone it */}));
        }

  

    public void setElevatorVoltage(double voltage){
        rightElevatorMotor.setVoltage(voltage);
    }

}

package frc.robot.subsystems;
import static java.lang.Math.abs;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Intake extends SubsystemBase{
    private DigitalInput beamBreak = new DigitalInput(1);

    private CANSparkMax intakePull = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax intakePull2 = new CANSparkMax(3, MotorType.kBrushless);

    private CANSparkMax intakeToggle = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax intakeToggle2 = new CANSparkMax(3, MotorType.kBrushless);

    private AbsoluteEncoder toggleEncoder = intakeToggle.getAbsoluteEncoder(Type.kDutyCycle);
    
    private SparkPIDController toggleController = intakeToggle.getPIDController();

    private double toggleP = 0.1;
    private double toggleI = 0;
    private double toggleD = 0;

    public enum intakePosition{
        up(90),
        down(0);

        public double angle;

        private intakePosition(double value){  
            this.angle=value;   
        }  
    }

    public Intake()
    {
        intakePull2.follow(intakePull);
        intakeToggle2.follow(intakeToggle);

        toggleController.setP(toggleP);
        toggleController.setI(toggleI);
        toggleController.setD(toggleD);

        toggleEncoder.setPositionConversionFactor(360);
        toggleEncoder.setVelocityConversionFactor(360/60);
    }

    public void ChangeIntakePosition(intakePosition chosenPosition)
    {
        toggleController.setReference(chosenPosition.angle, ControlType.kPosition);
    }

    public double GetIntakeAngle() {
        return toggleEncoder.getPosition();
    }

    public intakePosition GetIntakePosition()
    {
        return GetIntakeAngle() > 45 ? intakePosition.up : intakePosition.down;
    }

    public boolean IntakeStopped(){
        return abs(toggleEncoder.getVelocity()) < 5;
    }

    public boolean HasNote()
    {
        return beamBreak.get();
    }

    public Command intakeNote()
    {
        return SetIntakePosition(intakePosition.down).andThen(runOnce(()->{
            intakePull.setVoltage(0.2);
        })).andThen(waitUntil(()->{
            return HasNote();
        })).andThen(
            parallel(
                SetIntakePosition(intakePosition.up),
                runOnce(()->{stopIntake();})
            )
        ).finallyDo(()->{
            stopIntake();
        });
    }

    public Command SetIntakePosition(intakePosition position)
    {
        return run(()->{
            ChangeIntakePosition(position);
        }).until(()->{
            return GetIntakePosition() == position && IntakeStopped();
        });
    }

    public void stopIntake(){
        setVoltage(0);
    }

    public void setVoltage(double voltage){
        intakePull.setVoltage(voltage);
    }

}

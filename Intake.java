package frc.robot.subsystems;
import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public enum intakePosition{up, down}

    public Intake()
    {
        intakePull2.follow(intakePull);
        intakeToggle2.follow(intakeToggle);

        toggleController.setP(toggleP);
        toggleController.setI(toggleI);
        toggleController.setD(toggleD);

        toggleEncoder.setPositionConversionFactor(360);
    }

    public void ChangeIntakePosition(intakePosition chosenPosition)
    {
        if(chosenPosition == intakePosition.up)
        {
            toggleController.setReference(90, ControlType.kPosition);
        }
        else if(chosenPosition == intakePosition.down)
        {
            toggleController.setReference(0, ControlType.kPosition);
        }
        else
        {
            System.out.println("ChangeIntakePosition must take in either 'intakePosition.up' or 'intakePosition.down'");
        }
    }

    public intakePosition GetIntakePosition()
    {
        if(toggleEncoder.getPosition() >= 88)
        {    return intakePosition.up;   }

        else if(toggleEncoder.getPosition() >= -1 && toggleEncoder.getPosition() <= 1)
        {    return intakePosition.down;   }

        else { return null; }
    }

    public BooleanSupplier notePresent()
    {
        BooleanSupplier beam;
        if(beamBreak.get() == true){beam = () -> true;}
        else{beam = () -> false;}
        
        return beam;
    }

    public Command runIntake()
    {
        return run(() -> 
        {
            if(GetIntakePosition() == intakePosition.down)
            {
                intakePull.setVoltage(0.2);
            }
            if(GetIntakePosition() == intakePosition.up)
            {
                ChangeIntakePosition(intakePosition.down);
            }
            
        }).until(notePresent()).withTimeout(5).finallyDo(() ->
        { 
            intakePull.setVoltage(0);
        });
    }

}

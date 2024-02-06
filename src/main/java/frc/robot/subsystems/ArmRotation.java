package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmRotation extends SubsystemBase{
    private CANSparkMax leftRotateMotor = new CANSparkMax(0,MotorType.kBrushless);
    private CANSparkMax rightRotateMotor = new CANSparkMax(0,MotorType.kBrushless);

    private AbsoluteEncoder rightRotateEncoder = rightRotateMotor.getAbsoluteEncoder(Type.kDutyCycle);

    private SparkPIDController rightController = rightRotateMotor.getPIDController();
    private double MotorP = 0.1;
    private double MotorI = 0.0;
    private double MotorD = 0.0;

    public ArmRotation(){
        leftRotateMotor.follow(rightRotateMotor);
        rightController.setP(MotorP);
        rightController.setI(MotorI);
        rightController.setD(MotorD);
        rightRotateEncoder.setPositionConversionFactor(360);
    }
    public void RotateDirection( boolean wantUp){
        if ( wantUp == true){
            rightController.setReference(80,ControlType.kPosition);
        }
        else if (wantUp == false){
            rightController.setReference(20,ControlType.kPosition);

        }
    }
    public boolean isUp(){
        if (rightRotateEncoder.getPosition() >= 80){
            return true;

        }
        else {
            return false;
        }
    }
}

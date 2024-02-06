package frc.robot.subsystems;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private CANSparkMax leftElevatorMotor = new CANSparkMax(0,MotorType.kBrushless);
    private CANSparkMax rightElevatorMotor = new CANSparkMax(0,MotorType.kBrushless);

    private AbsoluteEncoder rightElevatorEncoder = rightElevatorMotor.getAbsoluteEncoder(Type.kDutyCycle);

    private SparkPIDController rightController = rightElevatorMotor.getPIDController();
    private double motorP =0.1;
    private double motorI = 0.0;
    private double motorD = 0.0;



    public Elevator(){
        leftElevatorMotor.follow(rightElevatorMotor);
        rightController.setP(motorP);
        rightController.setI(motorI);
        rightController.setD(motorD);
    }
    
    public void elevatordirection(int chosenPosition){

        if ( chosenPosition == 0){
            rightController.setReference(0,ControlType.kPosition);
        }
        
        else if ( chosenPosition == 1 ){
            rightController.setReference(1,ControlType.kPosition);
        }
        else{
            System.out.println("Error");
        }
    }
    public int getElevatorPosition(){
        if (rightElevatorEncoder.getPosition() >= 10){
            return 0;
        }
        else if (rightElevatorEncoder.getPosition() <= 100){
            return 1;
        }
        else{
            return 0;
        }
    }
}
package frc.robot.subsystems;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Yeeter extends SubsystemBase {
    public CANSparkMax shooterTop = new CANSparkMax(29, MotorType.kBrushless);
    public CANSparkMax shooterBottom = new CANSparkMax(28, MotorType.kBrushless);
    public RelativeEncoder shooterEncoder = shooterTop.getEncoder();


    public Yeeter() {
        shooterBottom.restoreFactoryDefaults();
        shooterTop.restoreFactoryDefaults();
        shooterBottom.follow(shooterTop);
        shooterBottom.setIdleMode(IdleMode.kBrake);
        shooterTop.setIdleMode(IdleMode.kCoast);
    }

    public void SetVoltage(double voltage) {
        shooterTop.setVoltage(voltage);
    }

    public void Stop() {
        shooterTop.stopMotor();
    }

    public double getSpeed(){
        return shooterEncoder.getVelocity();
    }
    
}

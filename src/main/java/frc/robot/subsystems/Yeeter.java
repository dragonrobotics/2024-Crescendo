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
    public CANSparkMax shooterTop = new CANSparkMax(0, MotorType.kBrushless);
    public CANSparkMax shooterBottom = new CANSparkMax(0, MotorType.kBrushless);
    public RelativeEncoder shooterEncoder = shooterTop.getEncoder();
    private DigitalInput beamBreak = new DigitalInput(1);


    public Yeeter() {
        shooterBottom.setInverted(true);
        shooterBottom.follow(shooterTop);
        shooterBottom.setIdleMode(IdleMode.kCoast);
        shooterTop.setIdleMode(IdleMode.kCoast);
    }

    public void SetVoltage(double voltage) {
        shooterTop.setVoltage(voltage);
    }

    public void Stop() {
        shooterTop.stopMotor();
    }

    public boolean HasNote() {
        return beamBreak.get();
    }

    public double getSpeed(){
        return shooterEncoder.getVelocity();
    }
    
}

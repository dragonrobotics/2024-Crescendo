package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Yeeter extends SubsystemBase {
    public CANSparkMax shooterTop = new CANSparkMax(29, MotorType.kBrushless);
    public CANSparkMax shooterBottom = new CANSparkMax(28, MotorType.kBrushless);
    public RelativeEncoder shooterEncoder = shooterTop.getEncoder();
    private DigitalInput beamBreak = new DigitalInput(9);


    public Yeeter() {
        SmartDashboard.putData("Shooter Has Note", beamBreak);
        shooterBottom.restoreFactoryDefaults();
        shooterTop.restoreFactoryDefaults();
        shooterBottom.follow(shooterTop);
        shooterBottom.setIdleMode(IdleMode.kBrake);
        shooterTop.setIdleMode(IdleMode.kCoast);
        shooterTop.burnFlash();
        shooterBottom.burnFlash();
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

    public boolean hasNote(){
        return !beamBreak.get();
    }
    
}

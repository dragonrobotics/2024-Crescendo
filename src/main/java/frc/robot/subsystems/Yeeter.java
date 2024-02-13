package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public CANSparkMax shooterTop = new CANSparkMax(0, MotorType.kBrushless);
    public CANSparkMax shooterBottom = new CANSparkMax(0, MotorType.kBrushless);
    private DigitalInput beamBreak = new DigitalInput(1);


    public Shooter() {
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
    
}

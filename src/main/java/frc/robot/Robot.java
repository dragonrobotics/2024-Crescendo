// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.RobotController;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private CommandXboxController m_controller = new CommandXboxController(0);
  //private Drivetrain m_driveTrain = new Drivetrain();

  // 17, 16
  CANSparkMax idMax = new CANSparkMax(16, MotorType.kBrushless);
  AbsoluteEncoder encoder = idMax.getAbsoluteEncoder(Type.kDutyCycle);

  @Override
  public void robotInit() {
    encoder.setPositionConversionFactor(360);
    encoder.setVelocityConversionFactor(360);

    
    /*m_driveTrain.setDefaultCommand(m_driveTrain.getDriveCommand(() -> {
      return m_controller.getRawAxis(0);
    }, () -> {
      return m_controller.getRawAxis(1);
    }, () -> {
      return m_controller.getRawAxis(4);
    }, true));
*/
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    SysIdRoutine routine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
        (Measure<Voltage> voltage)->{
          idMax.setVoltage(voltage.in(Units.Volts));
        },
        (SysIdRoutineLog log)->{
          log.motor("idMotor")
          .angularPosition(Units.Degrees.of(encoder.getPosition()))
          .angularVelocity(Units.DegreesPerSecond.of(encoder.getVelocity()))
          .voltage(Units.Volts.of(idMax.get() * RobotController.getBatteryVoltage()));
        },
        new SubsystemBase("motor"){}
      )
    );
    routine.quasistatic(SysIdRoutine.Direction.kForward).schedule();
  }

  @Override
  public void testPeriodic() {
  }
}

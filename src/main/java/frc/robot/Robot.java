// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Blank;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExtendoArm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Yeeter;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private CommandXboxController controller = new CommandXboxController(0);
  private Drivetrain m_driveTrain = new Drivetrain();
  private Yeeter yeeter = new Yeeter();
  private Intake intake = new Intake();
  Blank blank = new Blank();

  @Override
  public void robotInit() {

    m_driveTrain.setDefaultCommand(m_driveTrain.getDriveCommand(() -> {
      return controller.getRawAxis(0);
    }, () -> {
      return controller.getRawAxis(1);
    }, () -> {
      return controller.getRawAxis(4);
    }, true));

    controller.back().onTrue(runOnce(()->{m_driveTrain.zero();}, m_driveTrain));
    CANSparkMax ev1 = new CANSparkMax(0, MotorType.kBrushless);
    CANSparkMax ev2 = new CANSparkMax(0, MotorType.kBrushless);
    ev2.follow(ev1, true);
    RelativeEncoder evEnc = ev1.getEncoder();
    CANSparkMax rt1 = new CANSparkMax(0, MotorType.kBrushless);
    CANSparkMax rt2 = new CANSparkMax(0, MotorType.kBrushless);   
    rt2.follow(rt1, true); 
    RelativeEncoder rtEnc = rt1.getEncoder();

    SysIdRoutine elevatorRoutine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism((Measure<Voltage> voltage) -> {
      ev1.set(voltage.in(Units.Volts)/ev1.getBusVoltage());
    }, (SysIdRoutineLog log) -> {
        log.motor("elevator")
          .voltage(Units.Volts.of(ev1.get()*ev1.getBusVoltage()))
          .linearPosition(Units.Meters.of(evEnc.getPosition()))
          .linearVelocity(Units.MetersPerSecond.of(evEnc.getVelocity()));
    }, blank));
    SysIdRoutine rotationRoutine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism((Measure<Voltage> voltage) -> {
       rt1.set(voltage.in(Units.Volts)/rt1.getBusVoltage());
    }, (SysIdRoutineLog log) -> {
        log.motor("rotation")
          .voltage(Units.Volts.of(rt1.get()*rt1.getBusVoltage()))
          .angularPosition(Units.Degrees.of(rtEnc.getPosition()))
          .angularVelocity(Units.DegreesPerSecond.of(rtEnc.getVelocity()));
    }, blank));

    controller.x().whileTrue(elevatorRoutine.dynamic(Direction.kForward).andThen(runOnce(()->{ev1.set(0);})));
    controller.a().whileTrue(elevatorRoutine.dynamic(Direction.kReverse).andThen(runOnce(()->{ev1.set(0);})));
    controller.y().whileTrue(elevatorRoutine.quasistatic(Direction.kForward).andThen(runOnce(()->{ev1.set(0);})));
    controller.b().whileTrue(elevatorRoutine.quasistatic(Direction.kReverse).andThen(runOnce(()->{ev1.set(0);})));

    controller.povLeft().whileTrue(rotationRoutine.dynamic(Direction.kForward).andThen(runOnce(()->{rt1.set(0);})));
    controller.povDown().whileTrue(rotationRoutine.dynamic(Direction.kReverse).andThen(runOnce(()->{rt1.set(0);})));
    controller.povUp().whileTrue(rotationRoutine.quasistatic(Direction.kForward).andThen(runOnce(()->{rt1.set(0);})));
    controller.povRight().whileTrue(rotationRoutine.quasistatic(Direction.kReverse).andThen(runOnce(()->{rt1.set(0);})));


    /*
     * RT: Shoot
     * Bumpers: Intake
     * Y: Arm up
     * A: Arm down
     * DPad-Up: Elevator Up
     * DPad-Down: Elevator Down
     * LT: Automated Climb
     * (Elevator controls to be automated once reasonable)
     */

/*
    controller.leftBumper().whileTrue(intake.intakeNote());

    controller.rightBumper().whileTrue(run(() -> {
      intake.setVoltage(-3);
    }, intake).finallyDo(() -> {
      intake.stopIntake();
    }));
*/
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}

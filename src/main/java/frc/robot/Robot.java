// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.subsystems.Intake.intakePosition;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Robot extends TimedRobot {
  private CommandXboxController controller = new CommandXboxController(0);
  private Drivetrain m_driveTrain = new Drivetrain();
  private Command m_autonomousCommand = m_driveTrain.getDriveCommand(()->{return 0;}, ()->{return .4;}, ()->{return 0;}, true).withTimeout(3);
  private Yeeter yeeter = new Yeeter();
  private Intake intake = new Intake();
  Blank blank = new Blank();

  @Override
  public void robotInit() {

    m_driveTrain.setDefaultCommand(m_driveTrain.getDriveCommand(() -> {
      return -controller.getRawAxis(0);
    }, () -> {
      return -controller.getRawAxis(1);
    }, () -> {
      return controller.getRawAxis(4);
    }, true));
    controller.back().onTrue(runOnce(()->{m_driveTrain.zero();}));
    SmartDashboard.putData(intake);
    SmartDashboard.putData(yeeter);
    controller.rightBumper().whileTrue(intake.intakeNote(controller))
        .onFalse(intake.SetIntakePosition(intakePosition.up));
    
    controller.y().whileTrue(sequence(runOnce(() -> {
      yeeter.SetVoltage(12);
    }), waitSeconds(.5), runOnce(() -> {
      intake.setVoltage(12);
    }), waitSeconds(3)).finallyDo(() -> {
      intake.stopIntake();
      yeeter.Stop();
    }));
    controller.x().onTrue(runOnce(()->{yeeter.SetVoltage(12);})).onFalse(runOnce(()->{yeeter.Stop();}));
    controller.b().onTrue(runOnce(()->{intake.setVoltage(8);})).onFalse(runOnce(()->{intake.setVoltage(0);}));

    // controller.leftBumper().onTrue(intake.SetIntakePosition(intakePosition.down));
    // controller.leftBumper().onTrue(intake.SetIntakePosition(intakePosition.up));

    // controller.leftTrigger().whileTrue(intake.intakeNote());
    // controller.rightTrigger().whileTrue(run(()->{yeeter.setVoltage(12);})).andThen(waitSeconds(1)).andThen(run(()->{intake.setVoltage(12);})).andThen(waitSeconds(3)).finallyDo((boolean
    // interup)->{intake.stop(); shooter.stop();});++

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
     * controller.leftBumper().whileTrue(intake.intakeNote());
     * 
     * controller.rightBumper().whileTrue(run(() -> {
     * intake.setVoltage(-3);
     * }, intake).finallyDo(() -> {
     * intake.stopIntake();
     * }));
     */
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("bb", intake.HasNote());

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

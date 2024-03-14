// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.Blank;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Yeeter;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmRotation;
import frc.robot.subsystems.Intake.intakePosition;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;

public class Robot extends TimedRobot {
  //private ArmExtension armExtension = new ArmExtension();
  private ArmRotation armRotation = new ArmRotation();
  private CommandXboxController controller = new CommandXboxController(0);
  private Drivetrain m_driveTrain = new Drivetrain();
  private Command m_autonomousCommand = either(getShootCommand(), none(), () -> {
    return SmartDashboard.getBoolean("ShouldShoot", false);
  }).andThen(m_driveTrain.getDriveCommand(() -> {
    return 0;
  }, () -> {
    return .4;
  }, () -> {
    return 0;
  }, true)).withTimeout(3);
  private Yeeter yeeter = new Yeeter();
  private Intake intake = new Intake();
  Blank blank = new Blank();

  public Command getShootCommand() {
    return sequence(runOnce(() -> {
      yeeter.SetVoltage(12);
    }), waitSeconds(.5), runOnce(() -> {
      intake.setVoltage(12);
    }), waitSeconds(3)).finallyDo(() -> {
      intake.stopIntake();
      yeeter.Stop();
    });
  }

  @Override
  public void robotInit() {

    configTestControls(controller);

    SmartDashboard.putBoolean("ShouldShoot", true);
    m_driveTrain.setDefaultCommand(m_driveTrain.getDriveCommand(() -> {
      return -controller.getRawAxis(0);
    }, () -> {
      return -controller.getRawAxis(1);
    }, () -> {
      return controller.getRawAxis(4);
    }, true));
    controller.back().onTrue(runOnce(() -> {
      m_driveTrain.zero();
    }));
    SmartDashboard.putData(intake);
    SmartDashboard.putData(yeeter);
    controller.rightBumper().whileTrue(intake.intakeNote(controller))
        .onFalse(intake.SetIntakePosition(intakePosition.up));

    teleop().and(controller.y()).whileTrue(getShootCommand());
    teleop().and(controller.x()).onTrue(runOnce(() -> {
      yeeter.SetVoltage(12);
    })).onFalse(runOnce(() -> {
      yeeter.Stop();
    }));
    teleop().and(controller.b()).onTrue(runOnce(() -> {
      intake.setVoltage(8);
    })).onFalse(runOnce(() -> {
      intake.setVoltage(0);
    }));

    teleop().and(controller.leftBumper()).whileTrue(run(()->{intake.setVoltage(-4);}).finallyDo(()->{intake.stopIntake();}));
    controller.povUp().whileTrue(armRotation.rotateToPosition(100));
    controller.povDown().whileTrue(armRotation.rotateToPosition(0));
  }

  private void configTestControls(CommandXboxController controller){

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
}

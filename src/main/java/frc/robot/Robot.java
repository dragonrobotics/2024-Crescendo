// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Yeeter;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ArmRotation;
import frc.robot.subsystems.ArmExtension.ExtensionPosition;
import frc.robot.subsystems.ArmRotation.RotationAngle;
import frc.robot.subsystems.Intake.intakePosition;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.GenericEntry;

public class Robot extends TimedRobot {
  private ArmExtension armExtension = new ArmExtension();
  private ArmRotation armRotation = new ArmRotation();
  private CommandXboxController controller = new CommandXboxController(0);
  private Drivetrain m_driveTrain = new Drivetrain();
  private Yeeter yeeter = new Yeeter();
  private Intake intake = new Intake();
  private GenericEntry entry = Shuffleboard.getTab("hi").add("Hi", 0).getEntry();

  public Command getShootCommand() {
    return either(
      sequence(runOnce(() -> {
      yeeter.SetVoltage(12);
    }, yeeter), waitSeconds(.5), runOnce(() -> {
      intake.setVoltage(12);
    }, intake), waitSeconds(1))
        .finallyDo(() -> {
          intake.stopIntake();
          yeeter.Stop();
        }),
        sequence(
          runOnce(() -> {
            yeeter.SetVoltage(8);
          }, yeeter),
          waitSeconds(1),
          runOnce(() -> {
            yeeter.Stop();
          }),
          either(
              sequence(armExtension.extendToPosition(ExtensionPosition.In),
                   armRotation.rotateToPosition(RotationAngle.Down)),
              none(), () -> {
                return armRotation.getRotation() == RotationAngle.Amp;
              })),
              ()->{return armRotation.getRotation() == RotationAngle.Down;}
    );
  }

  public Command getHandoffCommand() {

    return either(sequence(
        runOnce(() -> {
          intake.setVoltage(4);
          yeeter.SetVoltage(2);
        }, intake, yeeter),
        waitUntil(() -> {
          return !intake.hasNote();
        }),
        runOnce(() -> {
          intake.stopIntake();
          yeeter.Stop();
        }, intake, yeeter)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming), none(), ()->{return intake.hasNote();});
  }

  @Override
  public void robotInit() {
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

    controller.rightBumper().and(() -> {
      return armRotation.atGoal() && armRotation.getRotation() == RotationAngle.Down;
    }).whileTrue(intake.intakeNote(controller))
        .onFalse(intake.SetIntakePosition(intakePosition.up));

    controller.leftBumper().whileTrue(run(() -> {
      intake.setVoltage(-4);
    }).finallyDo(() -> {
      intake.stopIntake();
    }));

    controller.y().onTrue(getShootCommand());

    controller.b().and(() -> {
      return armRotation.atGoal() && armRotation.getRotation() == ArmRotation.RotationAngle.Down;
    }).onTrue(
        sequence(
            getHandoffCommand(),
            armRotation.rotateToPosition(RotationAngle.Amp),
            armExtension.extendToPosition(ExtensionPosition.Amp)));


    controller.povUp().and(test()).onTrue(
        sequence(
          armExtension.extendToPosition(ExtensionPosition.In),
          waitSeconds(3),
          armRotation.rotateToPosition(RotationAngle.Down)
        )
      );

    controller.povDown().and(()->{
            return armRotation.atGoal() && armRotation.getRotation() == ArmRotation.RotationAngle.Down;
      }).and(test()).onTrue(
        sequence(
          armRotation.rotateToPosition(RotationAngle.Trap),
          armExtension.extendToPosition(ExtensionPosition.Trap),
          getShootCommand()
        )
      );

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void teleopInit() {

  }
}

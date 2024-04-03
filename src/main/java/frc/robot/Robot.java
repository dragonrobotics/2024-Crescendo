// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

import java.io.File;
import java.util.HashMap;
import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

public class Robot extends TimedRobot {
  private ArmExtension armExtension = new ArmExtension();
  private ArmRotation armRotation = new ArmRotation();
  private CommandXboxController controller = new CommandXboxController(0);
  private Drivetrain m_driveTrain = new Drivetrain();
  private Yeeter yeeter = new Yeeter();
  private Intake intake = new Intake();
  // private GenericEntry entry = Shuffleboard.getTab("hi").add("Hi",
  // 0).getEntry();
  private SendableChooser<SendableChooser<PathPlannerPath>> autonSelector = new SendableChooser<>();

  private Command autonCommand = none();
  private Field2d field = new Field2d();

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
            waitSeconds(0.5),
            runOnce(() -> {
              yeeter.Stop();
            }),
            either(
                sequence(armExtension.extendToPosition(ExtensionPosition.In),
                    armRotation.rotateToPosition(RotationAngle.Down)),
                none(), () -> {
                  return armRotation.getRotation() == RotationAngle.Amp;
                })),
        () -> {
          return armRotation.getRotation() == RotationAngle.Down;
        });
  }

  public Command getHandoffCommand() {

    return either(sequence(
        runOnce(() -> {
          intake.setVoltage(8);
          yeeter.SetVoltage(2);
        }, intake, yeeter),
        waitUntil(() -> {
          return !intake.hasNote();
        }),
        runOnce(() -> {
          intake.stopIntake();
          yeeter.Stop();
        }, intake, yeeter)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming), none(), () -> {
          return intake.hasNote();
        });
  }

  @Override
  public void robotInit() {
    configureAuton();

    disabled().onTrue(
        runOnce(() -> {
          armExtension.disable();
          armRotation.disable();
          intake.disable();
        }).ignoringDisable(true))
        .onFalse(runOnce(() -> {
          armExtension.enable();
          armRotation.enable();
          intake.enable();
        }).ignoringDisable(true));

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

    controller.povUp().onTrue(
        sequence(
            runOnce(() -> {
              armExtension.climbSpeed();
              armRotation.climbSpeed();
            }),
            armExtension.extendToPosition(ExtensionPosition.In),
            armRotation.rotateToPosition(RotationAngle.Down)).finallyDo(() -> {
              armExtension.normalSpeed();
              armRotation.normalSpeed();
            }));

    controller.povDown().and(() -> {
      return armRotation.atGoal() && armRotation.getRotation() == ArmRotation.RotationAngle.Down;
    }).onTrue(
        sequence(
            armRotation.rotateToPosition(RotationAngle.Trap),
            armExtension.extendToPosition(ExtensionPosition.Trap),
            getShootCommand()));

    FollowPathCommand.warmupCommand().schedule();

  }

  void configureAuton() {

    NamedCommands.registerCommand("Shoot", new PrintCommand(
        "Hello! ###################################################################################################"));
    SmartDashboard.putData("Auton Field", field);
    Consumer<PathPlannerPath> updateFieldMap = (PathPlannerPath path) -> {
      if (path == null)
        return;
      field.getObject("Path").setPoses(path.getPathPoses());
      field.getObject("Robot").setPose(path.getPreviewStartingHolonomicPose());
    };
    // Load all paths
    HashMap<String, SendableChooser<PathPlannerPath>> positionMap = new HashMap<>();

    SendableChooser<PathPlannerPath> nullChooser = new SendableChooser<>();
    nullChooser.setDefaultOption("None", null);
    autonSelector.setDefaultOption("None", nullChooser);

    File autonFolder = new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/choreo/");
    for (File file : autonFolder.listFiles()) {
      String autoName = file.getName().replace(".traj", "");
      if (autoName.endsWith(".1"))
        continue;
      String[] nameParts = autoName.split(" ", 2);
      if (!positionMap.containsKey(nameParts[0])) {
        positionMap.put(nameParts[0], new SendableChooser<>());
        positionMap.get(nameParts[0]).setDefaultOption("None", null);
        positionMap.get(nameParts[0]).onChange(updateFieldMap);
      }

      positionMap.get(nameParts[0]).addOption(nameParts[1], PathPlannerPath.fromChoreoTrajectory(autoName));
    }

    positionMap.keySet().forEach((String position) -> {
      autonSelector.addOption(position, positionMap.get(position));
    });

    // Put on dashboard
    SmartDashboard.putData("Auton Position Selecter", autonSelector);
    SmartDashboard.putData("Auton Selecter", autonSelector.getSelected());

    autonSelector.onChange((SendableChooser<PathPlannerPath> positionChooser) -> {
      SmartDashboard.putData("Auton Selecter", positionChooser);
    });

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

  }

  @Override
  public void autonomousInit() {
    PathPlannerPath path = autonSelector.getSelected().getSelected();
    if (path == null)
      autonCommand = none();
    else
      autonCommand = AutoBuilder.followPath(autonSelector.getSelected().getSelected());
    autonCommand.schedule();
  }

  @Override
  public void teleopInit() {
    autonCommand.cancel();

  }
}

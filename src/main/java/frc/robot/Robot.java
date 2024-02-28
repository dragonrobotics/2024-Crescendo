// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExtendoArm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Yeeter;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private CommandXboxController controller = new CommandXboxController(0);
  private Drivetrain m_driveTrain = new Drivetrain();
  private ExtendoArm arm = new ExtendoArm();
  private Yeeter yeeter = new Yeeter();
  private Intake intake = new Intake();

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
    controller.leftBumper().and(() -> {
      return !(yeeter.HasNote() || intake.HasNote());
    }).whileTrue(intake.intakeNote());

    controller.rightBumper().whileTrue(run(() -> {
      intake.setVoltage(-3);
    }, intake).finallyDo(() -> {
      intake.stopIntake();
    }));

    controller.b().and(() -> {return arm.isAngleDown();}).onTrue(either(
        runOnce(() -> {
          intake.setVoltage(4);
          yeeter.SetVoltage(4);
        }, intake, yeeter).until(() -> {
          return !intake.HasNote() || yeeter.HasNote();
        }).andThen(runOnce(() -> {
          intake.stopIntake();
          yeeter.Stop();
        }, intake, yeeter)).andThen(arm.SetAngle(90)),
        
        runOnce(() -> {
          arm.setAngle(90);
        }),
        
        () -> {
          return intake.HasNote();
        }));

    controller.y().and(() -> {
      return intake.HasNote() || yeeter.HasNote();
    }).whileTrue(
        either(run(() -> {yeeter.SetVoltage(4);}, yeeter)
        .until(() -> {return yeeter.getSpeed() > 5; /*TODO: test for a better value *//*})
        .andThen(run(() -> {intake.setVoltage(4);}, intake))
        .until(() -> {return !yeeter.HasNote() && !intake.HasNote();})
        .andThen(run(() -> {yeeter.Stop(); intake.stopIntake();}, intake, yeeter)), 
        }, yeeter).until(() -> {
          return !yeeter.HasNote();
        }).andThen(waitSeconds(.2)).andThen(runOnce(() -> {
          yeeter.Stop(); // TODO: Make arm go down if not doing trap.
        })),
        
        () -> {
          return intake.HasNote();
        }));

 */    // TODO: Climbing controls
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

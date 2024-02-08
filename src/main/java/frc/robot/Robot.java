// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private CommandXboxController m_controller = new CommandXboxController(0);
  private Drivetrain m_driveTrain = new Drivetrain();

  @Override
  public void robotInit() {

    m_driveTrain.setDefaultCommand(m_driveTrain.getDriveCommand(() -> {
      return m_controller.getRawAxis(0);
    }, () -> {
      return m_controller.getRawAxis(1);
    }, () -> {
      return m_controller.getRawAxis(4);
    }, true));

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

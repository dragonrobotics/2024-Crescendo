package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static java.lang.Math.signum;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class Drivetrain extends SubsystemBase {
    private final double maximumSpeed = 3.66;
    private final double maxRotationalSpeed = 3;
    public SwerveDrive swerveDrive;

    public Drivetrain() {
        swervelib.telemetry.SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
        } catch (Exception e) {
            e.printStackTrace();
            System.exit(1);
        }
        swerveDrive.headingCorrection = true;
        zero();

        AutoBuilder.configureHolonomic(() -> swerveDrive.getPose(), (Pose2d pose) -> swerveDrive.resetOdometry(pose),
                () -> swerveDrive.getRobotVelocity(), (ChassisSpeeds speeds) -> swerveDrive.setChassisSpeeds(speeds),
                new HolonomicPathFollowerConfig(
                    new PIDConstants(0),
                    new PIDConstants(0),  
                maximumSpeed, 2, 
                new ReplanningConfig()), () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, this);
    }
    
    public Command getDriveCommand(DoubleSupplier translationY, DoubleSupplier translationX,
    DoubleSupplier angularRotation, boolean fieldRelative) {
        return run(() -> {
            double xSpeed = translationX.getAsDouble();
            double ySpeed = translationY.getAsDouble();
            double rSpeed = angularRotation.getAsDouble();
            xSpeed = xSpeed * xSpeed * signum(xSpeed) * maximumSpeed;
            ySpeed = ySpeed * ySpeed * signum(ySpeed) * maximumSpeed;
            rSpeed = rSpeed * rSpeed * -signum(rSpeed) * maxRotationalSpeed;
            swerveDrive.drive(new Translation2d(xSpeed, ySpeed), rSpeed, fieldRelative, false);
        });
    }

    public void zero() {
        swerveDrive.zeroGyro();
    }
}

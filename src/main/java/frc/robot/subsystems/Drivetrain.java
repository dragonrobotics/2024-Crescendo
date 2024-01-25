package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static java.lang.Math.signum;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;

public class Drivetrain extends SubsystemBase {
    private final double maximumSpeed = 4.5;
    private SwerveDrive swerveDrive;

    public Drivetrain() {
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
        } catch (Exception e) {
            e.printStackTrace();
            System.exit(1);
        }
    }

    public Command getDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotation, boolean fieldRelative) {
        return run(() -> {
            double xSpeed = translationX.getAsDouble();
            double ySpeed = translationY.getAsDouble();
            double rSpeed = angularRotation.getAsDouble();
            xSpeed = xSpeed * xSpeed * signum(xSpeed);
            ySpeed = ySpeed * ySpeed * signum(ySpeed);
            rSpeed = rSpeed * xSpeed * signum(rSpeed);
            swerveDrive.drive(new Translation2d(xSpeed, ySpeed), rSpeed, fieldRelative, false);
        });
    }

    public void setDefaultCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotation, boolean fieldRelative) {

        super.setDefaultCommand(getDriveCommand(null, null, null, false));
    }
}

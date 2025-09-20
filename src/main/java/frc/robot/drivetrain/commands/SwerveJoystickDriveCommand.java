package frc.robot.drivetrain.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.drivetrain.SwerveSubsystem;

public class SwerveJoystickDriveCommand extends Command {
    private final SwerveSubsystem drive;
    private final Supplier<Double> xIn, yIn, rotIn;
    private final boolean useDeadzone;

    public SwerveJoystickDriveCommand(
            SwerveSubsystem drive,
            Supplier<Double> xSupplier,
            Supplier<Double> ySupplier,
            Supplier<Double> rotSupplier,
            boolean useDeadzone) {

        this.drive = drive;
        this.xIn = xSupplier;
        this.yIn = ySupplier;
        this.rotIn = rotSupplier;
        this.useDeadzone = useDeadzone;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double x = xIn.get();
        double y = yIn.get();
        double r = rotIn.get();

        if (useDeadzone) {
            x = applyDeadzone(x, DriveConstants.JOYDeadzone_X);
            y = applyDeadzone(y, DriveConstants.JOYDeadzone_Y);
            r = applyDeadzone(r, DriveConstants.JOYDeadzone_Rot);
        }

        Translation3d request = new Translation3d(x, y, r);
        drive.setDriveRequest(request);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.printf("SwerveJoystickDriveCommand finished (interrupted=%b)%n", interrupted);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private static double applyDeadzone(double v, double Deadzone) {
        if (Deadzone <= 0.0) return v;
        double av = Math.abs(v);
        return (av > 0.0 && av <= Deadzone) ? 0.0 : v;
    }
}
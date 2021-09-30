package frc.robot.commands.background;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.config.SwerveDriveConfig;
import frc.robot.subsystems.SwerveDrivetrain;
import static frc.robot.OI.*;

import org.strykeforce.thirdcoast.util.ExpoScale;

public class SwerveDriveCommand extends CommandBase {
    private final SwerveDrivetrain DRIVE = Robot.sSwerveDriveTrain;

    private static final double FORWARD_DEADBAND = 0.05;
    private static final double STRAFE_DEADBAND = 0.05;
    private static final double YAW_DEADBAND = 0.01;

    private static final double FORWARD_XPOSCALE = 0.6;
    private static final double STRAFE_XPOSCALE = 0.6;
    private static final double YAW_XPOSCALE = 0.75;

    private final ExpoScale forwardScale;
    private final ExpoScale strafeScale;
    private final ExpoScale yawScale;

    public SwerveDriveCommand() {
        addRequirements(DRIVE);

        forwardScale = new ExpoScale(FORWARD_DEADBAND, FORWARD_XPOSCALE);
        strafeScale = new ExpoScale(STRAFE_DEADBAND, STRAFE_XPOSCALE);
        yawScale = new ExpoScale(YAW_DEADBAND, YAW_XPOSCALE);
    }

    @Override
    public void execute(){
        double forward = forwardScale.apply(sLeftJoystick.getY());
        double strafe = strafeScale.apply(sLeftJoystick.getX());
        double yaw = yawScale.apply(sRightJoystick.getX());
        double vx = forward * SwerveDriveConfig.kMaxSpeedMetersPerSecond;
        double vy = strafe * SwerveDriveConfig.kMaxSpeedMetersPerSecond;
        double omega = yaw * SwerveDriveConfig.kMaxOmega;

        DRIVE.drive(vx, vy, omega);
    }

    @Override
    public void end(boolean interrupted) {
        DRIVE.drive(0, 0, 0);
    }
}

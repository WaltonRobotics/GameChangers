package frc.robot.commands.auton.shootingChallenges;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.RamseteTrackingCommand;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static frc.robot.Constants.DriverPreferences.kDriveJoystickDeadband;
import static frc.robot.OI.sLeftJoystick;
import static frc.robot.OI.sRightJoystick;

public class ManualOverrideTrackingCommand extends SequentialCommandGroup {

    private final BooleanSupplier manualOverrideSupplier =
            () -> (Math.abs(sLeftJoystick.getY()) + Math.abs(sRightJoystick.getY())) / 2 > kDriveJoystickDeadband;

    public ManualOverrideTrackingCommand(Trajectory trajectory, boolean useSparkPID, boolean disableRamsete) {
        addCommands(
                new RamseteTrackingCommand(trajectory, useSparkPID, disableRamsete)
                        .withInterrupt(manualOverrideSupplier)
        );
    }

    public ManualOverrideTrackingCommand(Supplier<Trajectory> trajectorySupplier, boolean useSparkPID,
                                         boolean disableRamsete) {
        addCommands(
                new RamseteTrackingCommand(trajectorySupplier, useSparkPID, disableRamsete)
                        .withInterrupt(manualOverrideSupplier)
        );
    }

}

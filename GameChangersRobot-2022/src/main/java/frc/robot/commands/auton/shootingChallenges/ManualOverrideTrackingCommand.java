package frc.robot.commands.auton.shootingChallenges;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.RamseteTrackingCommand;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static frc.robot.Constants.DriverPreferences.kDriveDeadband;
import static frc.robot.OI.sDriveGamepad;

public class ManualOverrideTrackingCommand extends SequentialCommandGroup {

    private final BooleanSupplier manualOverrideSupplier =
            () -> (Math.abs(sDriveGamepad.getY()) + Math.abs(sDriveGamepad.getRawAxis(3))) / 2 > kDriveDeadband;

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

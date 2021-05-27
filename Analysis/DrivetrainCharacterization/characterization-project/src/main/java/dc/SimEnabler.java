package dc;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class SimEnabler implements Sendable {

    public SimEnabler() {
        DriverStationSim.setAutonomous(true);
    }

    public void setEnabled(boolean enabled) {
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
        DriverStation.getInstance().isNewControlData();
        while (DriverStation.getInstance().isEnabled() != enabled) {
            try {
                Thread.sleep(1);
            } catch (InterruptedException exception) {
                exception.printStackTrace();
            }
        }
    }

    @Override
    public String getName() {
        return "SimEnabler";
    }

    @Override
    public void setName(String name) {
    }

    @Override
    public String getSubsystem() {
        return "";
    }

    @Override
    public void setSubsystem(String subsystem) {
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Enabled",
                () -> DriverStation.getInstance().isEnabled(),
                enabled -> setEnabled(enabled));
    }
}

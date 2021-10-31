package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CANBusIDs.kClimberID;
import static frc.robot.Constants.PneumaticsIDs.kClimberDeploySolenoidID;
import static frc.robot.Constants.PneumaticsIDs.kClimberLockSolenoidID;
import static frc.robot.Constants.SmartDashboardKeys.kClimberIsDeployedKey;
import static frc.robot.Constants.SmartDashboardKeys.kClimberIsUnlockedKey;

public class Climber extends SubsystemBase {

    private final TalonFX mClimberController = new TalonFX(kClimberID);

    private final Solenoid mLockSolenoid = new Solenoid(kClimberLockSolenoidID);
    private final Solenoid mDeploySolenoid = new Solenoid(kClimberDeploySolenoidID);

    public Climber() {
        mClimberController.selectProfileSlot(0, 0);
        mClimberController.setNeutralMode(NeutralMode.Coast);
        mClimberController.setInverted(true);
        mClimberController.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
                true, 25, 30, 1));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean(kClimberIsUnlockedKey, isClimberUnlocked());
        SmartDashboard.putBoolean(kClimberIsDeployedKey, isClimberDeployed());
    }

    public boolean isClimberUnlocked() {
        return mLockSolenoid.get();
    }

    public void setClimberUnlocked(boolean isUnlocked) {
        mLockSolenoid.set(isUnlocked);
    }

    public void toggleClimberLock() {
        mLockSolenoid.toggle();
    }

    public boolean isClimberDeployed() {
        return mDeploySolenoid.get();
    }

    public void setClimberDeployed(boolean isDeployed) {
        mDeploySolenoid.set(isDeployed);
    }

    public void toggleClimberDeployed() {
        mDeploySolenoid.toggle();
    }

    public void setClimberControllerOutput(TalonFXControlMode controlMode, double output) {
        mClimberController.set(controlMode, output);
    }

}

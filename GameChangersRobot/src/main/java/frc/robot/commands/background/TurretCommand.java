package frc.robot.commands.background;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.stateMachine.IState;
import frc.robot.stateMachine.StateMachine;

import static frc.robot.Robot.sTurret;

public class TurretCommand extends CommandBase {

    private IState mIdle;
    private IState mZeroing;
    private IState mHoming;
    private IState mAligningRobotRelative;
    private IState mAligningFieldRelative;

    private StateMachine mStateMachine;

    private boolean mHasZeroed;

    public TurretCommand() {
        addRequirements(sTurret);

        mHasZeroed = false;
    }

    @Override
    public void execute() {
//        sTurret.setOpenLoopDutyCycle(sGamepad.getLeftX());

        if (sTurret.isForwardLimitRisingEdge()) {
            sTurret.zero();
            mHasZeroed = true;
        }
    }
}

package frc.robot.utils;

import edu.wpi.first.wpilibj.geometry.Pose2d;

import java.util.function.BooleanSupplier;

import static frc.robot.Robot.sDrivetrain;

public class HomingSupplier implements BooleanSupplier {

    private final Pose2d mFinalPose;
    private final double mMinimumRadius;
    private final double mMaximumRadius;

    public HomingSupplier(Pose2d finalPose, double minimumRadius, double maximumRadius) {
        this.mFinalPose = finalPose;
        this.mMinimumRadius = minimumRadius;
        this.mMaximumRadius = maximumRadius;
    }

    @Override
    public boolean getAsBoolean() {
        Pose2d robotPose = sDrivetrain.getCurrentPose();
        double distanceToFinalPose = robotPose.getTranslation().getDistance(mFinalPose.getTranslation());

        return robotPose.getX() < mFinalPose.getX()
                && distanceToFinalPose >= mMinimumRadius
                && distanceToFinalPose <= mMaximumRadius;
    }

}

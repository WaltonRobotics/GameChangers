package frc.robot.robots;

/**
 * Uses the DIO ports on the rio to identify the current robot.
 */

public enum RobotIdentification {

    COMP_DEEP_SPACE(true, false, new CompDeepSpace()),
    PRACTICE_GAME_CHANGERS(false, true, new PracticeGameChangers()),
    COMP_GAME_CHANGERS(false, false, new CompGameChangers());

    boolean input1;
    boolean input2;

    WaltRobot currentRobot;

    RobotIdentification(boolean input1, boolean input2, WaltRobot robot) {
        this.input1 = input1;
        this.input2 = input2;
        this.currentRobot = robot;
    }

    public static RobotIdentification findByInputs(boolean input1, boolean input2) {
        for (RobotIdentification i : values()) {
            if (i.input1 == input1 && i.input2 == input2) {
                return i;
            }
        }
        return COMP_GAME_CHANGERS;
    }

    public WaltRobot getCurrentRobot() {
        return currentRobot;
    }
}
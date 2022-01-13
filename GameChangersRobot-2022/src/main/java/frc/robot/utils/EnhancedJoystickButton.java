package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;

import java.util.function.IntSupplier;
import java.util.function.Supplier;

/**
 * An EnhancedJoystickButton is a Button with rising and falling edge getter methods and
 * implementation to have a dynamic mapping of index and of controller. Use the public constants to
 * make it a POV button.
 *
 * @author Russell Newton, Walton Robotics
 */
public class EnhancedJoystickButton extends Button {

    public static final int UNBOUND = -1;
    public static final int POV_N = -2;
    public static final int POV_NE = -3;
    public static final int POV_E = -4;
    public static final int POV_SE = -5;
    public static final int POV_S = -6;
    public static final int POV_SW = -7;
    public static final int POV_W = -8;
    public static final int POV_NW = -9;

    private boolean mIsRisingEdge;
    private boolean mIsFallingEdge;

    /**
     * Create a dynamic EnhancedJoystickButton with an EnhancedButtonIndex.
     */
    public EnhancedJoystickButton(EnhancedButtonIndex buttonIndex) {
        // The default for get() is based on a BooleanSupplier passed into the Button constructor.
        // Because of this, we can use the EnhancedButtonIndex get() instead of overriding it.
        super(buttonIndex::get);

        CommandScheduler.getInstance().addButton(
                new Runnable() {
                    private boolean mPreviousState = get();

                    @Override
                    public void run() {
                        boolean currentState = get();

                        mIsRisingEdge = currentState && !mPreviousState;
                        mIsFallingEdge = !currentState && mPreviousState;

                        mPreviousState = currentState;
                    }
                }
        );
    }

    /**
     * Create a EnhancedJoystickButton at {@code buttonNumber} on {@code joystick}.
     */
    public EnhancedJoystickButton(Joystick joystick, int buttonNumber) {
        this(new EnhancedButtonIndex(() -> joystick, () -> buttonNumber));
    }

    /**
     * Returns true if the button has just been pressed.
     */
    public boolean isRisingEdge() {
        return mIsRisingEdge;
    }

    /**
     * Returns true if the button has just been released.
     */
    public boolean isFallingEdge() {
        return mIsFallingEdge;
    }

    /**
     * An EnhancedButtonIndex wraps together a GenericHID (Joysticks) supplier and a button index
     * supplier. These suppliers can point to dynamic values, so that an EnhancedJoystickButton can be
     * moved from index to index and controller to controller.
     */
    public static class EnhancedButtonIndex {

        private final Supplier<GenericHID> joystickSupplier;
        private final IntSupplier indexSupplier;

        /**
         * Useful for dynamic mappings.
         */
        public EnhancedButtonIndex(Supplier<GenericHID> joystickSupplier, IntSupplier indexSupplier) {
            this.joystickSupplier = joystickSupplier;
            this.indexSupplier = indexSupplier;
        }

        /**
         * Useful for default mappings.
         */
        public EnhancedButtonIndex(GenericHID joystick, int index) {
            this(() -> joystick, () -> index);
        }

        public boolean get() {
            // Unbound
            if (indexSupplier.getAsInt() == 0 || indexSupplier.getAsInt() == -1) {
                return false;
            }

            // If POV_N:  (-2 + 2) * -45 = 0
            // If POV_NE: (-3 + 2) * -45 = 45
            // If POV_E:  (-4 + 2) * -45 = 90
            // etc...
            if (indexSupplier.getAsInt() < 0) {
                return joystickSupplier.get().getPOV() == (indexSupplier.getAsInt() + 2) * -45;
            }

            return joystickSupplier.get().getRawButton(indexSupplier.getAsInt());
        }

        public GenericHID getJoystick() {
            return joystickSupplier.get();
        }

        public int getIndex() {
            return indexSupplier.getAsInt();
        }
    }

}
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DebuggingLog;

import java.util.logging.Level;

import static frc.robot.Constants.ProMicro.kSerialPortBaudRate;
import static frc.robot.Constants.ProMicro.kUpdateRateSeconds;
import static frc.robot.Constants.SmartDashboardKeys.kProMicroLEDWriteMessageKey;
import static frc.robot.Constants.SmartDashboardKeys.kProMicroPixyCamReadMessageKey;

public class ProMicro extends SubsystemBase {

    private SerialPort mSerialPort;

    private PixyCamReadMessage mCurrentPixyCamReadMessage;
    private LEDStripWriteMessage mCurrentLEDStripWriteMessage;

    public enum PixyCamReadMessage {
        NO_DETERMINATION((byte)0x0A),
        GALACTIC_SEARCH_RED_A((byte)0x0B),
        GALACTIC_SEARCH_RED_B((byte)0x0C),
        GALACTIC_SEARCH_BLUE_A((byte)0x0D),
        GALACTIC_SEARCH_BLUE_B((byte)0x0E);

        private final byte mMessageByte;

        PixyCamReadMessage(byte messageByte) {
            this.mMessageByte = messageByte;
        }

        public static PixyCamReadMessage findByMessageByte(byte readByte) {
            for (PixyCamReadMessage state : values()) {
                if (state.getMessageByte() == readByte) {
                    return state;
                }
            }

            return NO_DETERMINATION;
        }

        public byte getMessageByte() {
            return mMessageByte;
        }
    }

    public enum LEDStripWriteMessage {
        IDLE((byte)0x00),
        TURN_LEFT_RANGE_BLINKING((byte)0x1A),
        TURN_RIGHT_RANGE_BLINKING((byte)0x1B),
        TURN_LEFT((byte)0x1C),
        TURN_RIGHT((byte)0x1D),
        ALIGNED_RANGE_BLINKING((byte)0x1E),
        ALIGNED_AND_IN_RANGE((byte)0x1F);

        private final byte mMessageByte;

        LEDStripWriteMessage(byte messageByte) {
            this.mMessageByte = messageByte;
        }

        public byte getMessageByte() {
            return mMessageByte;
        }
    }

    public ProMicro() {
        try {
            mSerialPort = new SerialPort(kSerialPortBaudRate, SerialPort.Port.kUSB1);
        } catch(Exception e) {
            try {
                DebuggingLog.getInstance().getLogger().log(Level.WARNING,
                        "Serial port connection on USB 1 port failed. Falling back to USB 2 port");

                mSerialPort = new SerialPort(kSerialPortBaudRate, SerialPort.Port.kUSB2);
            } catch(Exception e1) {
                DebuggingLog.getInstance().getLogger().log(Level.SEVERE,
                        "Serial port connection failed on both ports");
            }
        }

        mCurrentPixyCamReadMessage = PixyCamReadMessage.NO_DETERMINATION;
        mCurrentLEDStripWriteMessage = LEDStripWriteMessage.IDLE;

        Notifier updateNotifier = new Notifier(this::update);
        updateNotifier.startPeriodic(kUpdateRateSeconds);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString(kProMicroPixyCamReadMessageKey, mCurrentPixyCamReadMessage.name());
        SmartDashboard.putString(kProMicroLEDWriteMessageKey, mCurrentLEDStripWriteMessage.name());
    }

    public void setLEDStripMessage(LEDStripWriteMessage state) {
        mCurrentLEDStripWriteMessage = state;
    }

    public PixyCamReadMessage getPixyCamDetermination() {
        return mCurrentPixyCamReadMessage;
    }

    private void update() {
        if (mSerialPort != null) {
            mSerialPort.write(new byte[] {mCurrentLEDStripWriteMessage.getMessageByte()}, 1);

            if (mSerialPort.getBytesReceived() > 0) {
//                DebuggingLog.getInstance().getLogger().log(Level.FINE,
//                        "Received byte " + mSerialPort.read(1)[0] + "" + " from Pro Micro");
                mCurrentPixyCamReadMessage = PixyCamReadMessage.findByMessageByte(mSerialPort.read(1)[0]);
            }
        }
    }

}

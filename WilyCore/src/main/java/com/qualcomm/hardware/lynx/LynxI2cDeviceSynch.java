package com.qualcomm.hardware.lynx;

import android.content.Context;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.TimestampedData;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.concurrent.BlockingQueue;
import java.util.function.Supplier;

public class LynxI2cDeviceSynch {
    protected LynxI2cDeviceSynch(final Context context, final LynxModule module, int bus) {}

    //----------------------------------------------------------------------------------------------
    // HardwareDevice
    //----------------------------------------------------------------------------------------------

    public String getDeviceName()
    {
        return "";
    }

    public String getConnectionInfo()
    {
        return "";
    }

    public void resetDeviceConfigurationForOpMode() {}

    public void close() {}
    public boolean isArmed() { return true; }
    public void setI2cAddress(I2cAddr i2cAddr) {}
    public void setI2cAddr(I2cAddr i2cAddr) {}
    public I2cAddr getI2cAddress() { return new I2cAddr(0); }
    public I2cAddr getI2cAddr() { return getI2cAddress(); }
    public void setUserConfiguredName(@Nullable String name) {}
    @Nullable public String getUserConfiguredName() { return ""; }
    public void setLogging(boolean enabled) {}
    public boolean getLogging() { return false; }
    public void setLoggingTag(String loggingTag) {}
    public String getLoggingTag() { return ""; }
    public void setHistoryQueueCapacity(int capacity) {}
    public int getHistoryQueueCapacity() { return 0; }
    // public BlockingQueue<TimestampedI2cData> getHistoryQueue()
    public byte[] read(int ireg, int creg) { return new byte[]{}; }
    public synchronized byte read8(final int ireg) { return 0; }
    // public abstract TimestampedData readTimeStamped(final int ireg, final int creg);
    public  byte[] read(int creg) { return new byte[]{}; }
    public synchronized byte read8() { return 0; }
    public synchronized TimestampedData readTimeStamped(final int creg) { return null; }
    public void write(int ireg, byte[] data) {}
    public synchronized void write8(int ireg, int bVal) {}
    public synchronized void write8(int ireg, int bVal, I2cWaitControl waitControl) {}
    public synchronized void write(int ireg, byte[] data, I2cWaitControl waitControl) {}
    private void internalWrite(int ireg, byte[] data, final I2cWaitControl waitControl) {}
    public synchronized void write(byte[] data) {}
    public synchronized void write(byte[] data, I2cWaitControl waitControl) {}
    public synchronized void write8(int bVal) {}
    public synchronized void write8(int bVal, I2cWaitControl waitControl) {}
    private void internalWrite(final byte[] payload, final I2cWaitControl waitControl) {}
    public synchronized void waitForWriteCompletions(final I2cWaitControl waitControl) {}
    public void enableWriteCoalescing(boolean enable) {}
    public boolean isWriteCoalescingEnabled() { return true; }
//    //----------------------------------------------------------------------------------------------
//    // I2cDeviceSynch API support methods
//    //----------------------------------------------------------------------------------------------
//
//    /**
//     * Waits for the current I2C transaction to finish, then executes the supplied transaction.
//     * <p>
//     * Only intended for use with I2C commands that directly cause the firmware to perform an I2C
//     * transaction.
//     *
//     * @param transactionSupplier A {@link Supplier} that provides a new {@link LynxCommand} every
//     *                            time it's called. The command must perform an on-the-wire I2C
//     *                            transaction.
//     */
//    protected void sendI2cTransaction(Supplier<? extends LynxCommand<?>> transactionSupplier) throws LynxNackException, InterruptedException, RobotCoreException
//    {
//        for (;;)
//        {
//            try {
//                transactionSupplier.get().send();
//                break;
//            }
//            catch (LynxNackException e)
//            {
//                switch (e.getNack().getNackReasonCodeAsEnum())
//                {
//                    case I2C_MASTER_BUSY:
//                    case I2C_OPERATION_IN_PROGRESS:
//                        break;
//                    default:
//                        throw e;
//                }
//            }
//        }
//    }
//
//    protected <T> T acquireI2cLockWhile(Supplier<T> supplier) throws InterruptedException, RobotCoreException, LynxNackException
//    {
//        return this.getModule().acquireI2cLockWhile(supplier);
//    }
//
//    protected void internalWaitForWriteCompletions(com.qualcomm.robotcore.hardware.I2cWaitControl
//    waitControl)
//    {
//        /* Note: called with i2c lock held!
//         *
//         * For {@link I2cWaitControl#NONE} and {@link I2cWaitControl#ATOMIC}, we have nothing to do
//         * because we transmit synchronously to USB in the original write call. For
//         * {@link I2cWaitControl#WRITTEN}, we have work to do.
//         */
//        if (waitControl == com.qualcomm.robotcore.hardware.I2cWaitControl.WRITTEN)
//        {
//            boolean keepTrying = true;
//            while (keepTrying)
//            {
//                final LynxI2cWriteStatusQueryCommand writeStatus = new LynxI2cWriteStatusQueryCommand(this.getModule(), this.bus);
//                try {
//                    LynxI2cWriteStatusQueryResponse response = writeStatus.sendReceive();
//                    if (response.isStatusOk())
//                    {
//                        I2cWarningManager.removeProblemI2cDevice(this);
//                    }
//                    else
//                    {
//                        I2cWarningManager.notifyProblemI2cDevice(this);
//                    }
//                    // Receiving a query response instead of an I2C_OPERATION_IN_PROGRESS nack means
//                    // that the write either failed or was completed, so we can go ahead and exit.
//                    return;
//                }
//                catch (LynxNackException e)
//                {
//                    switch (e.getNack().getNackReasonCodeAsEnum())
//                    {
//                        case I2C_NO_RESULTS_PENDING:
//                            return;
//                        case I2C_OPERATION_IN_PROGRESS:
//                            continue;
//                        default:
//                            handleException(e);
//                            keepTrying = false;
//                            break;
//                    }
//                }
//                catch (InterruptedException|RuntimeException e)
//                {
//                    handleException(e);
//                    keepTrying = false;
//                }
//            }
//        }
//    }
//
//    protected TimestampedData pollForReadResult(I2cAddr i2cAddr, int ireg, int creg)
//    {
//        // Poll until the data is available
//        boolean keepTrying = true;
//
//        while (keepTrying)
//        {
//            LynxI2cReadStatusQueryCommand readStatus = new LynxI2cReadStatusQueryCommand(this.getModule(), this.bus, creg);
//            try {
//                LynxI2cReadStatusQueryResponse response = readStatus.sendReceive();
//                long now = System.nanoTime();
//                response.logResponse();
//                //
//                TimestampedI2cData result = new TimestampedI2cData();
//                result.data = response.getBytes();
//                result.nanoTime = response.getPayloadTimeWindow().isCleared() ? now : response.getPayloadTimeWindow().getNanosecondsLast();
//                result.i2cAddr = i2cAddr;
//                result.register = ireg;
//
//                // Return real data if we've got it
//                if (result.data.length == creg)
//                {
//                    readStatusQueryPlaceholder.reset();
//                    readHistory.addToHistoryQueue(result);
//                    I2cWarningManager.removeProblemI2cDevice(this);
//                    return result;
//                }
//
//                // Log the error, alert the user, and return placeholder data if we don't
//                RobotLog.ee(loggingTag, "readStatusQuery: cbExpected=%d cbRead=%d", creg, result.data.length);
//                I2cWarningManager.notifyProblemI2cDevice(this);
//                keepTrying = false;
//            }
//            catch (LynxNackException e)
//            {
//                switch (e.getNack().getNackReasonCodeAsEnum())
//                {
//                    case I2C_MASTER_BUSY:               // TODO: REVIEW: is this ever actually returned in this situation?
//                    case I2C_OPERATION_IN_PROGRESS:
//                        continue;
//                    case I2C_NO_RESULTS_PENDING:
//                        // This is an internal error of some sort
//                        handleException(e);
//                        keepTrying = false;
//                        I2cWarningManager.notifyProblemI2cDevice(this);
//                        break;
//                    default:
//                        handleException(e);
//                        keepTrying = false;
//                        I2cWarningManager.notifyProblemI2cDevice(this);
//                        break;
//                }
//            }
//            catch (InterruptedException|RuntimeException e)
//            {
//                handleException(e);
//                keepTrying = false;
//            }
//        }
//        return readStatusQueryPlaceholder.log(TimestampedI2cData.makeFakeData(i2cAddr, ireg, creg));
//    }
//
//    //----------------------------------------------------------------------------------------------
//    // Miscellaneous methods
//    //----------------------------------------------------------------------------------------------
//
//    /**
//     * Lynx I2C bus speed.
//     */
//    public enum BusSpeed
//    {
//        STANDARD_100K
//                {
//                    @Override
//                    protected LynxI2cConfigureChannelCommand.SpeedCode toSpeedCode()
//                    {
//                        return LynxI2cConfigureChannelCommand.SpeedCode.STANDARD_100K;
//                    }
//                },
//        FAST_400K
//                {
//                    @Override
//                    protected LynxI2cConfigureChannelCommand.SpeedCode toSpeedCode()
//                    {
//                        return LynxI2cConfigureChannelCommand.SpeedCode.FAST_400K;
//                    }
//                };
//
//        protected LynxI2cConfigureChannelCommand.SpeedCode toSpeedCode()
//        {
//            throw new AbstractMethodError();
//        }
//    }
//
//    /**
//     * Sets the bus speed.
//     * @param speed new bus speed
//     */
//    public void setBusSpeed(BusSpeed speed)
//    {
//        LynxI2cConfigureChannelCommand command = new LynxI2cConfigureChannelCommand(this.getModule(), bus, speed.toSpeedCode());
//        try
//        {
//            command.send();
//        }
//        catch (InterruptedException|RuntimeException|LynxNackException e)
//        {
//            handleException(e);
//        }
//    }
}

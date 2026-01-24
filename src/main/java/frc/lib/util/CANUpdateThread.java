package frc.lib.util;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.Callable;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CompletionException;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import au.grapplerobotics.ConfigurationFailedException;

public class CANUpdateThread implements AutoCloseable {

    private static final int MAX_RETRIES = 5;

    private final BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
    private final ThreadPoolExecutor executor =
        new ThreadPoolExecutor(1, 1, 5, TimeUnit.MILLISECONDS, queue);

    /**
     * Attempts a CTRE status-returning action up to MAX_RETRIES times.
     */
    public CompletableFuture<Void> CTRECheckErrorAndRetry(
        Supplier<StatusCode> action)
    {
        return CompletableFuture.runAsync(() -> {
            StatusCode lastStatus = StatusCode.OK;

            for (int i = 0; i < MAX_RETRIES; i++) {
                lastStatus = action.get();
                if (lastStatus.isOK()) {
                    return;
                }
            }

            throw new RuntimeException("CTRE config failed: " + lastStatus);
        }, executor);
    }

    /**
     * Attempts a LaserCAN configuration action up to MAX_RETRIES times.
     */
    public CompletableFuture<Void> LaserCANCheckErrorAndRetry(
        Callable<ConfigurationFailedException> action)
    {
        return CompletableFuture.runAsync(() -> {
            ConfigurationFailedException lastException = null;

            for (int i = 0; i < MAX_RETRIES; i++) {
                try {
                    action.call();
                    return; // success
                } catch (ConfigurationFailedException e) {
                    lastException = e;
                } catch (Exception e) {
                    throw new CompletionException(e);
                }
            }

            throw new CompletionException(lastException);
        }, executor);
    }

    @Override
    public void close()
    {
        executor.shutdownNow();
    }
}

/*
 * Copyright (C) 2026 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */

package frc.lib.util;

import java.util.concurrent.BlockingQueue;
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
    public CompletableFuture<Void> laserCANCheckErrorAndRetry(
        ThrowingRunnable<ConfigurationFailedException> action)
    {
        return CompletableFuture.runAsync(() -> {
            ConfigurationFailedException lastException = null;

            for (int i = 0; i < MAX_RETRIES; i++) {
                try {
                    action.run();
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

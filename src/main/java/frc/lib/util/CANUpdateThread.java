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
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.function.Supplier;
import com.ctre.phoenix6.StatusCode;
import frc.lib.util.LaserCANConfigurator.ConfigurationStatus;

public class CANUpdateThread implements AutoCloseable {
    // Executor for retrying config operations asynchronously
    private BlockingQueue<Runnable> queue = new LinkedBlockingQueue<>();
    private ThreadPoolExecutor threadPoolExecutor = new ThreadPoolExecutor(1, 1, 5,
        java.util.concurrent.TimeUnit.MILLISECONDS, queue);

    /**
     * Attempts a CTRE action up to 5 times until it succeeds.
     *
     * @param action The status-returning operation to retry.
     */
    @SuppressWarnings("FutureReturnValueIgnored")
    public void CTRECheckErrorAndRetry(Supplier<StatusCode> action)
    {
        threadPoolExecutor.submit(() -> {
            for (int i = 0; i < 5; i++) {
                StatusCode result = action.get();
                if (result.isOK()) {
                    break;
                }
            }
        });
    }

    @SuppressWarnings("FutureReturnValueIgnored")
    public void LaserCANCheckErrorAndRetry(Supplier<ConfigurationStatus> action)
    {
        threadPoolExecutor.submit(() -> {
            for (int i = 0; i < 5; i++) {
                ConfigurationStatus result = action.get();
                if (result == ConfigurationStatus.SUCCESS) {
                    break;
                }
            }
        });
    }

    @Override
    public void close()
    {
        threadPoolExecutor.shutdownNow();
    }
}

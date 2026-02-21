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

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.nio.file.Files;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * A lightweight HTTP file server that serves files from the WPILib deploy directory.
 *
 * <p>This server allows browser-based tools (like the Behavior Tree Viewer) to be loaded directly
 * from the robot or simulation without needing any external hosting. The deploy directory maps
 * URL paths to files: a request for {@code /behaviortree/index.html} returns
 * {@code deploy/behaviortree/index.html}.
 *
 * <p>Call {@link #start(int)} once from {@code robotInit()} to launch the server in a background
 * daemon thread so it never blocks the robot loop.
 *
 * <p>Default port is {@code 5800}, which is in the WPILib-reserved range (5800–5809) for team
 * use.
 *
 * <p>Access the Behavior Tree Viewer at:
 *
 * <ul>
 *   <li><b>Simulation:</b> {@code http://localhost:5800/behaviortree/index.html}
 *   <li><b>Real robot:</b> {@code http://roboRIO-3467-FRC.local:5800/behaviortree/index.html}
 * </ul>
 */
public final class DeployFileServer {

    private static final Logger LOG = Logger.getLogger(DeployFileServer.class.getName());

    // MIME types for common file extensions served from the deploy directory
    private static final Map<String, String> MIME_TYPES = new HashMap<>();

    static {
        MIME_TYPES.put(".html", "text/html; charset=UTF-8");
        MIME_TYPES.put(".css", "text/css; charset=UTF-8");
        MIME_TYPES.put(".js", "application/javascript; charset=UTF-8");
        MIME_TYPES.put(".json", "application/json; charset=UTF-8");
        MIME_TYPES.put(".png", "image/png");
        MIME_TYPES.put(".jpg", "image/jpeg");
        MIME_TYPES.put(".svg", "image/svg+xml");
        MIME_TYPES.put(".ico", "image/x-icon");
        MIME_TYPES.put(".txt", "text/plain; charset=UTF-8");
        MIME_TYPES.put(".woff2", "font/woff2");
    }

    private DeployFileServer() {
        // Utility class — not instantiable
    }

    /**
     * Starts the HTTP file server on the given port, serving the WPILib deploy directory.
     *
     * <p>This method is safe to call multiple times; subsequent calls after the first successful
     * start are ignored. The server runs in a daemon thread and does not interfere with the robot
     * loop.
     *
     * <p>In simulation, the deploy directory is {@code src/main/deploy/} (relative to the project
     * root). On the real robot it is {@code /home/lvuser/deploy/}.
     *
     * @param port TCP port to listen on (recommended: {@code 5800})
     */
    public static void start(int port) {
        File deployDir = Filesystem.getDeployDirectory();
        try {
            HttpServer server = HttpServer.create(new InetSocketAddress(port), /* backlog= */ 0);
            server.createContext("/", exchange -> handleRequest(exchange, deployDir));

            // Use a cached thread pool of daemon threads so the server never blocks shutdown
            server.setExecutor(
                    Executors.newCachedThreadPool(
                            r -> {
                                Thread t = new Thread(r, "DeployFileServer");
                                t.setDaemon(true);
                                return t;
                            }));

            server.start();
            LOG.info(
                    "DeployFileServer started on port "
                            + port
                            + ", serving: "
                            + deployDir.getAbsolutePath());
        } catch (IOException e) {
            LOG.log(Level.WARNING, "DeployFileServer failed to start on port " + port, e);
        }
    }

    /** Handles a single HTTP request, mapping the URL path to a file in the deploy directory. */
    private static void handleRequest(HttpExchange exchange, File deployDir) throws IOException {
        String urlPath = exchange.getRequestURI().getPath();

        // Default to index.html for bare root request
        if (urlPath.equals("/")) {
            urlPath = "/index.html";
        }

        // Resolve the file relative to the deploy directory (safely)
        File requested = new File(deployDir, urlPath).getCanonicalFile();

        if (!isWithinDeployDir(deployDir, requested) || !requested.isFile()) {
            sendText(exchange, 404, "Not Found: " + urlPath);
            return;
        }

        byte[] body = Files.readAllBytes(requested.toPath());
        String mime = mimeTypeFor(requested.getName());

        exchange.getResponseHeaders().set("Content-Type", mime);
        exchange.getResponseHeaders().set("Access-Control-Allow-Origin", "*");
        exchange.sendResponseHeaders(200, body.length);
        try (OutputStream out = exchange.getResponseBody()) {
            out.write(body);
        }
    }

    /** Returns true only if {@code file} is inside {@code deployDir} (prevents path traversal). */
    private static boolean isWithinDeployDir(File deployDir, File file) throws IOException {
        return file.getCanonicalPath().startsWith(deployDir.getCanonicalPath() + File.separator)
                || file.getCanonicalPath().equals(deployDir.getCanonicalPath());
    }

    /** Sends a plain-text response with the given status code. */
    private static void sendText(HttpExchange exchange, int status, String message)
            throws IOException {
        byte[] body = message.getBytes(java.nio.charset.StandardCharsets.UTF_8);
        exchange.getResponseHeaders().set("Content-Type", "text/plain; charset=UTF-8");
        exchange.sendResponseHeaders(status, body.length);
        try (OutputStream out = exchange.getResponseBody()) {
            out.write(body);
        }
    }

    /** Returns the MIME type for a filename based on its extension. */
    private static String mimeTypeFor(String filename) {
        int dot = filename.lastIndexOf('.');
        if (dot >= 0) {
            return MIME_TYPES.getOrDefault(
                    filename.substring(dot).toLowerCase(), "application/octet-stream");
        }
        return "application/octet-stream";
    }
}

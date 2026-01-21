// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package util;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Utility class for logging messages both to console and to the data log
 */

/** Add your docs here. */
public class Logger {
    private static StringLogEntry logEntry;
    private static boolean initialized = false;
    
    /**
     * Initialize the logger
     */
    public static void init() {
        if (initialized) {
            return;
        }
        
        // Initialize data logging
        DataLogManager.start();
        DataLog log = DataLogManager.getLog();
        logEntry = new StringLogEntry(log, "/robot/messages");
        
        // Log that logger has been initialized
        log("Logger initialized");
        
        initialized = true;
    }
    
    /**
     * Log a message both to console and to the data log
     * @param message The message to log
     */
    public static void log(String message) {
        // System.out.println(message);
        if (logEntry != null) {
            logEntry.append(message);
        }
    }
    
    /**
     * Log a formatted message
     * @param format Format string
     * @param args Arguments for the format string
     */
    public static void logf(String format, Object... args) {
        log(String.format(format, args));
    }
    
    /**
     * Log an error message
     * @param message The error message
     */
    public static void error(String message) {
        String errorMsg = "ERROR: " + message;
        System.err.println(errorMsg);
        if (logEntry != null) {
            logEntry.append(errorMsg);
        }
    }
    
    /**
     * Log a warning message
     * @param message The warning message
     */
    public static void warning(String message) {
        String warningMsg = "WARNING: " + message;
        System.out.println(warningMsg);
        if (logEntry != null) {
            logEntry.append(warningMsg);
        }
    }
    
    /**
     * Log a debug message (only in debug mode)
     * @param message The debug message
     */
    public static void debug(String message) {
        // Could add a debug flag check here
        String debugMsg = "DEBUG: " + message;
        System.out.println(debugMsg);
        if (logEntry != null) {
            logEntry.append(debugMsg);
        }
    }
}

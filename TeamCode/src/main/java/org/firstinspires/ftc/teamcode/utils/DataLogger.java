package org.firstinspires.ftc.teamcode.utils;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;



public class DataLogger {
    // Configurable file location constant
    private static final String FILE_DIRECTORY = "/sdcard/FIRST/";
    
    private FileWriter writer;
    private String filename;
    private boolean isOpen;
    
    /**
     * Creates a new DataLogger with the specified filename
     * @param filename The name of the file to write to (without path)
     * @throws IOException if the file cannot be created
     */
    public DataLogger(String filename) throws IOException {
        this.filename = filename;
        File file = new File(FILE_DIRECTORY + filename);
        this.writer = new FileWriter(file, false); // false = overwrite existing file
        this.isOpen = true;
    }
    
    /**
     * Creates a new DataLogger with the specified filename and append mode
     * @param filename The name of the file to write to (without path)
     * @param append If true, appends to existing file; if false, overwrites
     * @throws IOException if the file cannot be created
     */
    public DataLogger(String filename, boolean append) throws IOException {
        this.filename = filename;
        File file = new File(FILE_DIRECTORY + filename);
        this.writer = new FileWriter(file, append);
        this.isOpen = true;
    }
    
    /**
     * Writes a dataLog object to the file by calling its toString() method
     * @param dataLog The object to log (must have toString() implemented)
     * @throws IOException if writing fails
     */
    public void log(DataLog dataLog) throws IOException {
        if (!isOpen) {
            throw new IOException("DataLogger is closed");
        }
        writer.write(dataLog.toString());
        writer.write("\n");
    }
    
    /**
     * Writes a dataLog object and immediately flushes to disk
     * Use this to ensure data is saved even if the program crashes
     * @param dataLog The object to log (must have toString() implemented)
     * @throws IOException if writing fails
     */
    public void logAndFlush(DataLog dataLog) throws IOException {
        log(dataLog);
        flush();
    }
    
    /**
     * Flushes the buffer, ensuring all data is written to disk
     * @throws IOException if flushing fails
     */
    public void flush() throws IOException {
        if (isOpen) {
            writer.flush();
        }
    }
    
    /**
     * Closes the file. No more data can be written after this.
     * @throws IOException if closing fails
     */
    public void close() throws IOException {
        if (isOpen) {
            writer.close();
            isOpen = false;
        }
    }
    
    /**
     * @return true if the logger is still open and can accept data
     */
    public boolean isOpen() {
        return isOpen;
    }
    
    /**
     * @return the filename being written to
     */
    public String getFilename() {
        return filename;
    }
    
    /**
     * @return the full path to the file
     */
    public String getFullPath() {
        return FILE_DIRECTORY + filename;
    }
}

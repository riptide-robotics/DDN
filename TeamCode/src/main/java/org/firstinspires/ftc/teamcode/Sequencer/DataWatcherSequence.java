package org.firstinspires.ftc.teamcode.Sequencer;

import java.lang.invoke.TypeDescriptor;

public class DataWatcherSequence<T> extends FullLoopSequence {
    public T tracked;
    public interface returningRunnable {Object run();}
    public Class<T> clazz;

    /**the DataWatcherSequence is a sequence that constantly processes one form of data into another. <br>
     * Use #getTracked() to retrieve the variable watched, which is assigned by the first runnable added, <br>
     * through its return statement. The second runnable is a standard processor where you can use #getTracked(). <br>
     * @param getTrackedVariable Use this to retrieve the variable.
     * @param process Use this to process the variable and write it wherever.
     * @param watchedObjectClass The class of the watched object.
     **/
    public DataWatcherSequence(returningRunnable getTrackedVariable, Runnable process, Class<T> watchedObjectClass) {
        super(() -> {});
        this.clazz = clazz;
        this.runnable = () -> {
            if (!clazz.isInstance(getTrackedVariable)) throw new RuntimeException("Type does not match watched!");
            tracked = (T) getTrackedVariable.run();
            process.run();
        };
    }

    public T getTracked() {
        return clazz.cast(tracked);
    }
}

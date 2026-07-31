package org.firstinspires.ftc.teamcode.Sequencer;

import java.lang.invoke.TypeDescriptor;

public class DataWatcherSequence<T> extends FullLoopSequence {
    public T tracked;
    public interface returningRunnable {Object run();}
    public Class<T> clazz;

    public DataWatcherSequence(returningRunnable getTrackedVariable, Runnable process, Class<T> clazz) {
        super(() -> {});
        this.clazz = clazz;
        this.runnable = () -> {
            if (!clazz.isInstance(getTrackedVariable)) throw new RuntimeException("Type does not match watched!");
            tracked = (T) getTrackedVariable.run();
            process.run();
        };


        []   }

    public T getTracked() {
        return clazz.cast(tracked);
    }
}

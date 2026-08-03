package org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes;

/**the DataWatcherSequence is a sequence that constantly processes one form of data into another. */
public class DataWatcherSequence<T> extends FullLoopSequence {
    public T tracked;
    public interface returningRunnable {Object run();}
    public Class<T> clazz;

    /**the DataWatcherSequence is a sequence that constantly processes one form of data into another. <br>
     * Use #getTracked() to retrieve the variable watched, which is assigned by the first runnable added, <br>
     * through its return statement. The second runnable is a standard processor where you can use #getTracked(). <br>
     * @param watchedObjectClass The class of the watched object.
     **/

    public DataWatcherSequence(returningRunnable runnable, Class<T> watchedObjectClass) {
        super(() -> {});
        this.clazz = watchedObjectClass;
        this.runnable = () -> values.set(0,runnable.run());
    }

    public T get() {
        return values.get(0,clazz);
    }
}

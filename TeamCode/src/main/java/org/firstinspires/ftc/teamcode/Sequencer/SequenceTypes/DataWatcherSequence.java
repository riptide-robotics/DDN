package org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes;

/**the DataWatcherSequence is a sequence that constantly processes one form of data into another. */
public class DataWatcherSequence<T> extends FullLoopSequence {
    public interface returningRunnable {Object run();}
    public Class<T> clazz;

    /**
     * The DataWatcherSequence is used to process data from one form to another.
     * @see #get() the <code>get()</code> method for retrieving that data.
     * */
    public DataWatcherSequence(returningRunnable runnable, Class<T> watchedObjectClass) {
        super((a) -> {});
        this.clazz = watchedObjectClass;
        this.runnable = (a) -> values.set(0,runnable.run());
    }
    /**
     * Fetches the variable saved within DataWatcherSequence. <br>
     * Stored within Values.
     **/
    public T get() {
        return values.get(0,clazz);
    }
}

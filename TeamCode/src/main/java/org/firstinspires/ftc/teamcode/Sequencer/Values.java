package org.firstinspires.ftc.teamcode.Sequencer;

import org.firstinspires.ftc.teamcode.Modules.Utils.Pair;

import java.util.ArrayList;
import java.util.List;


/** A container for data, built for the sequencer yet capable of other tasks as well. */
public class Values {
    List<Object> data = new ArrayList<>();

    public Values() {}

    public <T> void set(int slot, T value) {
        data.set(slot, value);
    }

    public <T> T get(int slot, Class<T> clazz) {
       return clazz.cast(data.get(slot));
    }
}

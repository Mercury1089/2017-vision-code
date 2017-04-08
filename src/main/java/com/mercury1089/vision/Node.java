package com.mercury1089.vision;

/**
 * @author Jared Tulayan
 */
@SuppressWarnings("unchecked")
public abstract class Node<I, O> {
    public final Class<I> INPUT_TYPE;
    public final Class<O> OUTPUT_TYPE;

    protected O output;

    public Node(Class<I> it, Class<O> ot) {
        INPUT_TYPE = it;
        OUTPUT_TYPE = ot;
    }

    public abstract void process(I input);
    public abstract void updateValue(String key, Object value);

    public O getOutput() {
        return output;
    }
}

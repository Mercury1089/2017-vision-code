package com.mercury1089.vision;

import org.opencv.core.Mat;

import java.util.ArrayList;

/**
 * Pipeline class that holds a list of {@link Node}s to traverse to process an image.
 * This is not meant to be derived from. Rather, {@code Node}s should be created,
 * then added to the pipeline.
 *
 * @see Node
 */
public class Pipeline {
    private final ArrayList<Node<?, ?>> NODES;

    public Pipeline() {
        NODES = new ArrayList<>();
    }

    /**
     * Adds a {@link Node}, but only if the output of the last node is the same as the input of the
     * specified {@code Node}.
     *
     * @param node the {@code Node} to add to the pipeline
     * @return the {@code Pipeline} for chaining
     */
    public Pipeline addNode(Node<?, ?> node) {
        Node<?, ?> prev = NODES.get(NODES.size() - 1);
        if (!NODES.isEmpty() && prev.OUTPUT_TYPE.equals(node.INPUT_TYPE) || node.INPUT_TYPE.equals(Mat.class))
            NODES.add(node);
        else {
            if (!NODES.isEmpty())
                throw new IllegalArgumentException(
                        "Node input is incompatible with previous node output.\n" +
                        "Required: " + prev.OUTPUT_TYPE.toString() + "\n" +
                        "Found: " + node.INPUT_TYPE.toString()
                );
            else
                throw new IllegalArgumentException ("Starting node must have an input of type Mat.");
        }

        return this;
    }

    @SuppressWarnings("unchecked")
    public void process(Mat input) {
        Mat in = input;
        ((Node<Mat, ?>)NODES.get(0)).process(in);
        for (int i = 1; i < NODES.size(); i++) {
            Node curNode = NODES.get(i);
            Node prevNode = NODES.get(i - 1);

            curNode.process(prevNode.getOutput());
        }
    }

    public Object getOutput() {
        return NODES.get(NODES.size()).getOutput();
    }

    public Object getOutput(int i) {
        return NODES.get(NODES.size()).getOutput();
    }
}

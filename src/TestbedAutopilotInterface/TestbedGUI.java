package TestbedAutopilotInterface;

import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Created by Martijn on 26/03/2018.
 * A class of tesbed Gui's for visualising the testbed contents
 */
public class TestbedGUI {

    /**
     * Constructor for a testbed GUI, generates all the gui elements from the testbed
     * @param guiQueue the queue to use for communication with the testbed, this is the
     *                 queue where the testbed will place its simulation results
     */
    public TestbedGUI(ConcurrentLinkedQueue<GUIQueueElement> guiQueue) {
        this.guiQueue = guiQueue;
    }

    /**
     * Reads an element from the queue. If the queue is empty returns the previously read element
     * as a placeholder until the next element will be simulated
     * @return a GUIQueue element containing the testbed results for rendering
     */
    private GUIQueueElement readFromQueue(){
        ConcurrentLinkedQueue<GUIQueueElement> queue = this.getGuiQueue();
        //peek in the queue, check if it is empty or not
        if(queue.isEmpty()){
            //if empty, return the previous one
            return this.getPrevGUIQueueElement();
        }
        //if not poll the head from the fifo queue
        GUIQueueElement queueHead = queue.poll();
        //set the previous one to the current
        this.setPrevGUIQueueElement(queueHead);
        //return the result
        return queueHead;
    }

    /**
     * Setter for the previous queue element
     * Saved to re-render while the testbed is busy simulating
     * @param prevGUIQueueElement the previous gui element
     */
    private void setPrevGUIQueueElement(GUIQueueElement prevGUIQueueElement) {
        this.prevGUIQueueElement = prevGUIQueueElement;
    }

    /**
     * getter for the previously polled Queue element
     * @return the previous queue element that was read from the queue
     */
    private GUIQueueElement getPrevGUIQueueElement() {
        return prevGUIQueueElement;
    }

    /**
     * Getter for the queue where all the testbed results will be posted
     * @return a concurrent linked queue to read the data retrieved by the testbed
     */
    private ConcurrentLinkedQueue<GUIQueueElement> getGuiQueue() {
        return guiQueue;
    }

    /**
     * The previously received Queue element
     */
    private GUIQueueElement prevGUIQueueElement;

    /**
     * The queue the Gui receives the queue elements from to render on screen
     */
    private ConcurrentLinkedQueue<GUIQueueElement> guiQueue;
}

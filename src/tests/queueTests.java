package tests;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.*;

/**
 * Created by Martijn on 26/03/2018.
 */
public class queueTests {

    @Test
    public void queueTest() throws InterruptedException, ExecutionException {
        ConcurrentLinkedQueue<String> queue = new ConcurrentLinkedQueue<>();
        Callable<Void> end1 = new Callable<Void>(){
            @Override
            public Void call() throws Exception {
                String messageString =  "hello this is a message that will be sent trough the queue";
                String[] lines = messageString.split(" ");
                //System.out.println(Arrays.toString(lines));
                for(String line: lines){
                    queue.add(line);
                    Thread.sleep(1000);
                }

                System.out.println("All lines added to queue");
                return null;
            }
        };

        Callable<Void> end2 = new Callable<Void>() {
            @Override
            public Void call() throws Exception {
                //read the queue until null
                String line;
                boolean reading = true;
                while(reading){
                    if(!queue.isEmpty()){
                        line = queue.poll();
                        System.out.println(line);
                        if(line.equals("queue")){
                            reading = false;
                        }
                    }
                }

                return null;
            }
        };

        //test read the queue, submit both simultaneously
        ExecutorService threads = Executors.newFixedThreadPool(4);
        List<Callable<Void>> callableList = new ArrayList<>();
        threads.submit(end1);
        Future<Void> future = threads.submit(end2);
        future.get();
    }
}

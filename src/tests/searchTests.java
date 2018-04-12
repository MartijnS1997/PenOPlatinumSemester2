package tests;

import AutopilotInterfaces.AutopilotInputs_v2;
import TestbedAutopilotInterface.Overseer.*;
import internal.Helper.Vector;
import org.junit.Test;

import java.util.*;

/**
 * Created by Martijn on 4/04/2018.
 * A class to test the search implementation
 */
public class searchTests {

    @Test
    public void testSearch() throws Exception {
        //generate the airports
        OverseerAirportMap airportMap = generateAirportMap();
        Map<String, Vector> drones = generateDrones(airportMap);
        Set<DeliveryPackage> packages = generateDeliveries(airportMap,0);

        //print the airport map
        System.out.println(airportMap);
        System.out.println(airportMap);
        long startTime = System.currentTimeMillis();
        DeliveryPlanning planner = new DeliveryPlanning(packages, airportMap, drones);
        Map<String, List<DeliveryPackage>> schedule = planner.call();
        long endTime = System.currentTimeMillis();
        long totalTime = endTime - startTime;
        printSchedule(schedule, airportMap, drones);

        //add new deliveries
        startTime = System.currentTimeMillis();
        Set<DeliveryPackage> newPackages = generateDeliveries(airportMap, 1);
        planner.addPackages(newPackages);
        Map<String, List<DeliveryPackage>> addedSchedule = planner.call();
        endTime = System.currentTimeMillis();
        printSchedule(addedSchedule, airportMap, drones);
        totalTime = totalTime + (endTime - startTime);
        System.out.println("total execution time: " + totalTime);

    }

    private void printSchedule(Map<String, List<DeliveryPackage>> schedule, OverseerAirportMap airportMap, Map<String, Vector> drones){
        for(String droneID : schedule.keySet()){
            System.out.println("Current Drone: " + droneID + ", Start: " + drones.get(droneID));
            System.out.println();
            //get the delivery list
            List<DeliveryPackage> deliveryPackages = schedule.get(droneID);
            for(DeliveryPackage deliveryPackage: deliveryPackages){
                //get the airport corresponding the delivery
                int sourceID = deliveryPackage.getSourceAirport();
                int destinationID = deliveryPackage.getDestinationAirport();
                System.out.println("Delivery: " );

                MapAirport sourceAirport = airportMap.getAirport(sourceID);
                System.out.println("Source: " + sourceAirport);
                MapAirport destinationAirport = airportMap.getAirport(destinationID);
                System.out.println("Destination: " + destinationAirport);
                System.out.println();
            }
        }
    }

    private static Map<String, Vector> generateDrones(OverseerAirportMap airportMap){
        //we only need two drones located at two airports
        Map<String, Vector> drones = new HashMap<>();
        //create the first drone
        MapAirport airport = airportMap.getAirport(0);
        Vector location1 = airport.getLocation();

        airport = airportMap.getAirport(6);
        Vector location2 = airport.getLocation();

        airport = airportMap.getAirport(15);
        Vector location3 = airport.getLocation();

        drones.put("0", location1);
        drones.put("1", location2);
        drones.put("2", location3);

        return drones;
    }

    private static OverseerAirportMap generateAirportMap(){
        //lets create 9 airports arranged in a square
        int nbRows = 4;
        int nbColumns = 4;
        float rowSpace = 2500; //z-distance between airports
        float columnSpace = 2000; //x-distance between airports

        float runwayLength = 280;
        float runwayWidth = 20;

        Vector heading = new Vector();
        Vector position = null;

        OverseerAirportMap map = new OverseerAirportMap(runwayWidth, runwayLength);
        for(int i = 0; i != nbRows; i++){
            for(int j = 0; j != nbColumns; j++){
                float xPos = j*columnSpace;
                float zPos = i*rowSpace;
                position = new Vector(xPos, 0,zPos);
                map.addAirport(position, heading);
            }
        }

        return map;
    }

    private static Set<DeliveryPackage> generateDeliveries(OverseerAirportMap airportMap, int modRes){
        //set one package for every airport
        //deliver the package to airport with index + 2 (mod nb airports)
        int nbOfAirports = airportMap.getNbOfAirports();
        Set<DeliveryPackage> deliveries = new HashSet<>();
        for(int id = 0; id != nbOfAirports; id++){
            if(id%2 == modRes)
                continue;
            DeliveryPackage deliveryPackage = new DeliveryPackage(id, 0, (id + 2)%nbOfAirports, 0);
            deliveries.add(deliveryPackage);
        }

        return deliveries;
    }

}

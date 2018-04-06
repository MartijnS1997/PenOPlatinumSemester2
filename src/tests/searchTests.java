package tests;

import AutopilotInterfaces.AutopilotInputs_v2;
import TestbedAutopilotInterface.*;
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
        Map<String, AutopilotInputs_v2> drones = generateDrones(airportMap);
        Set<DeliveryPackage> packages = generateDeliveries(airportMap);

        //print the airport map
        System.out.println(airportMap);
        System.out.println(airportMap);

        DeliveryPlanning planner = new DeliveryPlanning(packages, airportMap, drones);
        Map<String, List<DeliveryPackage>> schedule = planner.call();
        printSchedule(schedule, airportMap, drones);

    }

    private void printSchedule(Map<String, List<DeliveryPackage>> schedule, OverseerAirportMap airportMap, Map<String, AutopilotInputs_v2> drones){
        for(String droneID : schedule.keySet()){
            System.out.println("Current Drone: " + droneID + ", Start: " + AutopilotOverseer.extractPosition(drones.get(droneID)));
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

    private static Map<String, AutopilotInputs_v2> generateDrones(OverseerAirportMap airportMap){
        //we only need two drones located at two airports
        Map<String, AutopilotInputs_v2> drones = new HashMap<>();
        //create the first drone
        MapAirport airport = airportMap.getAirport(0);
        Vector location1 = airport.getLocation();
        AutopilotInputs_v2 inputsDrone1 = new AutopilotInputs_v2() {
            @Override
            public byte[] getImage() {
                return new byte[0];
            }

            @Override
            public float getX() {
                return location1.getxValue();
            }

            @Override
            public float getY() {
                return location1.getyValue();
            }

            @Override
            public float getZ() {
                return location1.getzValue();
            }

            @Override
            public float getHeading() {
                return 0;
            }

            @Override
            public float getPitch() {
                return 0;
            }

            @Override
            public float getRoll() {
                return 0;
            }

            @Override
            public float getElapsedTime() {
                return 0;
            }
        };

        airport = airportMap.getAirport(8);
        Vector location2 = airport.getLocation();

        AutopilotInputs_v2 inputsDrone2 = new AutopilotInputs_v2() {
            @Override
            public byte[] getImage() {
                return new byte[0];
            }

            @Override
            public float getX() {
                return location2.getxValue();
            }

            @Override
            public float getY() {
                return location2.getyValue();
            }

            @Override
            public float getZ() {
                return location2.getzValue();
            }

            @Override
            public float getHeading() {
                return 0;
            }

            @Override
            public float getPitch() {
                return 0;
            }

            @Override
            public float getRoll() {
                return 0;
            }

            @Override
            public float getElapsedTime() {
                return 0;
            }
        };

        drones.put("0", inputsDrone1);
        drones.put("1", inputsDrone2);

        return drones;
    }

    private static OverseerAirportMap generateAirportMap(){
        //lets create 9 airports arranged in a square
        int nbRows = 3;
        int nbColumns = 3;
        float rowSpace = 2000; //z-distance between airports
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

    private static Set<DeliveryPackage> generateDeliveries(OverseerAirportMap airportMap){
        //set one package for every airport
        //deliver the package to airport with index + 2 (mod nb airports)
        int nbOfAirports = airportMap.getNbOfAirports();
        Set<DeliveryPackage> deliveries = new HashSet<>();
        for(int id = 0; id != nbOfAirports; id++){
//            if(id%2 == 0)
//                continue;
            DeliveryPackage deliveryPackage = new DeliveryPackage(id, 0, (id + 2)%nbOfAirports, 0);
            deliveries.add(deliveryPackage);
        }

        return deliveries;
    }

}

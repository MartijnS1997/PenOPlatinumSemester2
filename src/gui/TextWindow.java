package gui;

import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.Dimension;

import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.SpringLayout;

import internal.Testbed.Drone;
import internal.Testbed.World;
import internal.Testbed.WorldObject;
import math.Vector3f;

import javax.swing.JFrame;
import javax.swing.JLabel;
 
public class TextWindow extends JPanel{
	private static final long serialVersionUID = 1L;
	private static JFrame frame;
	static Container contentPane;
    
    public static void update(World world) {
    	contentPane.removeAll();
    	addToContentPane(world);
    	contentPane.revalidate();
    	frame.repaint();
    }
    
    private static void addToContentPane(World world) {
    	Vector3f velocity = new Vector3f();
    	Vector3f position = new Vector3f();
    	Vector3f orientation = new Vector3f();
    	float heading = 0;
    	float pitch = 0;
    	float roll = 0;
    	
    	
        for (WorldObject object: world.getObjectSet()) {
        	if (object.getClass() == Drone.class) {
        		velocity = ((Drone) object).getVelocity().convertToVector3f();
        		position = ((Drone) object).getPosition().convertToVector3f();
        		heading = ((Drone) object).getHeading();
        		pitch = ((Drone) object).getPitch();
        		roll = ((Drone) object).getRoll();

        	}
        }
    	
    	JLabel velocityLabel = new JLabel("Velocity: ");
    	JTextField velocityField = new JTextField(" ( " + velocity.x + ", " + velocity.y + ", " + velocity.z + " ) ");
    	velocityField.setEditable(false);
    	contentPane.add(velocityLabel);
    	contentPane.add(velocityField); 
    	
    	JLabel positionLabel = new JLabel("Position: ");
    	JTextField positionField = new JTextField(" ( " + position.x + ", " + position.y + ", " + position.z + " ) ");
    	positionField.setEditable(false);
    	contentPane.add(positionLabel);
    	contentPane.add(positionField); 
    	
    	JLabel headingLabel = new JLabel("Heading: ");
    	JTextField headingField = new JTextField(" ( " + heading + " ) ");
    	headingField.setEditable(false);
    	contentPane.add(headingLabel);
    	contentPane.add(headingField); 
    	
    	JLabel pitchLabel = new JLabel("Pitch: ");
    	JTextField pitchField = new JTextField(" ( " + pitch + " ) ");
    	pitchField.setEditable(false);
    	contentPane.add(pitchLabel);
    	contentPane.add(pitchField); 
    	
    	JLabel rollLabel = new JLabel("Roll: ");
    	JTextField rollField = new JTextField(" ( " + roll + " ) ");
    	rollField.setEditable(false);
    	contentPane.add(rollLabel);
    	contentPane.add(rollField); 
    	
    	SpringLayout layout = new SpringLayout();
        contentPane.setLayout(layout);
        
    	layout.putConstraint(SpringLayout.WEST, velocityLabel, 5, SpringLayout.WEST, contentPane);
    	layout.putConstraint(SpringLayout.NORTH, velocityLabel, 5, SpringLayout.NORTH, contentPane);
    	layout.putConstraint(SpringLayout.WEST, velocityField, 5, SpringLayout.EAST, velocityLabel);
    	layout.putConstraint(SpringLayout.NORTH, velocityField, 5, SpringLayout.NORTH, contentPane);
    	
    	layout.putConstraint(SpringLayout.WEST, positionLabel, 5, SpringLayout.WEST, contentPane);
    	layout.putConstraint(SpringLayout.NORTH, positionLabel, 30, SpringLayout.NORTH, contentPane);
    	layout.putConstraint(SpringLayout.WEST, positionField, 5, SpringLayout.EAST, positionLabel);
    	layout.putConstraint(SpringLayout.NORTH, positionField, 30, SpringLayout.NORTH, contentPane);
    	
    	layout.putConstraint(SpringLayout.WEST, headingLabel, 5, SpringLayout.WEST, contentPane);
    	layout.putConstraint(SpringLayout.NORTH, headingLabel, 55, SpringLayout.NORTH, contentPane);
    	layout.putConstraint(SpringLayout.WEST, headingField, 5, SpringLayout.EAST, headingLabel);
    	layout.putConstraint(SpringLayout.NORTH, headingField, 55, SpringLayout.NORTH, contentPane);
    	
    	layout.putConstraint(SpringLayout.WEST, pitchLabel, 5, SpringLayout.WEST, contentPane);
    	layout.putConstraint(SpringLayout.NORTH, pitchLabel, 80, SpringLayout.NORTH, contentPane);
    	layout.putConstraint(SpringLayout.WEST, pitchField, 5, SpringLayout.EAST, pitchLabel);
    	layout.putConstraint(SpringLayout.NORTH, pitchField, 80, SpringLayout.NORTH, contentPane);
    	
    	layout.putConstraint(SpringLayout.WEST, rollLabel, 5, SpringLayout.WEST, contentPane);
    	layout.putConstraint(SpringLayout.NORTH, rollLabel, 105, SpringLayout.NORTH, contentPane);
    	layout.putConstraint(SpringLayout.WEST, rollField, 5, SpringLayout.EAST, rollLabel);
    	layout.putConstraint(SpringLayout.NORTH, rollField, 105, SpringLayout.NORTH, contentPane);
    }

    public static void createAndShowWindow(World world, Graphics graphics, String title, int xDimension, int yDimension, int xPos, int yPos) {
    	
        frame = new JFrame(title);
        frame.setPreferredSize(new Dimension(xDimension, yDimension));
        frame.setLocation(xPos, yPos);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        
        contentPane = frame.getContentPane();
        addToContentPane(world);
        frame.setContentPane(contentPane);
 
        frame.pack();
        frame.setVisible(true);
    }
}


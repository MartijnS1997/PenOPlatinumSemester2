package gui.Windows;

import java.awt.Color;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.SpringLayout;

import TestbedAutopilotInterface.GUI.DeliveryGuiState;
import TestbedAutopilotInterface.GUI.DroneGuiState;
import gui.WorldObjects.Drone;
import gui.WorldObjects.Objects;
import math.Vector2f;
import math.Vector3f;

import javax.swing.AbstractButton;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
 
public class TextWindow extends JPanel implements ActionListener{
	
	private Map<Vector2f, MiniMapObject> dronePositions = new HashMap<Vector2f, MiniMapObject>();
	private static final long serialVersionUID = 1L;
	private static JFrame frame;
	static Container contentPane;
	protected JButton button1, button2, button3, button4, button5;
	SpringLayout layout;
	private static Graphics graphics;
	byte counter = 0;
	float totalDistance = 0;
	Vector3f lastPosition;
	Vector3f firstPosition = null;
	Vector3f position;
	int deliveredPackages;
	int pickedUpPackages;
	int totalPackages;
	String hasPackage;
    
    public void update(DroneGuiState droneState, Set<DeliveryGuiState> deliveryGuiStates) {

		this.deliveredPackages = 0;
		this.pickedUpPackages = 0;
		this.totalPackages = 0;
		hasPackage = "No";
		for (DeliveryGuiState dgs: deliveryGuiStates) {
			totalPackages++;
			if (dgs.isDelivered()) {
				deliveredPackages++;
			}
			if (dgs.isPickedUp()) {
				pickedUpPackages++;
			}
			if (dgs.isPickedUp() && !dgs.isDelivered() && dgs.getDeliveryDrone().equals(Objects.getMainDroneID())) {
				hasPackage = "Yes";
			}
		}
		

    	counter++;
    	counter %= 5;
    	if (counter == 0) {
	    	contentPane.removeAll();
	    	addToContentPane(droneState);
	    	addButtons();
	    	contentPane.revalidate();
	    	frame.repaint();
    	}
    }
    
    private void addToContentPane(DroneGuiState droneState) {
    	Vector3f velocity = new Vector3f();
    	float speed = 0;
    	Vector3f orientation = new Vector3f();
    	float heading = 0;
    	float pitch = 0;
    	float roll = 0;
    	float distanceOrigin = 0;

    	velocity = droneState.getVelocity().convertToVector3f();
    	speed = droneState.getVelocity().convertToVector3f().length();

        if (firstPosition == null) {
			position = droneState.getPosition().convertToVector3f();
			firstPosition = position;
			lastPosition = position;
		} else {
			lastPosition = position;
			position = droneState.getPosition().convertToVector3f();
		}
        totalDistance += position.subtract(lastPosition).length();
        orientation = droneState.getOrientation().convertToVector3f();
        heading = orientation.x;
        pitch = orientation.y;
        roll = orientation.z;
		distanceOrigin = droneState.getPosition().convertToVector3f().subtract(firstPosition).length();
    	
    	JLabel velocityLabel = new JLabel("Velocity: ");
    	JTextField velocityField = new JTextField(" ( " + String.format("%.2f", velocity.x) + ", " + String.format("%.2f", velocity.y) + ", " + String.format("%.2f", velocity.z) + " ) ");
    	velocityField.setEditable(false);
    	contentPane.add(velocityLabel);
    	contentPane.add(velocityField); 
    	
    	JLabel speedLabel = new JLabel("Speed: ");
    	JTextField speedField = new JTextField(String.format("%.2f", speed));
    	speedField.setEditable(false);
    	contentPane.add(speedLabel);
    	contentPane.add(speedField); 
    	
    	JLabel positionLabel = new JLabel("Position: ");
    	JTextField positionField = new JTextField(" ( " + String.format("%.2f", position.x) + ", " + String.format("%.2f", position.y) + ", " + String.format("%.2f", position.z) + " ) ");
    	positionField.setEditable(false);
    	contentPane.add(positionLabel);
    	contentPane.add(positionField); 
    	
    	JLabel headingLabel = new JLabel("Heading: ");
    	JTextField headingField = new JTextField(String.format("%.2f", heading));
    	headingField.setEditable(false);
    	contentPane.add(headingLabel);
    	contentPane.add(headingField); 
    	
    	JLabel pitchLabel = new JLabel("Pitch: ");
    	JTextField pitchField = new JTextField(String.format("%.2f", pitch));
    	pitchField.setEditable(false);
    	contentPane.add(pitchLabel);
    	contentPane.add(pitchField); 
    	
    	JLabel rollLabel = new JLabel("Roll: ");
    	JTextField rollField = new JTextField(String.format("%.2f", roll));
    	rollField.setEditable(false);
    	contentPane.add(rollLabel);
    	contentPane.add(rollField); 
    	
    	JLabel distOriginLabel = new JLabel("Distance to origin: ");
    	JTextField distOriginField = new JTextField(String.format("%.2f", distanceOrigin));
		distOriginField.setEditable(false);
    	contentPane.add(distOriginLabel);
    	contentPane.add(distOriginField); 
    	
    	JLabel totalDistLabel = new JLabel("Total distance traveled: ");
    	JTextField totalDistField = new JTextField(String.format("%.2f", totalDistance));
		totalDistField.setEditable(false);
    	contentPane.add(totalDistLabel);
    	contentPane.add(totalDistField);

    	JLabel packagesPickedUpLabel = new JLabel("Packages picked up: ");
		JTextField packagesPickedUpField = new JTextField(pickedUpPackages + "/" + totalPackages);
		packagesPickedUpField.setEditable(false);
		contentPane.add(packagesPickedUpLabel);
		contentPane.add(packagesPickedUpField);

		JLabel packagesDeliveredLabel = new JLabel("Packages delivered: ");
		JTextField packagesDeliveredField = new JTextField(deliveredPackages + "/" + totalPackages);
		packagesDeliveredField.setEditable(false);
		contentPane.add(packagesDeliveredLabel);
		contentPane.add(packagesDeliveredField);
		
		JLabel hasPackageLabel = new JLabel("Does this drone have a package? ");
		JTextField hasPackageField = new JTextField(hasPackage);
		hasPackageField.setEditable(false);
		contentPane.add(hasPackageLabel);
		contentPane.add(hasPackageField);

		layout = new SpringLayout();
        contentPane.setLayout(layout);
        
    	layout.putConstraint(SpringLayout.WEST, velocityLabel, 5, SpringLayout.WEST, contentPane);
    	layout.putConstraint(SpringLayout.NORTH, velocityLabel, 50, SpringLayout.NORTH, contentPane);
    	layout.putConstraint(SpringLayout.WEST, velocityField, 5, SpringLayout.EAST, hasPackageLabel);
    	layout.putConstraint(SpringLayout.NORTH, velocityField, 50, SpringLayout.NORTH, contentPane);
    	
    	layout.putConstraint(SpringLayout.WEST, speedLabel, 5, SpringLayout.WEST, contentPane);
    	layout.putConstraint(SpringLayout.NORTH, speedLabel, 22, SpringLayout.NORTH, velocityField);
    	layout.putConstraint(SpringLayout.WEST, speedField, 5, SpringLayout.EAST, hasPackageLabel);
    	layout.putConstraint(SpringLayout.NORTH, speedField, 22, SpringLayout.NORTH, velocityField);
    	
    	layout.putConstraint(SpringLayout.WEST, positionLabel, 5, SpringLayout.WEST, contentPane);
    	layout.putConstraint(SpringLayout.NORTH, positionLabel, 22, SpringLayout.NORTH, speedField);
    	layout.putConstraint(SpringLayout.WEST, positionField, 5, SpringLayout.EAST, hasPackageLabel);
    	layout.putConstraint(SpringLayout.NORTH, positionField, 22, SpringLayout.NORTH, speedField);
    	
    	layout.putConstraint(SpringLayout.WEST, headingLabel, 5, SpringLayout.WEST, contentPane);
    	layout.putConstraint(SpringLayout.NORTH, headingLabel, 22, SpringLayout.NORTH, positionField);
    	layout.putConstraint(SpringLayout.WEST, headingField, 5, SpringLayout.EAST, hasPackageLabel);
    	layout.putConstraint(SpringLayout.NORTH, headingField, 22, SpringLayout.NORTH, positionField);
    	
    	layout.putConstraint(SpringLayout.WEST, pitchLabel, 5, SpringLayout.WEST, contentPane);
    	layout.putConstraint(SpringLayout.NORTH, pitchLabel, 22, SpringLayout.NORTH, headingField);
    	layout.putConstraint(SpringLayout.WEST, pitchField, 5, SpringLayout.EAST, hasPackageLabel);
    	layout.putConstraint(SpringLayout.NORTH, pitchField, 22, SpringLayout.NORTH, headingField);
    	
    	layout.putConstraint(SpringLayout.WEST, rollLabel, 5, SpringLayout.WEST, contentPane);
    	layout.putConstraint(SpringLayout.NORTH, rollLabel, 22, SpringLayout.NORTH, pitchField);
    	layout.putConstraint(SpringLayout.WEST, rollField, 5, SpringLayout.EAST, hasPackageLabel);
    	layout.putConstraint(SpringLayout.NORTH, rollField, 22, SpringLayout.NORTH, pitchField);
    	
    	layout.putConstraint(SpringLayout.WEST, distOriginLabel, 5, SpringLayout.WEST, contentPane);
    	layout.putConstraint(SpringLayout.NORTH, distOriginLabel, 22, SpringLayout.NORTH, rollField);
    	layout.putConstraint(SpringLayout.WEST, distOriginField, 5, SpringLayout.EAST, hasPackageLabel);
    	layout.putConstraint(SpringLayout.NORTH, distOriginField, 22, SpringLayout.NORTH, rollField);
    	
    	layout.putConstraint(SpringLayout.WEST, totalDistLabel, 5, SpringLayout.WEST, contentPane);
    	layout.putConstraint(SpringLayout.NORTH, totalDistLabel, 22, SpringLayout.NORTH, distOriginField);
    	layout.putConstraint(SpringLayout.WEST, totalDistField, 5, SpringLayout.EAST, hasPackageLabel);
    	layout.putConstraint(SpringLayout.NORTH, totalDistField, 22, SpringLayout.NORTH, distOriginField);

		layout.putConstraint(SpringLayout.WEST, packagesPickedUpLabel, 5, SpringLayout.WEST, contentPane);
		layout.putConstraint(SpringLayout.NORTH, packagesPickedUpLabel, 22, SpringLayout.NORTH, totalDistField);
		layout.putConstraint(SpringLayout.WEST, packagesPickedUpField, 5, SpringLayout.EAST, hasPackageLabel);
		layout.putConstraint(SpringLayout.NORTH, packagesPickedUpField, 22, SpringLayout.NORTH, totalDistField);

		layout.putConstraint(SpringLayout.WEST, packagesDeliveredLabel, 5, SpringLayout.WEST, contentPane);
		layout.putConstraint(SpringLayout.NORTH, packagesDeliveredLabel, 22, SpringLayout.NORTH, packagesPickedUpField);
		layout.putConstraint(SpringLayout.WEST, packagesDeliveredField, 5, SpringLayout.EAST, hasPackageLabel);
		layout.putConstraint(SpringLayout.NORTH, packagesDeliveredField, 22, SpringLayout.NORTH, packagesPickedUpField);
		
		layout.putConstraint(SpringLayout.WEST, hasPackageLabel, 5, SpringLayout.WEST, contentPane);
		layout.putConstraint(SpringLayout.NORTH, hasPackageLabel, 22, SpringLayout.NORTH, packagesDeliveredField);
		layout.putConstraint(SpringLayout.WEST, hasPackageField, 5, SpringLayout.EAST, hasPackageLabel);
		layout.putConstraint(SpringLayout.NORTH, hasPackageField, 22, SpringLayout.NORTH, packagesDeliveredField);
    }
    
    public void initButtons() {
    	button1 = new JButton("drone view");
        button1.setVerticalTextPosition(AbstractButton.CENTER);
        button1.setHorizontalTextPosition(AbstractButton.LEADING);
        button1.setMnemonic(KeyEvent.VK_1);
        button1.setActionCommand("DRONE_CAM");
 
        button2 = new JButton("chase view");
        button2.setMnemonic(KeyEvent.VK_2);
        button2.setActionCommand("DRONE_CHASE_CAM");
        
        button3 = new JButton("top down view");
        button3.setMnemonic(KeyEvent.VK_3);
        button3.setActionCommand("DRONE_TOP_DOWN_CAM");
        
        button4 = new JButton("side view");
        button4.setMnemonic(KeyEvent.VK_4);
        button4.setActionCommand("DRONE_SIDE_CAM");

		button5 = new JButton("Next drone");
		button5.setMnemonic(KeyEvent.VK_5);
		button5.setActionCommand("DRONE_NEXT");
 
        button1.setBackground(Color.CYAN);
        button2.setBackground(Color.lightGray);
        button3.setBackground(Color.lightGray);
        button4.setBackground(Color.lightGray);
        button5.setBackground(Color.WHITE);
 
        button1.setToolTipText("Shows the drone from its perspective.");
        button2.setToolTipText("Shows the drone from behind.");
        button3.setToolTipText("Shows the drone from the top.");
        button4.setToolTipText("Shows the drone from the side.");
		button5.setToolTipText("Switch to the next drone.");
        
        button1.addActionListener(this);
        button2.addActionListener(this);
        button3.addActionListener(this);
        button4.addActionListener(this);
		button5.addActionListener(this);
 
        addButtons();
    }
    
    public void addButtons() {
    	 
    	contentPane.add(button1);
        contentPane.add(button2);
        contentPane.add(button3);
        contentPane.add(button4);
		contentPane.add(button5);
        
        layout.putConstraint(SpringLayout.WEST, button1, 10, SpringLayout.WEST, contentPane);
    	layout.putConstraint(SpringLayout.NORTH, button1, 10, SpringLayout.NORTH, contentPane);
    	
    	layout.putConstraint(SpringLayout.WEST, button2, 10, SpringLayout.EAST, button1);
    	layout.putConstraint(SpringLayout.NORTH, button2, 10, SpringLayout.NORTH, contentPane);
    	
    	layout.putConstraint(SpringLayout.WEST, button3, 10, SpringLayout.EAST, button2);
    	layout.putConstraint(SpringLayout.NORTH, button3, 10, SpringLayout.NORTH, contentPane);
    	
    	layout.putConstraint(SpringLayout.WEST, button4, 10, SpringLayout.EAST, button3);
    	layout.putConstraint(SpringLayout.NORTH, button4, 10, SpringLayout.NORTH, contentPane);

		layout.putConstraint(SpringLayout.WEST, button5, 10, SpringLayout.EAST, button3);
		layout.putConstraint(SpringLayout.SOUTH, button5, -20, SpringLayout.SOUTH, contentPane);
    }

    public static TextWindow createAndShowWindow(Graphics graphics, String title, int xDimension, int yDimension, int xPos, int yPos, DroneGuiState droneState) {
    	
    	TextWindow.graphics = graphics;
    	
        frame = new JFrame(title);
        frame.setPreferredSize(new Dimension(xDimension, yDimension));
        frame.setLocation(xPos, yPos);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        
        contentPane = frame.getContentPane();
        TextWindow window = new TextWindow();
        window.addToContentPane(droneState);
        window.initButtons();
        frame.setContentPane(contentPane);
 
        frame.pack();
        frame.setVisible(true);
        
        return window;
    }

	@Override
	public void actionPerformed(ActionEvent e) {
		if ("DRONE_CAM".equals(e.getActionCommand())) {
    		button1.setBackground(Color.CYAN);
            button2.setBackground(Color.lightGray);
            button3.setBackground(Color.lightGray);
            button4.setBackground(Color.lightGray);
        	for (String key: graphics.windows.keySet()) {
    			Window window = graphics.windows.get(key);
    			window.setSetting(Settings.DRONE_CAM);
    			break;
        	}
        } else if ("DRONE_CHASE_CAM".equals(e.getActionCommand()))  {
        	button1.setBackground(Color.lightGray);
            button2.setBackground(Color.CYAN);
            button3.setBackground(Color.lightGray);
            button4.setBackground(Color.lightGray);
        	for (String key: graphics.windows.keySet()) {
    			Window window = graphics.windows.get(key);
    			window.setSetting(Settings.DRONE_CHASE_CAM);
    			break;
        	}
        } else if ("DRONE_TOP_DOWN_CAM".equals(e.getActionCommand()))  {
        	button1.setBackground(Color.lightGray);
            button2.setBackground(Color.lightGray);
            button3.setBackground(Color.CYAN);
            button4.setBackground(Color.lightGray);
        	for (String key: graphics.windows.keySet()) {
    			Window window = graphics.windows.get(key);
    			window.setSetting(Settings.DRONE_TOP_DOWN_CAM);
    			break;
        	}
        } else if ("DRONE_SIDE_CAM".equals(e.getActionCommand()))  {
        	button1.setBackground(Color.lightGray);
            button2.setBackground(Color.lightGray);
            button3.setBackground(Color.lightGray);
            button4.setBackground(Color.CYAN);
        	for (String key: graphics.windows.keySet()) {
    			Window window = graphics.windows.get(key);
    			window.setSetting(Settings.DRONE_SIDE_CAM);
    			break;
        	}
        }

		if ("DRONE_NEXT".equals(e.getActionCommand()))  {
			Objects.nextDrone();
		}
	}
}


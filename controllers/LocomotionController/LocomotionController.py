import math
from controller import Robot

class LocomotionController(Robot):
	"""
	This defines a controller for the Yamor robot.
	"""  

	# User defined function for initializing and running
	# the YamorController class
	def run(self):
    
		# Connect to devices
		motor = self.getMotor('motor')
		rear_connector = self.getConnector('rear_connector')
		front_connector = self.getConnector('front_connector')
		
		t = 0.0

		# Main loop
		while True:
			# Perform a simulation step of 64 milliseconds
			# and leave the loop when the simulation is over
			if self.step(64) == -1:
                  		break

			motor.setPosition(t * math.sin(t))
			t += 64.0/1000.0

    
		# Enter here exit cleanup code

# The main program starts from here

# This is the main program of your controller.
# It creates an instance of your Robot subclass, launches its
# function(s) and destroys it at the end of the execution.
# Note that only one instance of Robot should be created in
# a controller program.
controller = LocomotionController()
controller.run()

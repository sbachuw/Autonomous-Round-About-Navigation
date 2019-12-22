# Autonomous-Round-About-Navigation
A simulation for roundabout navigation with collision avoidance using force based local navigation.

Owners: Huzefa Dossaji, Sanket Bachuwar, Ardashir Bulsura
Motion Planning of a Vehicle in a Roundabout Scenario

The objective was achieved using the method of waypoints and forced based local navigation. Bezier curves were used to generate the waypoints and trajectory for the agents to enter and exit the roundabout as the the entry and exit lanes designed were single lanes. While forced based navigation was used inside the roundabout, as it consisted of two lanes. This allowed multiple agents to navigate the roundabout at the same time without colliding with each other.

This repositiory contains the simulator.py which runs the canvas simulation and agent.py file which contains definitions and functions of agent behavior of a roundabout scenerio simulation. 
This repositiory also contains agent csv file (crossing_agents) which has starting information for the agent on the roundabout.
This repositiory also contains our final report in pdf form and presentation.


To run:      ~: python simulator.py

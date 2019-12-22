import numpy as np
from math import sqrt

class Agent(object):

    def __init__(self, csvParameters,compiled_besier_curves):
        """ 
            Takes an input line from the csv file,  
            and initializes the agent
        """
        self.id = int(csvParameters[0]) # the id of the agent
        self.gid = int(csvParameters[1]) # the group id of the agent
        self.pos = np.array([float(csvParameters[2]), float(csvParameters[3])]) # the position of the agent 
        self.vel = np.zeros(2) # the velocity of the agent
        self.goal = np.array([float(csvParameters[4]), float(csvParameters[5])]) # the goal of the agent
        self.prefspeed = float(csvParameters[6]) # the preferred speed of the agent
        self.gvel = self.goal-self.pos # the goal velocity of the agent
        self.gvel = self.gvel/(sqrt(self.gvel.dot(self.gvel )))*self.prefspeed       
        self.maxspeed = float(csvParameters[7]) # the maximum sped of the agent
        self.radius = float(csvParameters[8]) # the radius of the agent
        self.goalRadiusSq =1 # parameter to determine if agent is close to the goal
        self.atGoal = False # has the agent reached its goal?
        self.ksi = 1 # the relaxation time used to compute the goal force
        self.dhor = 4000 # the sensing radius
        self.timehor = 90 # the time horizon for computing avoidance forces
        self.F = np.zeros(2) # the total force acting on the agent
        self.maxF = 200 # the maximum force that can be applied to the agent
        self.dhorSq = self.dhor*self.dhor # the sensing radius squared
        self.entrancex = compiled_besier_curves[int(csvParameters[9])*2] #entrance cure x co-ordinate waypoints
        self.entrancey = compiled_besier_curves[int(csvParameters[9])*2+1] #entrance cure y co-ordinate waypoints
        self.exitx = compiled_besier_curves[int(csvParameters[10])*2] #exit cure x co-ordinate waypoints
        self.exity = compiled_besier_curves[int(csvParameters[10])*2+1] #exit cure y co-ordinate waypoints
        self.entry_state = int(csvParameters[9]) #curve number that the agent is using to enter
        self.force_state = int(csvParameters[11]) #this value decides if the force based navigation state is to be executed or not
        self.exit_state = int(csvParameters[10]) #curve number that the agent is using to exit
        self.local_goal = [] #to store local goals/waypoints 
        self.rel_posi = [] #to store relative position between two agents
        self.n_bez = [] #direction vector for agent following bezier curve
    
    def ttc(self, neighbor, eps=.5): #time to collision function
        r = self.radius + neighbor.radius
        w = self.pos - neighbor.pos
        c = w.dot(w) - r*r
        if c < 0: # collision!
            print("Ouch")
            return 0
        v = self.vel - neighbor.vel 
        a = v.dot(v) - eps*eps
        b = w.dot(v) - eps*r
        if b > 0: # agents are diverging
            return float('inf')
        discr = b*b - a*c
        if discr<=0: 
            return float('inf')
        tau = c/(-b + sqrt(discr)) # smallest root
        if tau < 0:
             return float('inf')
        return tau

    def computeForces(self, neighbors=[]): #computing forces to drive the agents and avoid collisions
        """ 
            Your code to compute the forces acting on the agent. 
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors
        """       
        if not self.atGoal:
            if self.entry_state % 2 == 0 and len(self.entrancex) > 0 and self.id != 4 : #checks if assigned curve is entry and switches to state 1 to follow entry bezier curve
                time2=0.5 # time used to calculate driving force                        
                self.local_goal = [self.entrancex[0], self.entrancey[0]] #assigning waypoint as goal
                self.rel_posi = self.local_goal - self.pos #calculating relative position between agents
                self.n_bez = (self.rel_posi + (self.prefspeed*time2))/(abs(self.rel_posi + (self.prefspeed*time2))) #calculating direction vector
                self.F = ((max(self.timehor - time2/100, 0)/time2)*self.n_bez) #driving force
                self.entrancex = np.delete(self.entrancex,0) #eliminating the used waypoints from the list 
                self.entrancey = np.delete(self.entrancey,0) #eliminating the used waypoints from the list 
                
            elif self.force_state == 1 and (abs(self.pos[0] - self.goal[0]) >400 or abs(self.pos[1] - self.goal[1]) >400): #checks if force-based navigation is assigned, switches to state 2
                self.F = (self.gvel-self.vel)/self.ksi #driving force
                for neighbor in neighbors:
                    if neighbor.id != self.id: #and not neighbor.atGoal: 
                        distSq = (neighbor.pos-self.pos).dot(neighbor.pos-self.pos)
                        #print(distSq, self.dhorSq)
                        if distSq < self.dhorSq: # neighbor is inside the sensing radius
                            tau = self.ttc(neighbor)
                            #print(tau, self.timehor)
                            if tau < self.timehor: # will the two agents collide in less than timehor?
                                dir = self.pos + self.vel*tau - neighbor.pos - neighbor.vel*tau 
                                length = sqrt(dir.dot(dir))
                                if length > 0:
                                    dir = dir/length # the direction of the force
                                mag = (self.timehor - tau)/(tau + 1e-6) # the magnitude of the force
                                self.F += mag*dir # add the force
                                
            else: #state 3 - following the exit bezier curve
                time2=0.5 # time used to calculate driving force
                self.local_goal = [self.exitx[0], self.exity[0]]
                if abs(sqrt((self.local_goal - self.pos).dot((self.local_goal - self.pos)))) >10: #to reach first point of exit curve from agents previous state position
                    self.F = ((self.local_goal - self.pos)/(sqrt((self.local_goal - self.pos).dot((self.local_goal - self.pos) )))*self.prefspeed)/self.ksi
                else:
                    self.rel_posi = self.local_goal - self.pos #calculating relative position between agents
                    self.n_bez = (self.rel_posi + (self.prefspeed*time2))/(abs(self.rel_posi + (self.prefspeed*time2)))
                    self.F = ((max(self.timehor - time2/100, 0)/time2)*self.n_bez)
                    #print(self.pos, self.local_goal)
                    if len(self.exitx) > 1 :
                        self.exitx = np.delete(self.exitx,0)
                        self.exity = np.delete(self.exity,0)
        
# =============================================================================
#         # cap the final force
#         mag = sqrt(self.F.dot(self.F)) 
#         if mag > self.maxF: 
#             self.F = self.maxF*self.F/mag 
# =============================================================================
                     

    def update(self, dt):
        """ 
            Code to update the velocity and position of the agents.  
            as well as determine the new goal velocity 
        """
        if not self.atGoal:
            self.vel += self.F*dt     # update the velocity
            mag = sqrt(self.vel.dot(self.vel)) 
            if mag > self.maxspeed: self.vel = self.maxspeed*self.vel/mag # cap the velocity to max speed
            self.pos += self.vel*dt   #update the position
        
            # compute the goal velocity for the next time step. do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq: 
                self.atGoal = True  # goal has been reached
            else: 
                self.gvel = self.gvel/sqrt(distGoalSq)*self.prefspeed  
                

            
  
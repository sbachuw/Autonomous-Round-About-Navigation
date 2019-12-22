import numpy as np
from tkinter import *
import time
import functools
from agent import Agent
from scipy.misc import comb

"""
    Initalize parameters to run a simulation
"""
dt = 0.6 # the simulation time step
scenarioFile='crossing_agents.csv'
doExport = False # export the simulation?
agents = [] # the simulated agents
trajectories = [] # keep track of the agents' traces
ittr = 0 # keep track of simulation iterations 
maxIttr = 1500  #how many time steps we want to simulate
globalTime = 0  # simuation time      
reachedGoals = False # have all agents reached their goals

""" 
    Drawing parameters
"""
pixelsize = 1024 
framedelay = 1
drawVels = True
QUIT = False
paused = False
step = False
circles = []
print_bezier = False #True if you want to print bezier curves
velLines = []
gvLines = []
ids = []

def readScenario(fileName):
    """
        Read a scenario from a file
    """
    
    fp = open(fileName, 'r')
    lines = fp.readlines()
    fp.close()
    for line in lines:
        agents.append(Agent(line.split(','),compiled_besier_curves)) # create an agent and add it to the list
    
    # define the boundaries of the environment
    positions = [a.pos for a in agents]
    goals = [a.goal for a in agents]



def initWorld(canvas):
    """
        initialize the agents 
    """
    print ("")
    print ("Simulation of Agents on a 2D plane.")
    print ("Green Arrow is Goal Velocity, Red Arrow is Current Velocity")
    print ("SPACE to pause, 'S' to step frame-by-frame, 'V' to turn the velocity display on/off.")
    print ("")
       
    colors = ["#FAA", "blue","yellow", "white"]
    for a in agents:
        circles.append(canvas.create_oval(0, 0, a.radius, a.radius, fill=colors[a.gid%4])) # color the disc of an agenr based on its group id
        velLines.append(canvas.create_line(0,0,10,10,fill="red"))
        gvLines.append(canvas.create_line(0,0,10,10,fill="green"))
        ids.append(canvas.create_text(a.pos[0],a.pos[1],text =str(a.id)))
      
def drawWorld():
    """
        draw the agents
    """

    for i in range(len(agents)):
        agent = agents[i]
        if not agent.atGoal:
            canvas.coords(circles[i],(agent.pos[0]- agent.radius), (agent.pos[1] - agent.radius), (agent.pos[0] + agent.radius), (agent.pos[1] + agent.radius))
            canvas.coords(velLines[i],(agent.pos[0]), (agent.pos[1]), (agent.pos[0]+ agent.radius*agent.vel[0]), (agent.pos[1] + agent.radius*agent.vel[1]))
            canvas.coords(gvLines[i],(agent.pos[0]), (agent.pos[1]), (agent.pos[0]+ agent.radius*agent.gvel[0]), (agent.pos[1] + agent.radius*agent.gvel[1]))
            canvas.coords(ids[i],(agent.pos[0]), (agent.pos[1]))
            if drawVels:
                canvas.itemconfigure(velLines[i], state="normal")
                canvas.itemconfigure(gvLines[i], state="normal")
            else:
                canvas.itemconfigure(velLines[i], state="hidden")
                canvas.itemconfigure(gvLines[i], state="hidden")

def on_key_press(event):
    """
        keyboard events
    """                    
    global paused, step, QUIT, drawVels

    if event.keysym == "space":
        paused = not paused
    if event.keysym == "s":
        step = True
        paused = False
    if event.keysym == "v":
        drawVels = not drawVels
    if event.keysym == "Escape":
        QUIT = True

def updateSim(dt):
    """
        Update the simulation 
    """

    global reachedGoals
   
    # compute the forces acting on each agent
    for agent in agents:
        agent.computeForces(agents)
    
    
    reachedGoals = True    
    for agent in agents:
        agent.update(dt)
        if not agent.atGoal:
            reachedGoals = False


def drawFrame(dt):
    """
        simulate and draw frames 
    """

    global start_time,step,paused,ittr,globalTime

    if reachedGoals or ittr > maxIttr or QUIT: #Simulation Loop
        print("%s itterations ran ... quitting"%ittr)
        win.destroy()
    else:
        elapsed_time = time.time() - start_time
        start_time = time.time()
        if not paused:
            updateSim(dt)
            ittr += 1
            globalTime += dt
            for agent in agents:
                if not agent.atGoal:
                   trajectories.append([agent.id, agent.gid, agent.pos[0], agent.pos[1], agent.vel[0], agent.vel[1], agent.radius, globalTime])

        drawWorld()
        if step == True:
            step = False
            paused = True    
        
        win.title('Multi-Agent Navigation')
        win.after(framedelay,lambda: drawFrame(dt))


def bernstein_poly(i, n, t):
    """
     The Bernstein polynomial of n, i as a function of t
    """

    return comb(n, i) * ( t**(n-i) ) * (1 - t)**i


def bezier_curve(points, nTimes=1000):
    """
       Given a set of control points, return the
       bezier curve defined by the control points.

       points should be a list of lists, or list of tuples
       such as [ [1,1], 
                 [2,3], 
                 [4,5], ..[Xn, Yn] ]
        nTimes is the number of time steps, defaults to 1000

        See http://processingjs.nihongoresources.com/bezierinfo/
    """

    nPoints = len(points)
    xPoints = np.array([p[0] for p in points])
    yPoints = np.array([p[1] for p in points])

    t = np.linspace(0.0, 1.0, nTimes)

    polynomial_array = np.array([ bernstein_poly(i, nPoints-1, t) for i in range(0, nPoints)   ])

    xvals = np.dot(xPoints, polynomial_array)
    yvals = np.dot(yPoints, polynomial_array)

    return xvals, yvals  

#=======================================================================================================================
# Main execution of the code
#=======================================================================================================================


# set the visualizer
win = Tk()
# keyboard interaction
win.bind("<space>",on_key_press)
win.bind("s",on_key_press)
win.bind("<Escape>",on_key_press)
win.bind("v",on_key_press)
# the drawing canvas
canvas = Canvas(win, width=pixelsize, height=pixelsize, background="#666")
canvas.create_oval(200,200,824,824) #outside circle
canvas.create_oval(424,424,600,600) #inside circle
canvas.create_oval(312,312,712,712, dash=(3,5)) #middle circle
#North lane
canvas.create_line(400, 0, 400, 220) #left line
canvas.create_line(624, 0, 624, 220) #right line
canvas.create_line(512, 0, 512, 200 ,dash=(3,5)) #middle line
#South lane
canvas.create_line(400, 1024, 400, 804) #left line
canvas.create_line(624, 1024, 624, 804) #right line
canvas.create_line(512, 1024, 512, 824,dash=(3,5)) #middle line
#West lane
canvas.create_line(0, 400, 220, 400) #top line
canvas.create_line(0, 624, 220, 624) #bottom line
canvas.create_line(0, 512, 200, 512,dash=(3,5)) #middle line
#East lane
canvas.create_line(1024, 400, 804, 400) #top line
canvas.create_line(1024, 624, 804, 624) #bottom line
canvas.create_line(1024, 512, 824, 512,dash=(3,5)) #middle line

nTimes = 400 #number of points for bezier curves
#West Lane Enter Basier Points
Wpts_Enter = [[0,568], [120,568],[200,512],[304,644]]
Wxvals_Enter, Wyvals_Enter = bezier_curve(Wpts_Enter, nTimes=nTimes) #generating points from bezier_curve function based on given points above
Wxvals_Enter = np.flip(Wxvals_Enter) #flip array so that agent will receive correct sequence
Wyvals_Enter = np.flip(Wyvals_Enter)
# if print_bezier flag is true it will print the oringal points given to bezier curve generator and will also print entire Bezier curve
if print_bezier == True:
    for a in range(len(Wpts_Enter)):
        canvas.create_oval(Wpts_Enter[a][0], Wpts_Enter[a][1], Wpts_Enter[a][0], Wpts_Enter[a][1], width = 5, fill = 'yellow') #printing orginal points given to bezier curve generator
    for a in range(nTimes-1):
        canvas.create_line(Wxvals_Enter[a],Wyvals_Enter[a],Wxvals_Enter[a+1],Wyvals_Enter[a+1], fill='light green',arrow=LAST) #printing bezier curve with arrows indicating which direction the sequence is
#West Lane Exit Basier Points
Wpts_Exit = [[0,456], [120,456],[200,512],[304,380]]
Wxvals_Exit, Wyvals_Exit = bezier_curve(Wpts_Exit, nTimes=nTimes)
if print_bezier == True:
    for a in range(len(Wpts_Exit)):
        canvas.create_oval(Wpts_Exit[a][0], Wpts_Exit[a][1], Wpts_Exit[a][0], Wpts_Exit[a][1], width = 5, fill = 'yellow')
    for a in range(nTimes-1):
        canvas.create_line(Wxvals_Exit[a],Wyvals_Exit[a],Wxvals_Exit[a+1],Wyvals_Exit[a+1], fill='red',arrow=LAST)

#East Lane Enter Basier Points
Epts_Enter = [[1024,456], [904,456], [824,512],[720,380]]
Exvals_Enter, Eyvals_Enter = bezier_curve(Epts_Enter, nTimes=nTimes)
Exvals_Enter = np.flip(Exvals_Enter)
Eyvals_Enter = np.flip(Eyvals_Enter)
if print_bezier == True:
    for a in range(len(Epts_Enter)):
        canvas.create_oval(Epts_Enter[a][0], Epts_Enter[a][1], Epts_Enter[a][0], Epts_Enter[a][1], width = 5, fill = 'yellow')
    for a in range(nTimes-1):
        canvas.create_line(Exvals_Enter[a],Eyvals_Enter[a],Exvals_Enter[a+1],Eyvals_Enter[a+1], fill='light green',arrow=LAST)
#East Lane Exit Basier Points    
Epts_Exit = [[1024,568], [904,568],[824,512],[720,644]]
Exvals_Exit, Eyvals_Exit = bezier_curve(Epts_Exit, nTimes=nTimes)
if print_bezier == True: 
    for a in range(len(Epts_Exit)):
        canvas.create_oval(Epts_Exit[a][0], Epts_Exit[a][1], Epts_Exit[a][0], Epts_Exit[a][1], width = 5, fill = 'yellow')
    for a in range(nTimes-1):
        canvas.create_line(Exvals_Exit[a],Eyvals_Exit[a],Exvals_Exit[a+1],Eyvals_Exit[a+1], fill='red',arrow=LAST)

#North Lane Enter Basier Points
Npts_Enter = [[456,0], [456,120], [512,200],[380,304]]
Nxvals_Enter, Nyvals_Enter = bezier_curve(Npts_Enter, nTimes=nTimes)
Nxvals_Enter = np.flip(Nxvals_Enter)
Nyvals_Enter = np.flip(Nyvals_Enter)
if print_bezier == True: 
    for a in range(len(Npts_Enter)):
        canvas.create_oval(Npts_Enter[a][0], Npts_Enter[a][1], Npts_Enter[a][0], Npts_Enter[a][1], width = 5, fill = 'yellow')
    for a in range(nTimes-1):
        canvas.create_line(Nxvals_Enter[a],Nyvals_Enter[a],Nxvals_Enter[a+1],Nyvals_Enter[a+1], fill='light green',arrow=LAST)

#North Lane Exit Basier Points   
Npts_Exit = [[568,0], [568,120],[512,200],[644,304]]
Nxvals_Exit, Nyvals_Exit = bezier_curve(Npts_Exit, nTimes=nTimes)
if print_bezier == True:
    for a in range(len(Npts_Exit)):
        canvas.create_oval(Npts_Exit[a][0], Npts_Exit[a][1], Npts_Exit[a][0], Npts_Exit[a][1], width = 5, fill = 'yellow')
    for a in range(nTimes-1):
        canvas.create_line(Nxvals_Exit[a],Nyvals_Exit[a],Nxvals_Exit[a+1],Nyvals_Exit[a+1], fill='red',arrow=LAST)

#South Lane Enter Basier Points
Spts_Enter = [[568,1024], [568,904], [512,824],[644,720]]
Sxvals_Enter, Syvals_Enter = bezier_curve(Spts_Enter, nTimes=nTimes)
Sxvals_Enter = np.flip(Sxvals_Enter)
Syvals_Enter = np.flip(Syvals_Enter)
if print_bezier == True:
    for a in range(len(Spts_Enter)):
        canvas.create_oval(Spts_Enter[a][0], Spts_Enter[a][1], Spts_Enter[a][0], Spts_Enter[a][1], width = 5, fill = 'yellow',)
    for a in range(nTimes-1):
        canvas.create_line(Sxvals_Enter[a],Syvals_Enter[a],Sxvals_Enter[a+1],Syvals_Enter[a+1], fill='light green',arrow=LAST)
#South Lane Exit Basier Points   
Spts_Exit = [[456,1024], [456,904],[512,824],[380,720]]
Sxvals_Exit, Syvals_Exit = bezier_curve(Spts_Exit, nTimes=nTimes)
if print_bezier == True:
    for a in range(len(Spts_Exit)):
        canvas.create_oval(Spts_Exit[a][0], Spts_Exit[a][1], Spts_Exit[a][0], Spts_Exit[a][1], width = 5, fill = 'yellow')
    for a in range(nTimes-1):
        canvas.create_line(Sxvals_Exit[a],Syvals_Exit[a],Sxvals_Exit[a+1],Syvals_Exit[a+1], fill='red',arrow=LAST)

#compiled matrix of all x and y positions of bezier curves at each entry and exit. List goes NSEW    
compiled_besier_curves = np.array([Nxvals_Enter, Nyvals_Enter, Nxvals_Exit, Nyvals_Exit, Sxvals_Enter, Syvals_Enter,Sxvals_Exit,Syvals_Exit, Exvals_Enter,Eyvals_Enter,Exvals_Exit,Eyvals_Exit,Wxvals_Enter,Wyvals_Enter,Wxvals_Exit,Wyvals_Exit])
# array of starting and ending points for agents
start_end_positions = np.array([[456,0],[568,0],[568,1024],[456,1024],[1024,456],[1024,568],[0,568],[0,456]])

readScenario(scenarioFile)
canvas.pack()
initWorld(canvas)
start_time = time.time()
# the main loop of the program
win.after(framedelay, lambda: drawFrame(dt))
mainloop()
if doExport:
    header = "id,gid,x,y,v_x,v_y,radius,time"
    exportFile = scenarioFile.split('.csv')[0] + "_sim.csv"
    np.savetxt(exportFile, trajectories, delimiter=",", fmt='%d,%d,%f,%f,%f,%f,%f,%f', header=header, comments='')

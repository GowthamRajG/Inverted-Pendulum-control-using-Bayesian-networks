# Description: A cartpole implementation in Bayesian Network using Binary data tree structure.
# Basecode credits - https://github.com/craig-corcoran

import math
import random as pr
import matplotlib.pyplot as plt
from matplotlib import animation 
import numpy as np
from matplotlib.patches import Rectangle

class CartPole(object):

    def __init__(self, x = 0.0, xdot = 0.0, theta = 0.0, thetadot = 0.0):
        self.x = x
        self.xdot = xdot
        self.theta = theta
        self.thetadot = thetadot

        # some constants
        self.gravity = 9.8
        self.masscart = 1.0
        self.masspole = 0.1
        self.total_mass = (self.masspole + self.masscart)
        self.length = 0.5		  # actually half the pole's length
        self.polemass_length = (self.masspole * self.length)
        self.force_mag = 10.0
        self.tau = 0.02		  # seconds between state updates
        self.fourthirds = 1.3333333333333

	#code for bayesian update
    def posterior(self,direction,posteriorside):
        probability_left = 0.1
        probability_right = 0.1
        posterior_left = posteriorside
        posterior_right = posteriorside			
        if (direction==1):       #right
           # posterior_right = posteriorside
           posterior_right = posterior_right * probability_right
           return posterior_right
        elif (direction==-1):      #left
           # posterior_left = posteriorside
           posterior_left = posterior_left * probability_left
           return posterior_left
        else:
           return 1
		   
    def failure(self):
        twelve_degrees = 0.2094384
        if (not -2.4 < self.x < 2.4) or (not -twelve_degrees < self.theta < twelve_degrees):
            return True
        else:
            return False

    def reset(self):
        self.x,self.xdot,self.theta,self.thetadot = (0.0,0.0,0.0,0.0)

    def random_policy(self, *args):
        return pr.choice([0,1])

    def single_episode(self, policy = None):
        self.reset()
        if policy is None: policy = self.random_policy

        trace = []
        next_action = policy(self.x,self.xdot,self.theta,self.thetadot)
        while not self.failure():
            pstate, paction, reward, state = self.move(next_action)
            next_action = policy(self.x,self.xdot,self.theta,self.thetadot)
            trace.append([pstate, paction, reward, state, next_action])

        return trace

    def state(self): # get boxed version of the state as per the original code

        one_degree = 0.0174532
        six_degrees = 0.1047192
        twelve_degrees = 0.2094384
        fifty_degrees = 0.87266
        pendulum_direction = 0
        posterior_leftvalue = 1.0
        posterior_rightvalue = 1.0		
			
        if (self.x>0):
            # try:
                pendulum_direction = 1
                posterior_rightvalue = self.posterior(pendulum_direction, posterior_rightvalue)
            # except ValueError:
                # continue
            # break
			   
        if (self.x<0):
            # try:
                pendulum_direction = -1
                posterior_leftvalue = self.posterior(pendulum_direction, posterior_leftvalue)
            # except ValueError:
                # continue
            # break
			   
        if (not -2.4 < self.x < 2.4) or (not -twelve_degrees < self.theta < twelve_degrees):
            return -1

        box = 0
        
        if posterior_rightvalue < 0.0001:
		   
            if self.x < -0.8:
                box = 0
				
            if self.xdot < -0.5:
                pass
				
        if posterior_rightvalue > -0.0001:
		    
            if self.x < 0.8:
                box = 1
            else:
                box = 2
				
            if self.xdot < 0.5:
                box += 3
            else:
                box += 6
			
        if (self.x>0.8):
            box = 2
			
        if (self.x>0.5):
            box = 6
			
        if self.theta < -six_degrees:
            pass
        elif self.theta < -one_degree:
            box += 9
        elif self.theta < 0:
            box += 18
        elif self.theta < one_degree:
            box += 27
        elif self.theta < six_degrees:
            box += 36
        else:
            box += 45
        
        if self.thetadot < -fifty_degrees:
            pass
        elif self.thetadot < fifty_degrees:
            box += 54
        else:
            box += 108
        
        print (posterior_leftvalue)
        print (posterior_rightvalue)
        plt.plot(self.x, box)
        return box;
    plt.show()
    def reward(self):
        if self.failure():
            return -1.0
        else:
            return 0.0
			
    def animate_pendulum(self, t, state, length, filename=None):
        """ Animates the inverted pendulum given the time interval and states, 
            optionally saves it to file if the filename is given.
        
        If true a movie file will be saved of the animation, it may take some time.
        @param t        Time array
        @param states   States of the system
        @param length   Length of the pendulum links
        @param filename Name of the filename (optional)
        @return         Figure containing the plot and the animation as a tuple
        Credit: Jason Moore's tutorial on inverted pendulum using sympy.
        URL: http://www.moorepants.info/blog/npendulum.html
        """
        
        # the number of pendulum bobs
        numpoints = 2
        
        # first set up the figure, the axis, and the plot elements we want to animate
        fig = plt.figure()
        
        # some dimesions
        cart_width = 0.4
        cart_height = 0.2
        
        # set the limits based on the motion
        # xmin = np.around(self.state[:, 0].min() - cart_width / 2.0, 1)
        # xmax = np.around(self.state[:, 0].max() + cart_width / 2.0, 1)
        
        # create the axes
        ax = plt.axes(xlim=(0, 1), ylim=(-1.1, 1.1), aspect='equal')
        
        # display the current time
        time_text = ax.text(0.04, 0.9, '', transform=ax.transAxes)
        
        # create a rectangular cart
        rect = Rectangle((0,0),cart_width, cart_height, fill=True, color='red', ec='black')
        ax.add_patch(rect)
        
        # blank line for the pendulum
        line, = ax.plot([], [], lw=2, marker='o', markersize=6)
        
        # initialization function: plot the background of each frame
        def init():
            time_text.set_text('')
            rect.set_xy((0.0, 0.0))
            line.set_data([], [])
            return time_text, rect, line,
        
        # animation function: update the objects
        def animate(i):
            #time_text.set_text('time = {:2.2f}'.format(t[i]))
            rect.set_xy(((0,0), 2))
            x = np.hstack(((0,1), np.zeros((numpoints - 1))))
            y = np.zeros((numpoints))
            for j in np.arange(1, numpoints):
                x[j] = x[j - 1] + self.length * np.cos(0)
                y[j] = y[j - 1] + self.length * np.sin(0)
            line.set_data(x, y)
            return time_text, rect, line,
        
        print (t[0])
        print (t[1])
        print (t[2])
        print (t[-1])
		
        # call the animator function
        anim = animation.FuncAnimation(fig, animate, frames=len(t), init_func=init,
                interval=1000, blit=True, repeat=False)
        
        # save the animation if a filename is given
        # if filename is not None:
            # anim.save('Pendulum.mp4', fps=30, codec='libx264')
            # display_animation(anim)
        plt.show()
		
    def move(self, action, boxed=False): # binary L/R action
        force = 0.0
        if action > 0:
            force = self.force_mag
        else:
            force = -self.force_mag

        costheta = math.cos(self.theta)
        sintheta = math.sin(self.theta)

        tmp = (force + self.polemass_length * (self.thetadot ** 2) * sintheta) / self.total_mass;
        thetaacc = (self.gravity * sintheta - costheta * tmp) / (self.length * (self.fourthirds - self.masspole * costheta ** 2 / self.total_mass))
        xacc = tmp - self.polemass_length * thetaacc * costheta / self.total_mass

        (px,pxdot,ptheta,pthetadot) = (self.x, self.xdot, self.theta, self.thetadot)
        pstate = self.state()

        self.x += self.tau * self.xdot
        self.xdot += self.tau * xacc
        self.theta += self.tau * self.thetadot
        self.thetadot += self.tau * thetaacc

        if boxed:
            return pstate, action, self.reward(), self.state()
        else:
            return [px,pxdot,ptheta,pthetadot],action,self.reward(),[self.x,self.xdot, self.theta, self.thetadot]

if __name__ == '__main__':

    cp = CartPole()

    global ccount
    ccount = 1

    def alternating_policy(x,xdot,theta,thetadot):
        global ccount
        ccount = ccount + 1
        if ccount % 2 == 0:
            return 1
        else:
            return 0


    if False:
        for i in range(100):
            print (cp.update(i % 2), cp.state())
        for i in range(100):
            print (cp.update(i % 2), cp.state())

    if True:
        t = cp.single_episode()
        for i in t:
            print (i[2],i[1])

        t = cp.single_episode(alternating_policy)
        for i in t:
            print (i[2],i[1])
			
    cp.animate_pendulum(t, cp.state, cp.length, filename="C:/Users/Gowtham Raj/Desktop/Python/project/Pendulum.mp4")
			

		
    plt.show()
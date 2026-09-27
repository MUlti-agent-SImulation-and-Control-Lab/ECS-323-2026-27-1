import numpy as np
from matplotlib import pyplot as pt

class PendulumSMC:
    # x1. = x2
    # x2. = h(x)+g(x)u
    # s = a*x1 +x2
    # u = -beta*sgn(s)
    # beta >= |a*x2 + h(x)|/g(x) + beta0
    # rho = |a*x2 + h(x)|/g(x)
    # epsilon = boundary layer thickness (to handle chattering) -eps<s<eps
    # u = -beta* tanh(s/epsilon) for chattering replace abrupt signum with smooth sigmoid like cuve
    def __init__(self, a = 2, beta0 = 1.5, epsilon = 0.05, grav = 9.81):
        self.a = float(a)
        self.beta0 = float(beta0)
        self.epsilon = float(epsilon)
        self.grav = float(grav)
        self.m_min, self.m_max = 0.05, 0.2
        self.k_min, self.k_max = 0.0, 0.05
        self.l_min, self.l_max = 0.9, 1.1
        self.gx_min = 1/(self.m_min*(self.l_min**2))
    #eqn for pendulum
        # x1. = x2
        # x2. = (-g/l)*sin(x1) - (k/m)*x2 + (1/ml**2)u
        #assumption
        #.05<=m<=.2
        #.9<=l<=1.1
        #0<=k<=.05
        # |x2|<=pi
        # |x1+x2|<=pi 
    #h(x)=(-g/l)*sin(x1) - (k/m)*x2
    #g(x)= (1/ml**2)
    def sliding_var(self, x):
        x1, x2 = x[0], x[1]
        return self.a*x1 + x2
    def calc_u(self, x):
        x1, x2 = x[0], x[1]
        s = self.sliding_var(x)
        beta = self.calc_gain(x)
        #to deal with chattering
        if(self.epsilon>0):
            switch_term=np.tanh(s/self.epsilon)
        else:
            switch_term = np.sign(s/self.epsilon)
        u = -beta*switch_term
        return u, s
    def calc_gain(self, x):
        x1, x2 = x[0], x[1]
        # lower bound for |g| = |self.gx_min|
        # upper bound for |ax2+h(x)| 
        # s. = ax2 -g/l sinx1 - k/m x2 => |x2(a-k/m) - g/l sinx1|
        # triangle ineq -> |x2(a-k/m) - g/l sinx1|<=|a-k/m||x2| + |grav/l||sinx1|
        # maximise a-k/m
        max_damp_coeff = max(abs(self.a - self.k_max/self.m_min), abs(self.a - self.k_min/self.m_max))
        #maximise g/l
        max_grav_term = (self.grav/ self.l_min)*abs(np.sin(x1))
        num_bound = max_damp_coeff*abs(x2) + max_grav_term
        rho = num_bound/self.gx_min
        beta = rho + self.beta0
        return beta

class PendulumPlant:
    def __init__(self,m = 0.13, l= 1.0, k= 0.25, grav = 9.81):
        self.m = float(m)
        self.k = float(k)
        self.l = float(l)
        self.grav = float(grav)
    def g(self,x):
        return 1.0/(self.m*(self.l**2))
    def h(self,x):
        return -(self.grav/self.l)*np.sin(x[0]) - (self.k/self.m)*x[1]
    def dyna(self,x,u):
        dx1 = x[1]
        dx2 = self.h(x) + self.g(x)*u
        return np.array([dx1,dx2])
# execution
if __name__ == "__main__":
    dt = 0.001
    t_final = 10
    timestamps = np.arange(0.0, t_final, dt)
    no_of_ts = len(timestamps)

    smc = PendulumSMC(1.0,0.5,0.05)
    plant = PendulumPlant(0.18, 1.05, 0.04)
    u = np.zeros(no_of_ts)
    s = np. zeros(no_of_ts)
    x= np.zeros((no_of_ts,2))
           # pos  vel
    x[0] = [2.0, -1]   #x1< pi & x2+x1 <pi
    for i in range(no_of_ts-1):
        u[i],s[i] = smc.calc_u(x[i])
        #for solution of differential eqns, computers need to do it in discrete timwe steps.
        #method for computing integral 
        #next_state = curr_st + dt*slope(current)
        slope1 = plant.dyna(x[i], u[i])
        #slope of midpoint of 1st and second point
        slope2 = plant.dyna(x[i]+0.5*dt*slope1, u[i])
        slope3 = plant.dyna(x[i]+ 0.5*dt*slope2, u[i])
        slope4 = plant.dyna(x[i]+dt*slope3, u[i])
        # using taylor series
        x[i+1] = x[i]+(dt/6)*(slope1 + 2*slope2 + 2*slope3 + slope4)
    u[-1],s[-1] = smc.calc_u(x[-1])
    # plot
    fig, axs = pt.subplots(3,1,sharex=True)
    axs[0].plot(timestamps,x[:,0],label=r'$\theta$ ($x_1$)')
    axs[0].plot(timestamps, x[:, 1], label=r'$\dot{\theta}$ ($x_2$)')
    axs[0].axhline(np.pi, color='gray', linestyle=':', label=r'$\pm\pi$ bound')
    axs[0].axhline(-np.pi, color='gray', linestyle=':')
    axs[0].set_ylabel('States [rad, rad/s]')
    axs[0].grid(True)
    axs[0].legend(loc='upper right')

    axs[1].plot(timestamps, s, color='orange', label=r'$s(t) = x_1 + x_2$')
    axs[1].axhline(np.pi, color='gray', linestyle=':')
    axs[1].axhline(-np.pi, color='gray', linestyle=':')
    axs[1].axhline(0, color='black', linestyle='--', alpha=0.5)
    axs[1].set_ylabel('Sliding Variable $s$')
    axs[1].grid(True)
    axs[1].legend(loc='upper right')

    axs[2].plot(timestamps, u, color='green', label=r'Torque $u(t)$')
    axs[2].set_ylabel('Control Input [N·m]')
    axs[2].set_xlabel('Time (s)')
    axs[2].grid(True)
    axs[2].legend(loc='upper right')

    pt.tight_layout()
    pt.show()
import pybullet as p
import pybullet_data
import numpy as np
import time
import os


PPRZ_HOME = os.getenv("PAPARAZZI_HOME", os.getcwd())
PYBULLET_CONF_PATH = os.path.join(PPRZ_HOME, "conf/simulator/pybullet")


class BulletFDM():
    def __init__(self, dt=0.02, GUI=True, urdf="coaxial_donut.urdf"):
        print(f"Hello from PyBullet ! dt={dt}, GUI={GUI}")

        pprz_src = os.getenv("PAPARAZZI_SRC")
        print(f"pprz_src = {pprz_src}")

        # self.last_time_print = 0
        
        # dt : Time step
        self.dt=dt
        if GUI:
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0) # remove useless graphical elements from GUI

        # Add path for pybullet assests e.g : plane and grid...
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        self.planeId = p.loadURDF("plane.urdf")
        self.textureId = p.loadTexture("checker_grid.jpg")

        self.vehicle_start_pos = [0, 0, 0.2]
        self.vehicle_start_orientation = p.getQuaternionFromEuler([0, 0, np.pi/2.])

        vehicule_urdf = os.path.join(PYBULLET_CONF_PATH, urdf)
        self.vehicle = p.loadURDF(vehicule_urdf, self.vehicle_start_pos, self.vehicle_start_orientation)

        # Bullet physics uses ENU frame
        p.setGravity(0, 0, -9.81,  physicsClientId=self.physicsClient)
        # Real time simulation should be off in order to apply external force and moments !
        p.setRealTimeSimulation(0,   physicsClientId=self.physicsClient)
        # dt : Time step
        p.setTimeStep(self.dt, physicsClientId=self.physicsClient)
        # Orient the camera if needed
        p.resetDebugVisualizerCamera(cameraDistance=3.5, cameraYaw=-80, cameraPitch=-40, cameraTargetPosition=[0.0, 0.0, 0.0])
        # Vehicle properties TODO use URDF data
        self.pwm2rpm_scale = [20000., 20000.]
        self.KF = 1.60e-7  # 0.01
        self.KM = 1.7e-10 # 0.001
        # Initialte velocity and acceleration vectors
        self.vel = np.zeros(3)
        self.accel = np.zeros(3)
        # Initialte angular velocity and acceleration vectors
        self.ang_vel = np.zeros(3)
        self.ang_accel = np.zeros(3)

        # State
        self.observation = {}

    def apply_force_and_moments(self, motors, servos, use_noise=False):
        ''' rpm = [ coaxial prop rpms ]'''
        rpm = self.pwm2rpm_scale * motors
        #print(f' commands {motors} -> {rpm} | {servos}')
        forces  = np.array(rpm**2)*self.KF
        torques = np.array(rpm**2)*self.KM

        if use_noise:
            f_noise = np.random.normal(0, 0.01, len(rpm))
            m_noise = np.random.normal(0, 0.001,len(rpm))
        else:
            f_noise = np.zeros(2)
            m_noise = np.zeros(2)

        forces  += f_noise
        torques += m_noise

        s_servos = [1., -1.]
        for i, cmd in enumerate(servos):
            deflection = s_servos[i] * cmd * np.deg2rad(15.0)  # cmd is in -1/+1 for radians
            p.resetJointState(self.vehicle, i, deflection)
            #print(f' {i}- deflection : {np.rad2deg(deflection)}')

        s_motors = [1., -1.]
        for i in range(2):
            #print(f' {i}- force : {forces[i]}, torque : {s[i] * torques[i]}')
            p.applyExternalForce(self.vehicle,
                    i + 1, # link 1: prop up, link 2: prop down
                    forceObj=[f_noise[0], f_noise[1], forces[i]],
                    posObj=[0, 0, 0],
                    flags=p.LINK_FRAME,
                    physicsClientId=self.physicsClient
                    )
            p.applyExternalTorque(self.vehicle,
                    i + 1, # link 1: prop up, link 2: prop down
                    torqueObj=[m_noise[0], m_noise[1], s_motors[i] * torques[i]],
                    flags=p.LINK_FRAME,
                    physicsClientId=self.physicsClient
                    )

    def step(self, commands):
        
        if not isinstance(commands, np.ndarray):
          commands = np.array(commands)
        
        #print(commands)
        if commands[0] == 0 and commands[1] == 0:
            return self.get_observation() # hack for not in flight

        # commands: motor up, motor down, tilt up x, tilt up y
        self.apply_force_and_moments(commands[0:2], commands[2:4])
        p.stepSimulation(physicsClientId=self.physicsClient)
        return self.get_observation()

    def get_observation(self):
        # Get observation
        v_pos, v_quat = p.getBasePositionAndOrientation(self.vehicle)
        v_rpy = p.getEulerFromQuaternion(v_quat)
        v_vel, v_ang_v = p.getBaseVelocity(self.vehicle)

        self.accel = (np.array(v_vel) - self.vel)/self.dt
        self.vel = np.array(v_vel)
        self.ang_accel = (np.array(v_ang_v) - self.ang_vel)/self.dt
        self.ang_vel = np.array(v_ang_v)
        R = np.array(p.getMatrixFromQuaternion(v_quat)).reshape(3, 3)
        body_ang_vel = R.T.dot(self.ang_vel)
        body_ang_accel = R.T.dot(self.ang_accel)

        self.observation = {'pos':v_pos,
                            'quat':v_quat,
                            'rpy':v_rpy,
                            'vel':tuple(self.vel),
                            'ang_v':tuple(body_ang_vel),
                            'accel':tuple(self.accel),
                            'ang_accel':tuple(body_ang_accel)
        }
        return self.observation

    def reset(self):
        # Reset the simulation and the state
        # p.resetSimulation(physicsClientId=self.physicsClient) # FIXME : this is not the correct way to reset the simulation
        p.resetBasePositionAndOrientation(self.vehicle, self.vehicle_start_pos, self.vehicle_start_orientation)
        p.resetBaseVelocity(self.vehicle, [0, 0, 0], [0, 0, 0])

        return self.get_observation()



if __name__ == "__main__":
    from time import sleep
    m = BulletFDM(GUI=True)

    # An example simulation loop with random commands generation
    while 1:
        for i in range(200):
            # commands = np.array([10., 10., 10., 10.]) # fixed rpms
            commands = np.random.normal(13., 0.5, 4) # random rpms
            m.step(commands)
            sleep(0.05) # FIXME : checck the computation time and sleep to sync realtime...
        m.reset()


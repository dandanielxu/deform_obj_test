import pybullet as p
import pybullet_data
from paper_1 import Paper

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
p.setRealTimeSimulation(0)
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
p.setGravity(0, 0, -9.8)
plane_id = p.loadURDF(fileName="plane.urdf",
                      basePosition=(0, 0, 0))
p.setTimeStep(1./180.)
paper_1 = Paper(basePosition=[0.05, 0.0, 0.01])
paper_1.randomize()
while True:
    p.stepSimulation()

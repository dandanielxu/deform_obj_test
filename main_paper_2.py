import pybullet as p
physicsClient = p.connect(p.GUI)
import pybullet_data

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

gravZ=-9.8
p.setGravity(0, 0, gravZ)


planeId = p.loadURDF("plane.urdf", [0,0,0])

def _load_softbody(basePos):
    return p.loadSoftBody("paper_2.obj", basePosition = basePos, scale = 0.5, mass = 1., 
                                            useNeoHookean = 0, useBendingSprings=1,useMassSpring=1, 
                                            springElasticStiffness=40, springDampingStiffness=.1, 
                                            springDampingAllDirections = 0, useSelfCollision = 1, 
                                            frictionCoeff = .5, useFaceContact=1, collisionMargin = 0.04)
    
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)


paper_2 = _load_softbody([0,0,0])


while p.isConnected():
    p.stepSimulation()

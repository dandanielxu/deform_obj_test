import pybullet as p
from numpy import pi


class PyBulletObject:
    def __init__(self, urdf_path, pose):
        self.load(urdf_path, pose)

    def load(self, urdf_path, pose):
        self.urdf_path = urdf_path
        self.body_id = p.loadURDF(
            self.urdf_path,
            pose[0], pose[1]
        )

    def delete(self):
        p.removeBody(self.body_id)

    def get_pose(self):
        return p.getBasePositionAndOrientation(self.body_id)


class Cube(PyBulletObject):
    def __init__(self, position, scale, mass):
        self.vs = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[scale] * 3,
            rgbaColor=(1, 0, 0, 1))
        self.body_id = p.createMultiBody(
            baseMass=mass,
            baseVisualShapeIndex=self.vs
        )
        p.resetBasePositionAndOrientation(
            self.body_id,
            position,
            (0, 0, 0, 1)
        )


class Paper(PyBulletObject):

    def __init__(self,
                 basePosition=[0, 0, 0],
                 baseOrientation=p.getQuaternionFromEuler([pi/2, 0, 0]),
                 scale=0.2,
                 mass=1,
                 n_cuts=25):
        self.n_cuts = n_cuts
        edge_length = (2.0 * scale) / (self.n_cuts - 1)
        self.collisionMargin = edge_length*0.25
        self.mass = mass
        self.scale = scale
        self.corners = []
        self.corner_anchors = []
        super().__init__(urdf_path='paper_1.obj',
                         pose=[basePosition, baseOrientation])

    def load(self, urdf_path, pose, kwargs={}):
        self.urdf_path = urdf_path
        self.body_id = p.loadSoftBody(
            fileName=self.urdf_path,
            basePosition=pose[0],
            baseOrientation=pose[1],
            scale=self.scale,
            mass=self.mass,
            collisionMargin=self.collisionMargin,
            # DON'T CHANGE
            useBendingSprings=True,
            useFaceContact=True,
            useMassSpring=True,
            useSelfCollision=True, ###changed
            springDampingAllDirections=True, ##changed
            # CAN CHANGE
            useNeoHookean=False,
            springElasticStiffness=40,
            springDampingStiffness=0.1,
            frictionCoeff=0.5
        )

        # setup corners
        data = p.getMeshData(bodyUniqueId=self.body_id)
        nb_vertices, vert_pos_l = data

        corner_indices = [
            0,
            self.n_cuts-1,
            (self.n_cuts-1)*self.n_cuts,
            self.n_cuts*self.n_cuts-1
        ]

        for corner_idx in corner_indices:
            self.corners.append(
                Cube(position=vert_pos_l[corner_idx],
                     scale=1e-5,
                     mass=1e-5)
            )
            self.corner_anchors.append(
                p.createSoftBodyAnchor(
                    softBodyBodyUniqueId=self.body_id,
                    nodeIndex=corner_idx,
                    bodyUniqueId=self.corners[-1].body_id,
                    linkIndex=-1,
                )
            )

    def randomize(self):
        for corner_idx in range(3):
            corner = self.corners[corner_idx]
            for _ in range(50):
                p.applyExternalForce(corner.body_id,
                                     linkIndex=-1,
                                     forceObj=[0, 0, self.mass*5],
                                     posObj=[0, 0, 0],
                                     flags=p.WORLD_FRAME)
                p.stepSimulation()
            for _ in range(200):
                p.stepSimulation()

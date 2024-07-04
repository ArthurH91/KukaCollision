import numpy as np

import pybullet as p
import hppfcl
import pinocchio as pin

RED = np.array([249, 136, 126, 125]) / 255

def display_ball(M, RADIUS=0.05, COLOR=[1.0, 1.0, 1.0, 1.0]) -> int:
    """
    Create a sphere visual object in PyBullet (no collision)
    Transformed because reference M is in pinocchio WORLD frame, which is different
    than PyBullet WORLD frame if the base placement in the simulator is not (eye(3), zeros(3))
    INPUT:
        M               : desired position of the ball in pinocchio.WORLD
        robot_base_pose : initial pose of the robot BASE in bullet.WORLD
        RADIUS          : radius of the ball
        COLOR           : color of the ball
    """
    # pose of the sphere in bullet WORLD

    quat = pin.SE3ToXYZQUAT(M)
    visualBallId = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=RADIUS,
        rgbaColor=COLOR,
        visualFramePosition=quat[:3],
        visualFrameOrientation=quat[3:],
    )
    ballId = p.createMultiBody(
        baseMass=0.0,
        baseInertialFramePosition=[0.0, 0.0, 0.0],
        baseVisualShapeIndex=visualBallId,
        basePosition=[0.0, 0.0, 0.0],
        useMaximalCoordinates=True,
    )

    return ballId


def transform_model_into_capsules(cmodel):
    """Modifying the collision model to transform the spheres/cylinders into capsules which makes it easier to have a fully constrained robot."""
    collision_model_reduced_copy = cmodel.copy()
    list_names_capsules = []

    # Going through all the goemetry objects in the collision model
    for geom_object in collision_model_reduced_copy.geometryObjects:
        if isinstance(geom_object.geometry, hppfcl.Cylinder):
            # Sometimes for one joint there are two cylinders, which need to be defined by two capsules for the same link.
            # Hence the name convention here.
            if (geom_object.name[:-1] + "capsule_0") in list_names_capsules:
                name = geom_object.name[:-1] + "capsule_" + "1"
            else:
                name = geom_object.name[:-1] + "capsule_" + "0"
            list_names_capsules.append(name)
            placement = geom_object.placement
            parentJoint = geom_object.parentJoint
            parentFrame = geom_object.parentFrame
            geometry = geom_object.geometry
            geom = pin.GeometryObject(
                name,
                parentFrame,
                parentJoint,
                hppfcl.Capsule(geometry.radius, geometry.halfLength),
                placement,
            )
            geom.meshColor = RED
            cmodel.addGeometryObject(geom)
            cmodel.removeGeometryObject(geom_object.name)
        elif (
            isinstance(geom_object.geometry, hppfcl.Sphere)
            and "link" in geom_object.name
        ):
            cmodel.removeGeometryObject(geom_object.name)
    return cmodel
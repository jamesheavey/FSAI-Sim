import math
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
import numpy as np


def quat_to_euler(quat):
    w, x, y, z = quat.w, quat.x, quat.y, quat.z

    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.atan2(t3, t4)

    return {"roll": X, "pitch": Y, "yaw": Z}


def euler_to_quat(roll=0, pitch=0, yaw=0):

    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
        math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
        math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
        math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
        math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)

    return {"x": qx, "y": qy, "z": qz, "w": qw}


def quat_to_rot(quat):
    w, x, y, z = quat.w, quat.x, quat.y, quat.z
    # Ref: https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
    return np.array([
        [1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
        [2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w],
        [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y]
    ])


def transform_to_matrix(transform):
    mat = np.zeros((4, 4))
    mat[0:3, 0:3] = quat_to_rot(transform.rotation)
    mat[:, 3] = np.array([
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
        1
    ])
    return mat


def pose_to_dict(pose):
    return {
        "position": {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z
        },
        "rotation": quat_to_euler(pose.orientation)
    }


def dict_to_pose(d):
    return Pose(
        position=Point(
            **d.get("position", {})),
        orientation=Quaternion(
            **euler_to_quat(**d.get("rotation", {})))
    )


def map_to_dict(m):
    return {
        "centerline": list(map(pose_to_dict, m.centerline)),
        "yellow_cones": list(map(pose_to_dict, m.yellow_cones)),
        "blue_cones": list(map(pose_to_dict, m.blue_cones)),
        "orange_cones": list(map(pose_to_dict, m.orange_cones)),
        "start": pose_to_dict(m.start),
        "name": m.name
    }


def car_to_dict(c):
    return {
        "name": c.name,
        "width": c.width,
        "length": c.length,
        "picture": c.picture
    }


def np_to_pose(v):
    return dict_to_pose({
        "position": {
            "x": v[0],
            "y": v[1]
        }})


def np_to_vector3(v):
    return Vector3(x=v[0], y=v[1], z=v[2])


def metadata_to_dict(metadata):
    def kvp_to_dict(kvps):
        return {kvp.key: kvp.value for kvp in kvps}

    def meta_point_to_dict(point):
        return {
            "properties": kvp_to_dict(point.properties),
            "position": {
                "x": point.position.x,
                "y": point.position.y,
                "z": point.position.z,
            }
        }

    def point_collection_to_dict(point_collection):
        return {
            "properties": kvp_to_dict(point_collection.properties),
            "points": list(map(meta_point_to_dict, point_collection.points))
        }

    return {
        "timestamp": metadata.timestamp,
        "source": metadata.source,
        "properties": kvp_to_dict(metadata.properties),
        "collections": list(map(point_collection_to_dict, metadata.collections))
    }

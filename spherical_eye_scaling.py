import math

import maya.cmds as mc


def compute_distance_from_two_transform_nodes(node1: str, node2: str) -> float:
    """
    Computes the Euclidean distance between two transform nodes.

    Args:
        node1 (str): The name of the first transform node.
        node2 (str): The name of the second transform node.

    Returns:
        float: The distance between the two transform nodes.
    """
    position1 = mc.xform(node1, q=True, t=True, ws=True)
    position2 = mc.xform(node2, q=True, t=True, ws=True)
    return math.sqrt(
        sum(pow(x2 - x1, 2) for x1, x2 in zip(position1, position2))
    )


def main(
    center_guide: str, radius_guide: str, pupil_guide: str, joint_amount: int
):
    """
    Creates an eye joint system based on specified guides and joint amount.

    This function creates an eye joint system,
    using the specified guides to determine the center, radius, and pupil positions.
    Quaternions are used to derive the scales and positions for the joints.
    The Euler angles are converted to quaternions,
    which are then used to set the scale on the X and Y axes and the position on the Z axis.
    The quaternion's X component is used for scaling, while the W component is used for positioning.

    Args:
        center_guide (str): The name of the center guide transform node.
        radius_guide (str): The name of the radius guide transform node.
        pupil_guide (str): The name of the pupil guide transform node.
        joint_amount (int): The number of joints to create.
    """

    eye_joint = mc.createNode("joint")
    mc.matchTransform(eye_joint, center_guide)

    iris_scale_attr_name = "irisScale"
    mc.addAttr(eye_joint, ln=iris_scale_attr_name, min=-1, max=1, k=True)
    iris_scale_attr = f"{eye_joint}.{iris_scale_attr_name}"

    radius = compute_distance_from_two_transform_nodes(
        center_guide, radius_guide
    )

    for index in range(joint_amount):

        current_joint = mc.createNode("joint")
        mc.matchTransform(current_joint, center_guide)
        mc.parent(current_joint, eye_joint)

        if index == (joint_amount - 1):
            pupil_distance = compute_distance_from_two_transform_nodes(
                center_guide, pupil_guide
            )
            current_cos = pupil_distance / radius
        else:
            current_cos = index * (1 / (joint_amount - 1))

        current_degree = math.degrees(math.acos(current_cos))
        default_param = current_degree / 90

        default_param_add = mc.createNode("addDoubleLinear")
        mc.connectAttr(iris_scale_attr, f"{default_param_add}.i1")
        mc.setAttr(f"{default_param_add}.i2", default_param)

        doubled_angle_mult = mc.createNode("multDoubleLinear")
        mc.connectAttr(f"{default_param_add}.o", f"{doubled_angle_mult}.i1")
        mc.setAttr(f"{doubled_angle_mult}.i2", 180)

        angle_clamp = mc.createNode("clamp")
        mc.connectAttr(f"{doubled_angle_mult}.o", f"{angle_clamp}.ipr")
        mc.setAttr(f"{angle_clamp}.mnr", 0)
        mc.setAttr(f"{angle_clamp}.mxr", 190)

        euler_to_quat = mc.createNode("eulerToQuat")
        mc.connectAttr(f"{angle_clamp}.opr", f"{euler_to_quat}.irx")
        mc.connectAttr(f"{euler_to_quat}.oqx", f"{current_joint}.sx")
        mc.connectAttr(f"{euler_to_quat}.oqx", f"{current_joint}.sy")

        position_mult = mc.createNode("multDoubleLinear")
        mc.connectAttr(f"{euler_to_quat}.oqw", f"{position_mult}.i1")
        mc.setAttr(f"{position_mult}.i2", radius)
        mc.connectAttr(f"{position_mult}.o", f"{current_joint}.tz")

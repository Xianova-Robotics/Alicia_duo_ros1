import torch

from .transform_tensor import *


def convert_ori_format(ori, src_format: str, tar_format: str):
    """
    将方向从源格式转换为目标格式。

    :param ori: 可以是四元数、旋转矩阵或欧拉角
    :param src_format: 源格式
    :param tar_format: 目标格式
    :return: 转换后的方向格式
    """
    assert src_format in ['quat', 'mat', 'euler'], "Unsupported source format."
    assert tar_format in ['quat', 'mat', 'euler'], "Unsupported target format."
    if src_format == tar_format:
        return ori
    if src_format == 'quat':
        if tar_format == 'mat':
            return rot_matrix_from_quat_tensor(ori)
        elif tar_format == 'euler':
            return euler_from_quat_tensor(ori)
    elif src_format == 'mat':
        if tar_format == 'quat':
            return quat_from_rot_matrix_tensor(ori)
        elif tar_format == 'euler':
            return euler_from_rot_matrix_tensor(ori)
    elif src_format == 'euler':
        if tar_format == 'quat':
            return quat_from_euler_tensor(ori)
        elif tar_format == 'mat':
            return rot_matrix_from_euler_tensor(ori)


def convert_trans_format(trans, src_format, tar_format):
    """
    将变换格式从源格式转换为目标格式

    :param trans: 变换矩阵或向量
    :param src_format: 源格式，可选 ["trans_mat", "pos_quat", "pos_euler"]
    :param tar_format: 目标格式，可选 ["trans_mat", "pos_quat", "pos_euler"]
    :return: 转换后的变换
    """
    assert src_format in ['trans_mat', 'pos_quat', 'pos_euler'], "Unsupported source format."
    assert tar_format in ['trans_mat', 'pos_quat', 'pos_euler'], "Unsupported target format."

    if src_format == 'trans_mat':
        mat = trans[:, :3, :3]
        pos = trans[:, :3, 3]
        if tar_format == 'pos_quat':
            quat = quat_from_rot_matrix_tensor(mat)
            return torch.cat([pos, quat], dim=1)
        elif tar_format == 'pos_euler':
            euler = euler_from_rot_matrix_tensor(mat)
            return torch.cat([pos, euler], dim=1)
    elif src_format == 'pos_quat':
        pos = trans[:, :3]
        quat = trans[:, 3:]
        if tar_format == 'trans_mat':
            mat = rot_matrix_from_quat_tensor(quat)
            trans_matrix = torch.eye(4).repeat(trans.shape[0], 1, 1).to(trans.device)
            trans_matrix[:, :3, :3] = mat
            trans_matrix[:, :3, 3] = pos
            return trans_matrix
        elif tar_format == 'pos_euler':
            euler = euler_from_quat_tensor(quat)
            return torch.cat([pos, euler], dim=1)
    elif src_format == 'pos_euler':
        pos = trans[:, :3]
        euler = trans[:, 3:]
        if tar_format == 'trans_mat':
            mat = rot_matrix_from_euler_tensor(euler)
            trans_matrix = torch.eye(4).repeat(trans.shape[0], 1, 1)
            trans_matrix[:, :3, :3] = mat
            trans_matrix[:, :3, 3] = pos
            return trans_matrix
        elif tar_format == 'pos_quat':
            quat = quat_from_euler_tensor(euler)
            return torch.cat([pos, quat], dim=1)


def convert_quat_order(quat, src_order, tar_order):
    """
    将四元数顺序从源顺序转换为目标顺序。
    注意：robolab中的四元数顺序默认为'xyzw'。

    :param quat: 四元数张量
    :param src_order: 源顺序，可选 ['wxyz', 'xyzw']
    :param tar_order: 目标顺序，可选 ['wxyz', 'xyzw']
    :return: 转换后的四元数
    """
    assert src_order in ['wxyz', 'xyzw'], "Unsupported source order."
    assert tar_order in ['wxyz', 'xyzw'], "Unsupported target order."
    quat = check_quat_tensor(quat)
    if src_order == tar_order:
        return quat
    if src_order == 'wxyz':
        if tar_order == 'xyzw':
            return quat[:, [1, 2, 3, 0]]
    elif src_order == 'xyzw':
        if tar_order == 'wxyz':
            return quat[:, [3, 0, 1, 2]]

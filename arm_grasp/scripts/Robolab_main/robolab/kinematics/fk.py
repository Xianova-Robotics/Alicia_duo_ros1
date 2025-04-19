def get_fk_from_chain(chain, joint_value, export_link_name=None):
    """
    从运动链计算正向运动学
    :param chain: 运动链对象
    :param joint_value: 关节角度值
    :param export_link_name: 需要输出的连杆名称
    :return: 末端执行器位姿和所有连杆的变换矩阵
    """
    # 计算正向运动学获取变换对象; end_only=False 返回所有连杆的变换字典
    ret = chain.forward_kinematics(joint_value)
    # 查找指定连杆的变换
    if export_link_name is not None:
        pose = ret[export_link_name]
    else:
        pose = None
    return pose, ret


def get_fk_from_model(model_path: str, joint_value, export_link=None, verbose=False):
    """
    从URDF或MuJoCo XML文件计算正向运动学
    :param model_path: URDF或MuJoCo XML文件路径
    :param joint_value: 关节角度值
    :param export_link: 末端执行器连杆名称
    :param verbose: 是否打印运动链信息
    :return: 末端执行器位姿和所有连杆的变换矩阵
    """
    from robolab.kinematics.pytorch_kinematics_utils import build_chain_from_model

    # 从模型文件构建运动链
    chain = build_chain_from_model(model_path, verbose)
    # 计算正向运动学
    pose, ret = get_fk_from_chain(chain, joint_value, export_link)
    return pose, ret
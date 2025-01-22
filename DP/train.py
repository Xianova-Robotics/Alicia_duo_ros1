# 这个脚本是训练模型的入口，值得注意的是要确认数据的有效性。

"""
Usage:
Training:
python train.py --config-name=train_diffusion_lowdim_workspace
"""

import sys
# use line-buffering for both stdout and stderr
sys.stdout = open(sys.stdout.fileno(), mode='w', buffering=1)
sys.stderr = open(sys.stderr.fileno(), mode='w', buffering=1)

import hydra
from omegaconf import OmegaConf
import pathlib
from diffusion_policy.workspace.base_workspace import BaseWorkspace

# allows arbitrary python code execution in configs using the ${eval:''} resolver
OmegaConf.register_new_resolver("eval", eval, replace=True)

@hydra.main(
    version_base=None,
    config_path=str(pathlib.Path(__file__).parent.joinpath(
        'diffusion_policy','config'))
)
def main(cfg: OmegaConf):
    # resolve immediately so all the ${now:} resolvers
    # will use the same time.
    OmegaConf.resolve(cfg)

    cls = hydra.utils.get_class(cfg._target_)
    workspace: BaseWorkspace = cls(cfg)
    workspace.run()

if __name__ == "__main__":
    main()

# import sys
# # use line-buffering for both stdout and stderr
# sys.stdout = open(sys.stdout.fileno(), mode='w', buffering=1)
# sys.stderr = open(sys.stderr.fileno(), mode='w', buffering=1)

# import hydra
# from omegaconf import OmegaConf
# import pathlib
# from diffusion_policy.workspace.base_workspace import BaseWorkspace

# # allows arbitrary python code execution in configs using the ${eval:''} resolver
# OmegaConf.register_new_resolver("eval", eval, replace=True)

# @hydra.main(
#     version_base=None,
#     config_path=str(pathlib.Path(__file__).parent.joinpath(
#         'diffusion_policy', 'config')),
#     config_name="train_diffusion_unet_real_image_workspace"  # 默认配置文件名
# )
# def main(cfg: OmegaConf):
#     # 手动设置默认值
#     # 设置 _target_ 类路径，如果没有配置
#     if "_target_" not in cfg:
#         cfg._target_ = "diffusion_policy.workspace.base_workspace.BaseWorkspace"

#     # 设置 task 字段，如果没有配置
#     if "task" not in cfg:
#         cfg.task = OmegaConf.create()

#     # 设置默认的 dataset_path（根据你提供的路径）
#     if "dataset_path" not in cfg.task:
#         cfg.task.dataset_path = "/home/xuanya/eto/diffusion_policy/data/real_pusht_17"

#     # 输出当前配置以便调试
#     # print("Current Configuration:")
#     # print(OmegaConf.to_yaml(cfg))

#     # 解析配置
#     OmegaConf.resolve(cfg)

#     # 获取目标类并运行
#     cls = hydra.utils.get_class(cfg._target_)
#     workspace: BaseWorkspace = cls(cfg)
#     workspace.run()

# if __name__ == "__main__":
#     main()
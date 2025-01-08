import argparse
import math
import queue
import threading
import time
from collections import deque
import numpy as np
import torch
import zmq
import dill
import hydra
from pytorch_util import dict_apply
from rotation_transformer import RotationTransformer

POLICY_CONTROL_PERIOD = 0.1  # 100 ms (10 Hz)
LATENCY_BUDGET = 0.2  # 200 ms including policy inference and communication
LATENCY_STEPS = math.ceil(LATENCY_BUDGET / POLICY_CONTROL_PERIOD)  # Up to 3 is okay, 4 is too high

class DiffusionPolicy:
    def __init__(self, ckpt_path):
        with open(ckpt_path, 'rb') as f:
            payload = torch.load(f, pickle_module=dill)
        cfg = payload['cfg']
        cls = hydra.utils.get_class(cfg._target_)
        workspace = cls(cfg)
        workspace.load_payload(payload)
        policy = workspace.model
        if cfg.training.use_ema:
            policy = workspace.ema_model
        self.policy = policy.to('cpu').eval()
        self.rotation_transformer = RotationTransformer(from_rep='rotation_6d', to_rep='quaternion')
        self.obs_shape_meta = cfg.shape_meta['obs']
        self.warmed_up = False

    def step(self, obs_sequence):
        obs_dict = self._convert_obs(obs_sequence)
        with torch.no_grad():
            if not self.warmed_up:
                self.policy.predict_action(obs_dict)
                self.warmed_up = True
            result = self.policy.predict_action(obs_dict)
            action = result['action'][0].detach().cpu().numpy()
        return self._convert_action(action)

    def _convert_obs(self, obs_sequence):
        obs_dict_np = {}
        for key, value in self.obs_shape_meta.items():
            if value.get('type') == 'rgb':
                images = np.stack([obs[key] for obs in obs_sequence], axis=0).astype(np.float32) / 255.0
                images = np.transpose(images, (0, 2, 1, 3))  # Ensure proper transposition
                print(f"Before transpose - images.shape: {images.shape}, expected shape: {tuple(value['shape'])}")
                assert images.shape[1:] == tuple(value['shape']), f"Shape mismatch: {images.shape[1:]} != {tuple(value['shape'])}"
                obs_dict_np[key] = images
            else:
                obs_dict_np[key] = np.stack([obs[key] for obs in obs_sequence], axis=0).astype(np.float32)
        return dict_apply(obs_dict_np, lambda x: torch.from_numpy(x).unsqueeze(0).to('cpu'))

    def _convert_action(self, action):
        act_sequence = []
        for act in action:
            action_dict = {
                'base_pose': act[:3],
                'arm_pos': act[3:6],
                'arm_quat': self.rotation_transformer.forward(act[6:12])[[1, 2, 3, 0]],
                'gripper_pos': act[12:13],
            }
            act_sequence.append(action_dict)
        return act_sequence

class PolicyWrapper:
    def __init__(self, policy, n_obs_steps=2, n_action_steps=8):
        self.policy = policy
        self.n_obs_steps = n_obs_steps
        self.n_action_steps = n_action_steps
        self.obs_queue = queue.Queue()
        self.act_queue = queue.Queue()
        self.lock = threading.Lock()
        threading.Thread(target=self.inference_loop, daemon=True).start()

    def reset(self):
        with self.lock:
            self.obs_queue.put('reset')

    def step(self, obs):
        with self.lock:
            self.obs_queue.put(obs)
            action = None if self.act_queue.empty() else self.act_queue.get()
            if action is None:
                print('Warning: Action queue is empty.')
            return action

    def inference_loop(self):
        obs_history = deque(maxlen=self.n_obs_steps)
        start_of_episode = True

        while True:
            with self.lock:
                if not self.obs_queue.empty():
                    obs = self.obs_queue.get()
                    if obs == 'reset':
                        self.policy.reset()
                        obs_history.clear()
                        start_of_episode = True
                        while not self.act_queue.empty():
                            self.act_queue.get()
                        continue
                    obs_history.append(obs)

            if len(obs_history) == self.n_obs_steps and self.act_queue.qsize() < LATENCY_STEPS:
                obs_sequence = list(obs_history)
                act_sequence = self.policy.step(obs_sequence)

                if start_of_episode:
                    act_sequence = act_sequence[:self.n_action_steps - LATENCY_STEPS]
                    start_of_episode = False
                else:
                    act_sequence = act_sequence[LATENCY_STEPS:self.n_action_steps]

                for action in act_sequence:
                    self.act_queue.put(action)
            time.sleep(POLICY_CONTROL_PERIOD)

class PolicyServer:
    def __init__(self, policy):
        self.policy = policy

    def step(self, obs):
        action = self.policy.step(obs)
        return action

    def run(self):
        while True:
            req = {
                'obs': {
                    'camera_0': np.random.randint(0, 255, (3, 240, 320), dtype=np.uint8),
                    'robot_eef_pose': np.array([0.5, -0.3], dtype=np.float32)
                },
                'action': np.array([1.0, -1.0], dtype=np.float32)
            }
            print(f"Generated image shape: {req['obs']['camera_0'].shape}")
            action = self.step(req['obs'])
            rep = {'action': action}
            # Optionally: send response back via socket or other communication channel
            # self.socket.send_pyobj(rep)

def main(ckpt_path):
    policy = PolicyWrapper(DiffusionPolicy(ckpt_path))
    server = PolicyServer(policy)
    server.run()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--ckpt-path', default='/home/eto/ZLK/diffusion_policy/latest.ckpt')
    args = parser.parse_args()
    main(args.ckpt_path)

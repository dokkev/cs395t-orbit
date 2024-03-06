# Copyright (c) 2022-2024, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates how to run the RL environment for the cartpole balancing task.
"""

from __future__ import annotations

"""Launch Isaac Sim Simulator first."""


import argparse

from omni.isaac.orbit.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on running the cartpole RL environment.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch
import traceback

import carb

from omni.isaac.orbit.envs import RLTaskEnv
from omni.isaac.orbit.envs import BaseEnv, BaseEnvCfg

# from omni.isaac.orbit_tasks.classic.cartpole.cartpole_env_cfg import CartpoleEnvCfg
from env_cfg import FrankaCubeLiftEnvCfg 


def main():
    """Main Function"""
    env_cfg = FrankaCubeLiftEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env = RLTaskEnv(cfg=env_cfg)
    
    # simulate the environment
    count = 0
    obs, _ = env.reset()
    while simulation_app.is_running():
        with torch.inference_mode():
            if count % 300 == 0:
                count = 0
                env.reset()
                print("-" * 80)
                print("[INFO]: Resetting environment...")
            # sample random actions
            # joint_efforts = torch.randn_like(env.action_manager.action)
            joint_efforts= torch.zeros(env_cfg.scene.num_envs, 8, device="cuda:0")

            # step the environment
            obs, rew, terminated, truncated, info = env.step(joint_efforts)
            # print current orientation of pole
            print("[Env 0] joint: ", obs["policy"][0][1].item())
            # update counter
                
            
            count = count + 1
        
    



if __name__ == "__main__":
    try:
        # run the main execution
        main()
    except Exception as err:
        carb.log_error(err)
        carb.log_error(traceback.format_exc())
        raise
    finally:
        # close sim app
        simulation_app.close()
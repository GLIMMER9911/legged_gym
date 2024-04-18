from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO

class BlotRoughCfg( LeggedRobotCfg ):
    class env( LeggedRobotCfg.env):
        num_actions = 6

    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.35487417] # x,y,z [m]
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_HAA': -0.3,   # [rad]
            'FR_HAA': 0.3 ,  # [rad]

            'FL_HFE': 0.78,     # [rad]
            'FR_HFE': 0.78,     # [rad]

            'FL_KFE': -1.5,    # [rad]
            'FR_KFE': -1.5,    # [rad]
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'HAA': 20, 'HFE': 20, 'KFE': 10}  # [N*m/rad]
        damping = {'HAA': 0.5, 'HFE': 0.5, 'KFE': 0.5}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/bolt/urdf/bolt.urdf'
        name = "bolt"
        foot_name = "foot"
        penalize_contacts_on = ["LOWER_LEG", "UPPER_LEG"]
        terminate_after_contacts_on = ["base"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
  
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.25
        class scales( LeggedRobotCfg.rewards.scales ):
            torques = -0.0002
            dof_pos_limits = -10.0

class BoltRoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        experiment_name = 'rough_bolt'

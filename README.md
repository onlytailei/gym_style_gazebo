# A gym-style gazebo interface for reinforcement learning
Please finish the installation of [gym_ped_sim](https://github.com/onlytailei/gym_ped_sim) at first.

### Example
```
roslaunch gym_style_gazebo default.launch
```

Based on the turtlebot 3 agent, interface with the gazebo pedestrain-rich environments through the service 
```
gazebo_env_io/pytorch_io_service
```

------

This is the reference implementation of the plugins and for the paper **Socially-compliant Navigation through Raw Depth Inputs with Generative Adversarial Imitation Learning**. 
If it helps your research, please cite:
```
@article{tai2017socially,
  title={Socially-compliant Navigation through Raw Depth Inputs with Generative Adversarial Imitation Learning},
  author={Tai, Lei and Zhang, Jingwei and Liu, Ming and Burgard, Wolfram},
  journal={arXiv preprint arXiv:1710.02543},
  year={2017}
}
```

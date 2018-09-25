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
@inproceedings{tai2018social,
    author={L. Tai and J. Zhang and M. Liu and W. Burgard},
    booktitle={2018 IEEE International Conference on Robotics and Automation (ICRA)}, 
    title={Socially Compliant Navigation Through Raw Depth Inputs with Generative Adversarial Imitation Learning}, 
    year={2018}, 
    pages={1111-1117}, 
    doi={10.1109/ICRA.2018.8460968}, 
    ISSN={2577-087X}, 
    month={May},
}
```

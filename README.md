# ROS2
 To build

    mkdir -p ws/src
    cd ws/src
    git clone -b ros2 https://github.com/MVerschure/rpg_ig_active_reconstruction.git

It depends on the Eigen library trhough the `EIGEN3_INCLUDE_DIR`. I "installed" it by downloading the library from [here](https://eigen.tuxfamily.org/index.php?title=Main_Page) and running

    export EIGEN3_INCLUDE_DIR='path/to/eigen-3.4.0'

Then to build run in root directory of the workspace

    colcon build --symlink-install --packages-skip flying_gazebo_stereo_cam


Information Gain Based Active Reconstruction
============================================

This repository contains the code for **information gain based active reconstruction**, presented in the papers:

*An Information Gain Formulation for Active Volumetric 3D Reconstruction*  
S. Isler, R. Sabzevari, J. Delmerico and D. Scaramuzza (ICRA 2016)  
[Paper: http://rpg.ifi.uzh.ch/docs/ICRA16_Isler.pdf](http://rpg.ifi.uzh.ch/docs/ICRA16_Isler.pdf)  
[Video (Youtube)](https://www.youtube.com/watch?v=ZcJcsoGGqbA&feature=youtu.be) 

and 

*A comparison of volumetric information gain metrics for active 3D object reconstruction*  
J. Delmerico, S. Isler, R. Sabzevari, and D. Scaramuzza (Autonomous Robots 2017)  
[Paper: http://rpg.ifi.uzh.ch/docs/AURO17_Delmerico.pdf](http://rpg.ifi.uzh.ch/docs/AURO17_Delmerico.pdf)

If you use this software in a scholarly work, please cite our journal paper:

```
@article{delmerico2018comparison,
  title={A comparison of volumetric information gain metrics for active 3D object reconstruction},
  author={Delmerico, Jeffrey and Isler, Stefan and Sabzevari, Reza and Scaramuzza, Davide},
  journal={Autonomous Robots},
  volume={42},
  number={2},
  pages={197--208},
  year={2018},
  publisher={Springer}
}
```

#### Disclaimer

The ig_active_reconstruction implementation in this repository is research code, any fitness for a particular purpose is disclaimed.

The code has been tested in Ubuntu 14.04 with ROS Indigo.

#### Licence

The source code is released under a GPLv3 licence.

http://www.gnu.org/licenses/

#### Citing

If you use ig_active_reconstruction in an academic context, please cite the following publication:

    @inproceedings{Isler2016ICRA,
      title={An Information Gain Formulation for Active Volumetric 3D Reconstruction},
      author={Isler, Stefan and Sabzevari, Reza and Delmerico, Jeffrey and Scaramuzza, Davide},
      booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
      year={2016},
      organization={IEEE}
    }

#### Install and run ig_active_reconstruction

The wiki 

https://github.com/uzh-rpg/rpg_ig_active_reconstruction/wiki

contains instructions on how to build and run the code.

#### Acknowledgments

Thanks to Pavel Vechersky for his key contributions, as well as to Elias Mueggler, Matthias Faessler, and Junjie Zhang for their valuable feedback.
   
#### Contributing

You are very welcome to contribute to ig_active_reconstruction by opening a pull request via Github.
We try to follow the ROS C++ style guide http://wiki.ros.org/CppStyleGuide

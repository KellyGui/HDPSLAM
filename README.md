# HDPSLAM
our simulation implementation of HDP SLAM based on the original code of the paper < SLAM with objects using a nonparametric pose graph >
the github is https://github.com/BeipengMu/objectSLAM

Two demos:
      version1: the parameter of standard deviation is set
      version2: use distribution with the hpyerparameter gamma to sample the parameter of standard deviation
      
The common-view frame matrix is computed for the measurement among the frames with the same commonview relationships.

see the bib reference of our HDP SLAM paper below:
@ARTICLE{8794595,
author={J. {Zhang} and M. {Gui} and Q. {Wang} and R. {Liu} and J. {Xu} and S. {Chen}},
journal={IEEE Transactions on Visualization and Computer Graphics},
title={Hierarchical Topic Model Based Object Association for Semantic SLAM},
year={2019},
volume={25},
number={11},
pages={3052-3062},
keywords={Simultaneous localization and mapping;Semantics;Optimization;Cameras;Image reconstruction;Atmospheric modeling;Computer science;Visual Semantic SLAM;Object Association;Hierarchical Dirichlet Process},
doi={10.1109/TVCG.2019.2932216},
ISSN={},
month={Nov},}

The code for "SLAM with objects using a nonparametric pose graph". See bib reference below:
@INPROCEEDINGS{Mu_iros_2016,
	author={B. Mu and S. Y. Liu and L. Paull and J. Leonard and J. P. How},
	booktitle={2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
	title={SLAM with objects using a nonparametric pose graph},
	year={2016},
	pages={4602-4609},
	keywords={SLAM (robots);graph theory;object detection;pose estimation;robot vision;sensor fusion;SLAM;data association;nonparametric pose graph;object detection;robotic application;simultaneous localization and mapping;Machine learning;Object detection;Proposals;Robustness;Simultaneous localization and mapping;Three-dimensional displays},
	doi={10.1109/IROS.2016.7759677},
	month={Oct},}

## iSAM library
Folder isam contains the modified isam library to optimize pose graphs. There are pre-compiled executable file isam is under the bin folder
To compile from source, following the commands on ubuntu:
cd isam
mkdir build && cd build && cmake ..
make
Fore more details about the library, refer to readme file under isam folder.

To generate simulated dataset, run generateSimData.m. Ground truth objects are randomly generated. Click in the figure to generate the ground truth trajectory, make sure there are enough loop closures. When finished, press enter button on the keyboard.

## HDP SLAM simulation demo
To run the algorithm and compared algorithms, run Demo1.m and Demo2.m

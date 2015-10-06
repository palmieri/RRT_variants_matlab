RRTblossom_matlab
=================

A RRT-Blossom Matlab implementation.

You need to download librobotics from http://srl.informatik.uni-freiburg.de/downloads

The script aims to show the funcionalities of RRT blossom [1], for a nonholonomic differential drive wheeled mobile robot. Motion Primitives are generated using the ODE command provided by MATLAB

-- rrtblossom.m is the main script, where to launch the motion planner

-- extendblossom.m, method to extend the tree

-- checkedge.m, to check a collision on a single edge

-- checkcollision.m, to check a collision on a single configuration

-- nearesttree.m, to find the nearest vertex on the tree

-- randomstate.m, to sample a random configuration in the defined space

-- unicycleKinematics.m, it implements the kinematic model of a unicycle

-- getControls.m, to extract controls from the tree

-- extractpath.m, to extract the obtained path from the tree

-- generateObstacles.m, to generate a scenario



References

[1] RRT-blossom: RRT with a local ﬂood-ﬁll behavior, Kalisiak M., van de Panne M., ICRA06 http://www.dgp.toronto.edu/~mac/pubs/rrt-blossom.pdf

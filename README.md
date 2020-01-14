This project is a library for hundreds of units to find path running on mobile phones.
##Introduction
The path finding library is based on grid map,it supports static,dynamic obstacle avoidance behaviour,boids behaviour.
Besides,it supports data determination for frame synchronization.The library uses A star,
flow field algorithm and dynamic AABB tree for improving performance.The units find path looks
like StarCraft2 path finding mode with a local clear direction finding algorithm,and without dynamic 
force it looks like warcraft3 path finding mode.
##how to use 
Refer to TestPathFinding.cs,you will find how to create PathFindingAgentBehaviour with setting up required data.
For test scene,you can set countLimit as path finding unit number.You can enable or disable some features 
by modify Player Settings Scripting Define Symbols,such as DYNAMIC_FORCE for enable dynamic force which means
it uses local clear direction finding algorithm,otherwise it just use A star algorithm for path finding,set USING_FLOW_FIELD
will enable flow field path finding,MULTI_THREAD will use multi thread ,remove MULTI_THREAD usually running for debug. 

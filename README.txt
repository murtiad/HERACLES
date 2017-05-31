===========================================================================================================
ooooo   ooooo oooooooooooo ooooooooo.         .o.         .oooooo.   ooooo        oooooooooooo  .oooooo..o 
`888'   `888' `888'     `8 `888   `Y88.      .888.       d8P'  `Y8b  `888'        `888'     `8 d8P'    `Y8 
 888     888   888          888   .d88'     .8"888.     888           888          888         Y88bo.      
 888ooooo888   888oooo8     888ooo88P'     .8' `888.    888           888          888oooo8     `"Y8888o.  
 888     888   888    "     888`88b.      .88ooo8888.   888           888          888    "         `"Y88b 
 888     888   888       o  888  `88b.   .8'     `888.  `88b    ooo   888       o  888       o oo     .d8P 
o888o   o888o o888ooooood8 o888o  o888o o88o     o8888o  `Y8bood8P'  o888ooooood8 o888ooooood8 8""88888P' 
===========================================================================================================
%%%%%%%%%%%%%%%%                      HERitAge by point CLoud procESsing                   %%%%%%%%%%%%%%%%
===========================================================================================================
%%%%%%%%%%%%%%%%                          (c) by Arnadi Murtiyoso                          %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%     Photogrammetry and Geomatics Group, ICube UMR 7357 INSA Strasbourg    %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%                  Contact: arnadi.murtiyoso@insa-strasbourg.fr             %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%                         https://github.com/murtiad                        %%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%                       Last update: 31 May 2017 (v0.1)                     %%%%%%%%%%%%%%%%
===========================================================================================================

Objective: performs various processing algorithms to point clouds in the interest of generating HBIMs (Heritage Building Information Models).

Required dependencies:
- PCL (http://pointclouds.org/)
- libLAS (https://www.liblas.org/)

Change log:
v0.1:
30 May 2017:
- integrated las2pcd into HERACLES
- integrated PARVIS into HERACLES
- create ROSACE (Ransac Object SegmentAtion Clearly simplE) as a function

31 May 2017:
- integrated ROSACE into HERACLES: enables simple RANSAC-based plane segmentation on .las and .pcd point clouds
- modified las2pcd in HERACLES to not generate a .pcd file, but rather keeps it in the cloud variable
- modified PARVIS to enable direct reading of .las files

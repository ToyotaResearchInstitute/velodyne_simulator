^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package velodyne_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.4 (2017-10-17)
------------------
* Use robotNamespace as prefix for PointCloud2 topic frame_id by default
* Use Gazebo LaserScan message instead of direct LaserShape access, fixes timestamp issue
* Contributors: Kevin Hallenbeck, Max Schwarz, Micho Radovnikovich

0.0.3 (2017-09-05)
------------------
* Fixed ground plane projection by removing interpolation
* Updated package.xml format to version 2
* Removed gazebo_plugins dependency
* Gazebo7 integration
* Contributors: Kevin Hallenbeck, Micho Radovnikovich, Konstantin Sorokin

0.0.2 (2016-02-04)
------------------
* Display laser count when loading gazebo plugin
* Don't reverse ring for newer gazebo versions
* Changed to PointCloud2. Handle min and max range. Noise. General cleanup.
* Start from block laser
* Contributors: Kevin Hallenbeck

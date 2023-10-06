# Pointcloud transformer

## Installation

``
catkin install pointcloud_transformer
``

## Usage
This package transforms `sensor_msgs/Pointcloud` or `sensor_msgs/Pointcloud2` to some other 
TF frame and republishes it.

### For input topic of type `sensor_msgs/Pointcloud`
``
roslaunch pointcloud_transformer pointcloud_transformer1.launch
``
* `~input_pcl`: `input_topic` (`sensor_msgs/Pointcloud`)
* `~output_pcl`: `output_topic` (`sensor_msgs/Pointcloud`) 

### For input topic of type `sensor_msgs/Pointcloud2`
``
roslaunch pointcloud_transformer pointcloud_transformer2.launch
``
* `~input_pcl2`: `input_topic` (`sensor_msgs/Pointcloud2`)
* `~output_pcl2`: `output_topic` (`sensor_msgs/Pointcloud2`) 

### Parameters
* `to_frame`: to what TF frame the input pointcloud is transformed to, default: (`base_link`)


## Credits
Robin Schmid, schmidrobin@outlook.com

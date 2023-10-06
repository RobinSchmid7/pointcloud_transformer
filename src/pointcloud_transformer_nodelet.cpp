#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


namespace pointcloud_transformer {
	class pointcloudTransformer : public nodelet::Nodelet {
		ros::Publisher pc_pub;
		ros::Publisher pc2_pub;
		ros::Subscriber pc_sub;
		ros::Subscriber pc2_sub;

		std::string REFERENCE_FRAME = "base_link";

	public:
		pointcloudTransformer() = default;

	private:
		virtual void onInit() {
			ros::NodeHandle nh = getNodeHandle();
			ros::NodeHandle nhp = getPrivateNodeHandle();
			if (!getPrivateNodeHandle().getParam("to_frame", REFERENCE_FRAME)) {
				ROS_ERROR("You must specify 'to_frame' parameter !");
				exit(-123);
			}
			pc_pub = nhp.advertise<sensor_msgs::PointCloud>("output_pcl", 1);
			pc2_pub = nhp.advertise<sensor_msgs::PointCloud2>("output_pcl2", 1);
			pc_sub = nhp.subscribe<sensor_msgs::PointCloud>("input_pcl", 1, &pointcloudTransformer::pointcloudCB,	this);
			pc2_sub = nhp.subscribe<sensor_msgs::PointCloud2>("input_pcl2", 1, &pointcloudTransformer::pointcloud2CB, this);
		};

		bool getTransform(const std::string &refFrame, const std::string &childFrame, tf::StampedTransform &transform) {
			std::string errMsg;
			tf::TransformListener tf_listener;
			if (!tf_listener.waitForTransform(refFrame, childFrame, ros::Time(0), ros::Duration(0.5),
											  ros::Duration(0.01), &errMsg)) {
				ROS_ERROR_STREAM("Pointcloud transform | Unable to get pose from TF: " << errMsg);
				return false;
			} else {
				try {
					tf_listener.lookupTransform(refFrame, childFrame, ros::Time(0), transform);
				}
				catch (const tf::TransformException &e) {
					ROS_ERROR_STREAM(
							"Pointcloud transform | Error in lookupTransform of " << childFrame << " in " << refFrame);
					return false;
				}
			}
			return true;
		}

		void pointcloudCB(const sensor_msgs::PointCloud::ConstPtr &cloud_in) {
			if (cloud_in->points.empty()) { return; }

			tf::StampedTransform tf_between_frames;
			getTransform(REFERENCE_FRAME, cloud_in->header.frame_id, tf_between_frames); // get transformation
			geometry_msgs::TransformStamped tf_between_frames_geo;
			tf::transformStampedTFToMsg(tf_between_frames, tf_between_frames_geo); // convert it to geometry_msg type

			sensor_msgs::PointCloud2 cloud_in2;
			sensor_msgs::convertPointCloudToPointCloud2(*cloud_in, cloud_in2); // convert from pointcloud to pointcloud2
			sensor_msgs::PointCloud2 cloud_in_transformed;
			tf2::doTransform(cloud_in2, cloud_in_transformed, tf_between_frames_geo); // do transformation
			if (pc2_pub.getNumSubscribers() > 0) { // publish pointcloud2
				pc2_pub.publish(cloud_in_transformed);
			}
			if (pc_pub.getNumSubscribers() > 0) {
				sensor_msgs::PointCloud cloud_out;
				sensor_msgs::convertPointCloud2ToPointCloud(cloud_in_transformed, cloud_out); //convert from pointcloud to pointcloud2
				pc_pub.publish(cloud_out);  // publish pointcloud
			}
		}

		void pointcloud2CB(const sensor_msgs::PointCloud2::ConstPtr &cloud_in) {
			if (cloud_in->data.empty()) { return; }

			tf::StampedTransform tf_between_frames;
			getTransform(REFERENCE_FRAME, cloud_in->header.frame_id, tf_between_frames);
			geometry_msgs::TransformStamped tf_between_frames_geo;
			tf::transformStampedTFToMsg(tf_between_frames, tf_between_frames_geo);

			sensor_msgs::PointCloud2 cloud_in_transformed;
			tf2::doTransform(*cloud_in, cloud_in_transformed, tf_between_frames_geo); // do transformation
//            std::cout << "cloud_in: " << tf2::getTimestamp(*cloud_in) << std::endl;
            cloud_in_transformed.header.stamp = tf2::getTimestamp(*cloud_in);
//			std::cout << "cloud_in_transformed: " << tf2::getTimestamp(cloud_in_transformed) << std::endl;
			if (pc2_pub.getNumSubscribers() > 0) {
				pc2_pub.publish(cloud_in_transformed);
			}
			if (pc_pub.getNumSubscribers() > 0) {
				sensor_msgs::PointCloud cloud_out;
				sensor_msgs::convertPointCloud2ToPointCloud(cloud_in_transformed, cloud_out); //convert from pointcloud to pointcloud2
				pc_pub.publish(cloud_out);
			}
		}


	};
}

PLUGINLIB_EXPORT_CLASS(pointcloud_transformer::pointcloudTransformer, nodelet::Nodelet);


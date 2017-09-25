#!/usr/bin/env python
# Completed by Yang Dai

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # TODO: Statistical Outlier Filtering
    outlier_filter = cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(20)

    # set threshold
    x = 0.5
    outlier_filter.set_std_dev_mul_thresh(x)

    # call Filter
    cloud_filtered = outlier_filter.filter()

    # TODO: Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter in Z-axis
    passthrough_z = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough_z.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 0.8
    passthrough_z.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough_z.filter()
    #filename = 'pass_through_filtered.pcd'
    #pcl.save(cloud_filtered, filename)

    # PassThrough Filter in Y-axis
    passthrough_y = cloud_filtered.make_passthrough_filter()
    filter_axis = 'y'
    passthrough_y.set_filter_field_name(filter_axis)
    axis_min = -0.44
    axis_max = 0.44
    passthrough_y.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough_y.filter()
    filename = 'pass_through_filtered.pcd'
    pcl.save(cloud_filtered, filename)

    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.04
    seg.set_distance_threshold(max_distance)

    # TODO: Extract inliers and outliers
    inliers, coefficients = seg.segment()
    # table
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    # not tables
    cloud_objects = cloud_filtered.extract(inliers, negative=True)
    filename = 'extracted_outliers.pcd'
    pcl.save(cloud_objects, filename)

    # TODO: Euclidean Clustering
    # create white cloud
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    # create cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # set tolerances, min/max cluster size
    ec.set_ClusterTolerance(0.015)
    ec.set_MinClusterSize(20)
    ec.set_MaxClusterSize(1000)
    # search k-d tree for clusters
    ec.set_SearchMethod(tree)
    # extract indices for each discovered cluster
    cluster_indices = ec.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, index in enumerate(indices):
            color_cluster_point_list.append([white_cloud[index][0],
                white_cloud[index][1], white_cloud[index][2],
                rgb_to_float(cluster_color[j])])
    # create cloud containing all clusters with color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # save to file
    filename = 'cluster_colored.pcd'
    pcl.save(cluster_cloud, filename)

    # TODO: Convert PCL data to ROS messages
    ros_objects = pcl_to_ros(cloud_objects)
    ros_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    labeled_features = []
    detected_objects_labels = []
    detected_objects = []
    object_centroids = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        labeled_features.append([feature, index])

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += 0.4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Calculate centroids
        points_array = []
        for pt in pts_list:
            points_array.append(white_cloud[pt])
        object_centroids.append(np.mean(points_array, axis=0))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    # Publish the list of detected objects
    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects_labels, object_centroids)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(detected_objects_labels, object_centroids):

    # TODO: Initialize variables
    test_scene_num = Int32()
    object_name = String()
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox = rospy.get_param('/dropbox')

    # TODO: Parse parameters into individual variables
    object_name_list = []
    object_group_list = []
    for item in object_list_param:
        object_name_list.append(item['name'])
        object_group_list.append(item['group'])

    for box in dropbox:
        if box['group'] == 'red':
            red_arm = box['name']
            red_pos = box['position']
        elif box['group'] == 'green':
            green_arm = box['name']
            green_pos = box['position']

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # CHANGE FOR DIFF WORLD
    test_scene_num.data = 2

    # TODO: Loop through the pick list
    dict_list = []

    for i, name in enumerate(object_name_list):
        object_name.data = name

        # TODO: Create 'place_pose' for the object
        # set to origin if item not found
        pick_pose_xyz = (0, 0, 0)
        for j, known_label in enumerate(detected_objects_labels):
            if name == known_label:
                # TODO: Get the PointCloud for a given object and obtain it's centroid
                pick_pose_xyz = object_centroids[j]
        pick_pose.position.x = np.asscalar(pick_pose_xyz[0])
        pick_pose.position.y = np.asscalar(pick_pose_xyz[1])
        pick_pose.position.z = np.asscalar(pick_pose_xyz[2])

        # TODO: Assign the arm to be used for pick_place
        if object_group_list[i] == 'green':
            arm_name.data = green_arm
            place_pose_xyz = green_pos
        elif object_group_list[i] == 'red':
            arm_name.data = red_arm
            place_pose_xyz = red_pos
        place_pose.position.x = place_pose_xyz[0]
        place_pose.position.y = place_pose_xyz[1]
        place_pose.position.z = place_pose_xyz[2]

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        dict_list.append(make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose))

        '''
        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            #resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)
            resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        '''

    # TODO: Output your request parameters into output yaml file
    # CHANGE FOR DIFF WORLD
    yaml_filename = 'output_2.yaml'
    send_to_yaml(yaml_filename, dict_list)


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering',anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber('/pr2/world/points', pc2.PointCloud2, pcl_callback,
                                queue_size=1)

    # TODO: Create Publishers
    detected_objects_pub = rospy.Publisher('/detected_objects', DetectedObjectsArray,
                                            queue_size=1)
    object_markers_pub = rospy.Publisher('/object_markers', Marker, queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open('model.sav','rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  	//#######################################################################
  	//#### Read data and perform segmentation and ground plane removal 
  	//#######################################################################

	// Convert ROS message to pcl format 
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::fromROSMsg (*input, *cloud);

	//*****************
	// GROUND PLANE FILTERING (DISABLED BECAUSE THIS IS NOT INCLUDED ATM)
	//*****************
  	// // Create the segmentation object for the planar model and set all the parameters
 	// pcl::SACSegmentation<pcl::PointXYZ> seg;
  	// pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  	// pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  	// seg.setOptimizeCoefficients (true);
  	// seg.setModelType (pcl::SACMODEL_PLANE);
  	// seg.setMethodType (pcl::SAC_RANSAC);
  	// seg.setMaxIterations (100);			
  	// seg.setDistanceThreshold (0.3);

  	// int i=0, nr_points = (int) cloud->points.size ();
  	// while (cloud->points.size () > 0.3 * nr_points)
  	// {
    // 	// Segment the largest planar component from the remaining cloud
    // 	seg.setInputCloud (cloud);
    // 	seg.segment (*inliers, *coefficients);
    // 	if (inliers->indices.size () == 0)
    // 	{
    //   		std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    // 	}

    // 	// Extract the planar inliers from the input cloud
    // 	pcl::ExtractIndices<pcl::PointXYZ> extract;
    // 	extract.setInputCloud (cloud);
    // 	extract.setIndices (inliers);
    // 	extract.setNegative (false);

    // 	// Get the points associated with the planar surface
    // 	extract.filter (*cloud_plane);
    // 	std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // 	// Remove the planar inliers, extract the rest, the ground is filterd out of the point cloud
    // 	extract.setNegative (true);
    // 	extract.filter (*cloud_f);
    // 	*cloud = *cloud_f;
	// }

  	//#######################################################################
  	//#### Do Euclidean cluster extraction
  	//#######################################################################

  	// Creating the KdTree object for the search method of the extraction
  	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  	tree->setInputCloud (cloud);

  	std::vector<pcl::PointIndices> cluster_indices;
  	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  	ec.setClusterTolerance (1); // 50cm
  	ec.setMinClusterSize (4);
  	ec.setMaxClusterSize (25000);
  	ec.setSearchMethod (tree);
  	ec.setInputCloud (cloud);
  	ec.extract (cluster_indices);

	// Prepare  2D message
	vision_msgs::Detection2DArray msgOut;
  	vision_msgs::Detection2D det;
  	
  	msgOut.header = input->header;
  	Eigen::Vector4f centroid;

	// Dividing the clusters
  	int j = 0;
  	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  	{
    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      	cloud_cluster->points.push_back (cloud->points[*pit]); //*
    	cloud_cluster->width = cloud_cluster->points.size ();
    	cloud_cluster->height = 1;
    	cloud_cluster->is_dense = true;

	// Extract centroid and min/max
    	pcl::compute3DCentroid (*cloud_cluster, centroid);
    	pcl::PointXYZ min_p, max_p;
    	pcl::getMinMax3D(*cloud_cluster, min_p, max_p);

	// Prepare info for 2D message
	det.bbox.center.x = centroid[0];
	det.bbox.center.y = centroid[1];
	findBestOBB(*cloud_cluster, det);

	det.header = input->header;
	msgOut.detections.push_back(det);
	
	std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
	std::stringstream ss;
	ss << "cloud_cluster_" << j << ".pcd";
	j++;
  }
	pub.publish(msgOut); 	// publish messages
}

void findBestOBB(const pcl::PointCloud<pcl::PointXYZ> &cloud_cluster, vision_msgs::Detection2D &det){
	// This function tries to approximate the optimal Oriented Bounding Box for the given cloud cluster.
	// It constructs a bounding box and rotates it between 0-90 degrees, while logging its area.
	// The bounding box with smallest volume is selected as the best OBB and it is put in the detection message.
	std::vector<OBB> obbVec;
	// Defien bounding box center position
	double c_x = det.bbox.center.x;
	double c_y = det.bbox.center.y;
	// Try multiple Orientations for the Bounding Box
	for(double theta = 0; theta<(pi/2); theta +=(pi/16)){
		double dxMax{0}, dyMax{0};
		// Project all cloud points onto the rotated axes
		for(int i = 0; i!=cloud_cluster.points.size(); i++)
		{
			pcl::PointXYZ pclPoint = cloud_cluster.points[i];
			vector2D A = {pclPoint.x-c_x,pclPoint.y-c_y};		// Vector A from c.m. to point
			vector2D vecXaxis = {cos(theta),-sin(theta)}; 		// normal vector rotated x-axis
			vector2D vecYaxis = {sin(theta), cos(theta)};		// normal vector rotated y-axis
			double distX = myDot(A,vecXaxis);					// project A on x-axis
			double distY = myDot(A,vecYaxis); 					// project A on y-axis
			if(distX>dxMax){
				dxMax = distX;	// If distance is larger, update size
			}
			if(distY>dyMax){
				dyMax = distY;	// If distance is larger, update size								
			}
		}
		double area = dxMax*dyMax;
		//cout<<"theta="<<theta<<" sz_x="<<dxMax<<" sx_y="<<dyMax<<" area="<<area<<endl;
		// Log the data for best box selection
		OBB box = {theta, dxMax, dyMax, area};
		obbVec.push_back(box);
	}
	// Loop through all boxes and select the smallest one
	int best; double Amin = inf;
	for(int i = 0; i!=obbVec.size(); i++){
		if(obbVec[i].area<Amin){
			Amin = obbVec[i].area;
			best = i;
		}
	}
	// Insert best box into detection message
	det.bbox.center.theta = obbVec[best].theta;
	det.bbox.size_x = obbVec[best].size_x;
	det.bbox.size_y = obbVec[best].size_y;
	return;
};

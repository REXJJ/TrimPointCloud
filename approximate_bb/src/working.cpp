#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Trigger.h>
#include "point_cloud_utilities/pcl_utilities.hpp"
#define PI 3.141592653589793

using namespace std;

ros::Publisher Publish_Cloud_Trimmed;

ros::Publisher Clicked_Points;

vector<vector<double>> bounding_polygon;

string output_frame;

pcl::PointCloud<pcl::PointXYZRGB> cloud_trimmed_global;

string filename_global;

Eigen::MatrixXd transformation_global;

double dx=0,dy=0,dz=0;

bool lines_intersect(double l1[2][2], double l2[2][2])
{
    // l1 for horizontal ray line...slope is always zero

    // checking if other slope is zero
    if (l2[0][1]==l2[1][1])
    {
        return false;
    }
    else
    {
        // checking both pts of second line above first line
        if ((l2[0][1]>l1[0][1] && l2[1][1]>l1[0][1]) || (l2[0][1]<l1[0][1] && l2[1][1]<l1[0][1]))
        {
            return false;
        }
        else
        {
            // checking both pts of second line either on right or on left of fist line
            if ((l2[0][0]<l1[0][0] && l2[1][0]<l1[0][0]) || (l2[0][0]>l1[1][0] && l2[1][0]>l1[1][0]))
            {
                return false;
            }
            else
            {
                // checking if other line is vertical
                if (l2[0][0]== l2[1][0])
                {
                    return true;
                }
                else
                {
                    // getting intersection point
                    double m2 = (l2[1][1]-l2[0][1])/(l2[1][0]-l2[0][0]);        
                    double x = (l1[0][1]+m2*l2[0][0]-l2[0][1])/m2;
                    // checking if intersection point lies on the first line
                    if ((x>l1[0][0] || std::abs(x-l1[0][0])<1e-9) && (x<l1[1][0] || std::abs(x-l1[1][0])<1e-9))
                    {
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
            }
        }
    } 
    return false;
}

Eigen::MatrixXd InPoly(Eigen::MatrixXd& q, Eigen::MatrixXd& p)
{
    // p : polygon points
    // q : query points

    double l1[2][2];
    double l2[2][2];

    Eigen::MatrixXd in = Eigen::VectorXd::Constant(q.rows(),1,0);

    double xmin = p.col(0).minCoeff();
    double xmax = p.col(0).maxCoeff();
    double ymin = p.col(1).minCoeff();
    double ymax = p.col(1).maxCoeff();

    for (long i=0;i<q.rows();++i)
    {
        // bounding box test
        if (q(i,0)<xmin || q(i,0)>xmax || q(i,1)<ymin || q(i,1)>ymax)
        {
            continue;
        }
        int intersection_count = 0;
        Eigen::MatrixXd cont_lines = Eigen::MatrixXd::Constant(p.rows(),1,0);
        for (int j=0;j<p.rows();++j)
        {
            if (j==0)
            {
                l1[0][0] = q(i,0);l1[0][1] = q(i,1);
                l1[1][0] = xmax;l1[1][1] = q(i,1);
                l2[0][0] = p(p.rows()-1,0);l2[0][1] = p(p.rows()-1,1);
                l2[1][0] = p(j,0);l2[1][1] = p(j,1);
                if (lines_intersect(l1,l2))
                {
                    intersection_count++;
                    cont_lines(j,0) = 1;
                }   
            }
            else
            {
                l1[0][0] = q(i,0);l1[0][1] = q(i,1);
                l1[1][0] = xmax;l1[1][1] = q(i,1);
                l2[0][0] = p(j,0);l2[0][1] = p(j,1);
                l2[1][0] = p(j-1,0);l2[1][1] = p(j-1,1);
                if (lines_intersect(l1,l2))
                {
                    intersection_count++;
                    cont_lines(j,0) = 1;
                    if (cont_lines(j-1,0)==1)
                    {
                        if (p(j-1,1)==q(i,1))
                        {
                            if (j-1==0)
                            {
                                if (!((p(p.rows()-1,1)<p(j-1,1) && p(j,1)<p(j-1,1)) || (p(p.rows()-1,1)>p(j-1,1) && p(j,1)>p(j-1,1))))
                                {
                                    intersection_count--;
                                }
                            }
                            else
                            {
                                if (!((p(j-2,1)<p(j-1,1) && p(j,1)<p(j-1,1)) || (p(j-2,1)>p(j-1,1) && p(j,1)>p(j-1,1))))
                                {
                                    intersection_count--;
                                }
                            }
                        }
                    }
                }
            }
        }
        if (intersection_count%2==1)
        {
            in(i,0) = 1;
        }
    }
    return in;
}

inline std::string validate_seq(std::string seq)
{
	if(seq =="")
		seq = "ZYX";	
	bool invalid_flag = false;
	if(seq.size()!=3)
	{
		invalid_flag = true;
	}
	for (int i =0;i<3;++i)
		if(seq[i]!='X' && seq[i]!='Y' && seq[i]!='Z' && seq[i]!='x' && seq[i]!='y' && seq[i]!='z')
		{
			invalid_flag = true; 
			break;
		}
	if(invalid_flag)
	{
		std::cerr << "ERROR: Invalid Rotations Sequence: " << seq << std::endl;
		std::terminate();		
	}
	return seq;
}

inline Eigen::Matrix3d eul2rot(Eigen::MatrixXd eul_angles, std::string seq)
{
	seq = validate_seq(seq);
	Eigen::Matrix3d rot_mat = Eigen::Matrix3d::Identity();
	for (int i=0; i<3; ++i)
	{
		if(seq[i]=='X' || seq[i]=='x')
			rot_mat = rot_mat * Eigen::AngleAxisd(eul_angles(0,i), Eigen::Vector3d::UnitX());
		else if(seq[i]=='Y' || seq[i]=='y')
			rot_mat = rot_mat * Eigen::AngleAxisd(eul_angles(0,i), Eigen::Vector3d::UnitY());			
		else if(seq[i]=='Z' || seq[i]=='z')
			rot_mat = rot_mat * Eigen::AngleAxisd(eul_angles(0,i), Eigen::Vector3d::UnitZ());					
	}
	return rot_mat; 
}

inline Eigen::MatrixXd apply_transformation(Eigen::MatrixXd data, Eigen::Matrix4d T_mat)
{
	//! putting data in [x, y, z, 1]' format
	Eigen::MatrixXd data_with_fourth_row(data.cols()+1,data.rows());
	Eigen::VectorXd ones_vec = Eigen::VectorXd::Constant(data.rows(),1);
	data_with_fourth_row.block(0,0,data.cols(),data.rows()) = data.transpose();
	data_with_fourth_row.block(data.cols(),0,1,data.rows()) = ones_vec.transpose();
	Eigen::MatrixXd transformed_data = T_mat*data_with_fourth_row;
	Eigen::MatrixXd transformed_data_mat(transformed_data.rows()-1,transformed_data.cols());
	transformed_data_mat = transformed_data.block(0,0,transformed_data.rows()-1,transformed_data.cols());
	return transformed_data_mat.transpose();
}


pcl::PointCloud<pcl::PointXYZRGB> transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,Eigen::MatrixXd& a_T_b)
{
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	for(auto point:point_cloud->points)
	{
		Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
		pts(0,0)=point.x;
		pts(0,1)=point.y;
		pts(0,2)=point.z;
		Eigen::MatrixXd pts_trans=apply_transformation(pts,a_T_b);
		pcl::PointXYZRGB pt;
		pt.x=pts_trans(0,0);
		pt.y=pts_trans(0,1);
		pt.z=pts_trans(0,2);
		pt.rgb=point.rgb;
		cloud.push_back(pt);
	}
	return cloud;
}

Eigen::MatrixXd transformPolygon(Eigen::MatrixXd& polygon,Eigen::MatrixXd& a_T_b)
{
	Eigen::MatrixXd trans=MatrixXd::Zero(polygon.rows(),3);
	for(int i=0;i<polygon.rows();i++)
	{
		Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
		pts(0,0)=polygon(i,0);
		pts(0,1)=polygon(i,1);
		pts(0,2)=polygon(i,2);
		Eigen::MatrixXd pts_trans=apply_transformation(pts,a_T_b);
		trans(i,0)=pts_trans(0,0);
		trans(i,1)=pts_trans(0,1);
	}
	return trans;
}

void publishPointCloudTrimmed(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,vector<vector<double>> &bounding_polygon_local)
{
  if(cloud->points.size()==0)
    return;
  if(bounding_polygon_local.size()==0)
    return;
	Eigen::MatrixXd a_T_b = transformation_global;
    Eigen::MatrixXd boundary_temp=MatrixXd::Zero(bounding_polygon.size(),3);
    for(int i=0;i<bounding_polygon.size();i++)
    {
    	boundary_temp(i,0)=bounding_polygon_local[i][0];
    	boundary_temp(i,1)=bounding_polygon_local[i][1];
    	boundary_temp(i,2)=bounding_polygon_local[i][2];
    }
    Eigen::MatrixXd boundary=transformPolygon(boundary_temp,a_T_b);
/*    cout<<"Before Transformation..";
    for(int i=0;i<boundary_temp.rows();i++)
    	cout<<boundary_temp(i,0)<<" "<<boundary_temp(i,1)<<endl;
    cout<<"After Transformation.."<<endl;
    for(int i=0;i<boundary.rows();i++)
    	cout<<boundary(i,0)<<" "<<boundary(i,1)<<endl;
    cout<<"-------------------------"<<endl;*/
	pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
	pcl::PointCloud<pcl::PointXYZRGB> cloud_t=transformPointCloud(cloud,a_T_b);
	PCLUtilities::PclToPcd<pcl::PointXYZRGB>("/home/rex/Data/test/transformed.pcd",cloud_t);
	int count=0;
	for(int i=0;i<cloud_t.points.size();i++)
	{
    	Eigen::MatrixXd point(1,2);
		point<<cloud_t.points[i].x,cloud_t.points[i].y;
		Eigen::MatrixXd state_boundary=InPoly(point,boundary);
		if(state_boundary(0,0))
		{
			pcl_cloud.points.push_back(cloud->points[i]);
			count++;
		}
	}
	// Eigen::MatrixXd b_T_a = a_T_b.inverse();
	// pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud=transformPointCloud(pcl_cloud_t.makeShared(),b_T_a);
	pcl_cloud.width = count;
	pcl_cloud.height = 1;
	pcl_cloud.is_dense = true;
	pcl_cloud.header.frame_id = output_frame;
    cloud_trimmed_global=pcl_cloud;
	PCLUtilities::publishPointCloud<pcl::PointXYZRGB>(pcl_cloud,Publish_Cloud_Trimmed);   
}

pcl::PointCloud<pcl::PointXYZRGB> clicked_pcd;
int count_points=0;

void getData(const geometry_msgs::PointStamped& msg)
{
  pcl::PointXYZRGB pt;
  pt.x=msg.point.x;
  pt.y=msg.point.y;
  pt.z=msg.point.z;
  uint32_t red=255,green=0,blue=0;
  uint32_t rgb = (static_cast<uint32_t>(red) << 16 |
              static_cast<uint32_t>(green) << 8 | static_cast<uint32_t>(blue));
  pt.rgb = *reinterpret_cast<float*>(&rgb);
  clicked_pcd.push_back(pt);
  cout << "point #"<<clicked_pcd.width*clicked_pcd.height<<": "<<pt.x<<" "<< pt.y<<" "<< pt.z<<endl;
  clicked_pcd.width=++count_points;
  clicked_pcd.height=1;
  clicked_pcd.is_dense=false;
  clicked_pcd.header.frame_id="pcl";
  PCLUtilities::publishPointCloud<pcl::PointXYZRGB>(clicked_pcd,Clicked_Points);
}

// Eigen::MatrixXd getPlane(Eigen::MatrixXd& points)
// {
// 	Eigen::MatrixXd ab = points.block<1,3>(1,0)-points.block<1,3>(0,0);
// 	Eigen::MatrixXd ac = points.block<1,3>(2,0)-points.block<1,3>(0,0);
// 	Eigen::MatrixXd partial = ab.cross(ac);
// 	Eigen::MatrixXd equation = MatrixXd::Zero(1,4);
// 	equation.block<1,3>(0,0)=partial;
// 	equation.block<1,1>(0,3)=partial.dot(points.block<1,3>(0,0));
// 	return equation;
// }

inline Eigen::MatrixXd getPlane(Eigen::MatrixXd& points)
{ 
	double x1=points(0,0),x2=points(1,0),x3=points(2,0),y1=points(0,1),y2=points(1,1),y3=points(2,1),z1=points(0,2),z2=points(1,2),z3=points(2,2);
	double a1 = x2 - x1,b1 = y2 - y1,c1 = z2 - z1,a2=x3 - x1,b2 = y3 - y1,c2 = z3 - z1; 
    Eigen::MatrixXd equation=MatrixXd::Zero(1,4);
    equation(0,0) = b1 * c2 - b2 * c1; 
    equation(0,1) = a2 * c1 - a1 * c2; 
    equation(0,2) = a1 * b2 - b1 * a2; 
    equation(0,3) = (- equation(0,0) * x1 - equation(0,1) * y1 - equation(0,2) * z1); 
    return equation;
} 

inline double getAngle(Eigen::MatrixXd& plane,Eigen::MatrixXd& plane_ref)
{
	double a1=plane(0,0),b1=plane(0,1),c1=plane(0,2);
	double a2=plane_ref(0,0),b2=plane_ref(0,1),c2=plane_ref(0,2);
	double d = (a1*a2 + b1*b2 + c1*c2); 
	double e1 = sqrt(a1*a1 + b1*b1 + c1*c1); 
	double e2 = sqrt(a2*a2 + b2*b2 + c2*c2); 
	d = d/(e1*e2); 
	double A = (acos(d)); 
	return A;	
}

bool reset(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
  res.success=true;
  bounding_polygon.clear();
  clicked_pcd.points.clear();
  count_points=0;
  return true;
}

bool savePointCloud(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
  res.success=true;
  PCLUtilities::PclToPcd<pcl::PointXYZRGB>(filename_global,cloud_trimmed_global);
  return true;
}

bool trimPointCloud(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
  res.success = false;
  bounding_polygon.clear();
  for(auto point:clicked_pcd.points)
    bounding_polygon.push_back(vector<double>({point.x,point.y,point.z}));
  if(bounding_polygon.size()<3)
    return false;
  Eigen::MatrixXd points = MatrixXd::Zero(3,3);
  for(int i=0;i<3;i++)
  {
    points(i,0)=clicked_pcd.points[i].x;
    points(i,1)=clicked_pcd.points[i].y;
    points(i,2)=clicked_pcd.points[i].z;
  }
	Eigen::Vector3d X = points.block<1,3>(1,0)-points.block<1,3>(0,0);
	Eigen::Vector3d Y = points.block<1,3>(2,0)-points.block<1,3>(0,0);
	X=X/X.norm();
	Y=Y/Y.norm();
	Eigen::Vector3d Z = X.cross(Y);
	Y = Z.cross(X);
	Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4,4);
	T.block<3,1>(0,0) = X.transpose();
	T.block<3,1>(0,1) = Y.transpose();
	T.block<3,1>(0,2) = Z.transpose();
	transformation_global=T.inverse();
  // X=X.array()/X.norm();

  res.success=true;
  return true;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud;

void onReceivedPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_in)
{
    // Convert to useful point cloud format
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud_in, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud = PCLUtilities::pointCloud2ToPclXYZRGB(pcl_pc2);
  global_cloud = cloud.makeShared();
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "merge_point_clouds");
	ros::NodeHandle pnh("~");
  ros::Subscriber point_cloud_sub = pnh.subscribe("input_point_cloud",1,&onReceivedPointCloud);
  Publish_Cloud_Trimmed = pnh.advertise<sensor_msgs::PointCloud2> ("/approximate_bb/cloud_trimmed", 1);
	Clicked_Points = pnh.advertise<sensor_msgs::PointCloud2> ("/approximate_bb/clicked_points", 1);
  ros::Subscriber point_sub_ = pnh.subscribe("/clicked_point", 10,getData);
	ros::ServiceServer trime_point_cloud=pnh.advertiseService("/approximate_bb/trim_point_cloud",trimPointCloud);
  ros::ServiceServer reset_all=pnh.advertiseService("/approximate_bb/reset",reset);
  ros::ServiceServer save_pcl=pnh.advertiseService("/approximate_bb/savePointCloud",savePointCloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZRGB>);
  global_cloud=cloud_temp;
  pnh.param<std::string>("output_frame", output_frame, "output_frame");
  pnh.param<std::string>("filename_global", filename_global, "filename_global");
  ros::Rate r(1);
	while(ros::ok())
	{
 	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = global_cloud;
		publishPointCloudTrimmed(cloud,bounding_polygon);
		r.sleep();
		ros::spinOnce();
	}
	return 0;
}
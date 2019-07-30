#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <string>
#include <fstream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>

using namespace std;
using namespace message_filters;

class Recorder
{
public:
	Recorder(ros::NodeHandle& nh);
	void write_oneline(const vector<double>& f_t_data, ofstream& outFile);
	void callback_quads(const nav_msgs::OdometryConstPtr& odom0, 
							   const geometry_msgs::WrenchStamped::ConstPtr& data_ft0,
							   const nav_msgs::OdometryConstPtr& odom1, 
							   const geometry_msgs::WrenchStamped::ConstPtr& data_ft1,
							   const nav_msgs::OdometryConstPtr& odom2, 
							   const geometry_msgs::WrenchStamped::ConstPtr& data_ft2,
							   const nav_msgs::OdometryConstPtr& odom3, 
							   const geometry_msgs::WrenchStamped::ConstPtr& data_ft3);

	void writeDataFromMsg(const nav_msgs::OdometryConstPtr& odom, 
						  const geometry_msgs::WrenchStamped::ConstPtr& data_ft, 
						  vector<double>& v_data_combine, 
						  ofstream& outFile);

private:
	vector<string> namespaces;
	int num_data;
	ofstream outFile_0;
	ofstream outFile_1;
	ofstream outFile_2;
	ofstream outFile_3;

	string filename_0;
	string filename_1;
	string filename_2;
	string filename_3;

	std::vector<double> v_data_combine_0; 
	std::vector<double> v_data_combine_1; 
	std::vector<double> v_data_combine_2; 
	std::vector<double> v_data_combine_3; 

};

Recorder::Recorder(ros::NodeHandle& nh):
	num_data(9),
	v_data_combine_0(num_data, 0.0),
	v_data_combine_1(num_data, 0.0),
	v_data_combine_2(num_data, 0.0),
	v_data_combine_3(num_data, 0.0)
{
	time_t t = time(0);
    char tmp[64];
    strftime( tmp, sizeof(tmp), "%Y_%m_%d_%X",localtime(&t) );

    char c_ft_data0[100];
    sprintf(c_ft_data0, "hummingbird_0_%s.csv", tmp);
    cout<<"----- file force torque data: "<< c_ft_data0<<endl;
    filename_0 = c_ft_data0;

    char c_ft_data1[100];
    sprintf(c_ft_data1,"hummingbird_1_%s.csv",tmp);
    cout<<"----- file force torque data: "<< c_ft_data1<<endl;
    filename_1 = c_ft_data1;

    char c_ft_data2[100];
    sprintf(c_ft_data2,"hummingbird_2_%s.csv", tmp);
    cout<<"----- file force torque data: "<< c_ft_data2<<endl;
    filename_2 = c_ft_data2;

    char c_ft_data3[100];
    sprintf(c_ft_data3,"hummingbird_3_%s.csv", tmp);
    cout<<"----- file force torque data: "<< c_ft_data3<<endl;
    filename_3 = c_ft_data3;

    outFile_0.open(filename_0);
    outFile_1.open(filename_1);
    outFile_2.open(filename_2);
    outFile_3.open(filename_3);
	
  	message_filters::Subscriber<nav_msgs::Odometry> Odom_sub_0(nh, "/hummingbird_0/ground_truth/odometry", 2);
 	message_filters::Subscriber<geometry_msgs::WrenchStamped> ft_sub_0(nh, "/hummingbird_0/f_t", 2);

 	message_filters::Subscriber<nav_msgs::Odometry> Odom_sub_1(nh, "/hummingbird_1/ground_truth/odometry", 2);
 	message_filters::Subscriber<geometry_msgs::WrenchStamped> ft_sub_1(nh, "/hummingbird_1/f_t", 2);

 	message_filters::Subscriber<nav_msgs::Odometry> Odom_sub_2(nh, "/hummingbird_2/ground_truth/odometry", 2);
 	message_filters::Subscriber<geometry_msgs::WrenchStamped> ft_sub_2(nh, "/hummingbird_2/f_t", 2);

 	message_filters::Subscriber<nav_msgs::Odometry> Odom_sub_3(nh, "/hummingbird_3/ground_truth/odometry", 2);
 	message_filters::Subscriber<geometry_msgs::WrenchStamped> ft_sub_3(nh, "/hummingbird_3/f_t", 2);

 	typedef sync_policies::ApproximateTime<
 										   nav_msgs::Odometry, geometry_msgs::WrenchStamped, 
 										   nav_msgs::Odometry, geometry_msgs::WrenchStamped,
 										   nav_msgs::Odometry, geometry_msgs::WrenchStamped,
 										   nav_msgs::Odometry, geometry_msgs::WrenchStamped
 										   > MySyncPolicy;

 	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), Odom_sub_0, ft_sub_0, Odom_sub_1, ft_sub_1, Odom_sub_2, ft_sub_2, Odom_sub_3, ft_sub_3);
 	sync.registerCallback(boost::bind(&Recorder::callback_quads, this, _1, _2, _3, _4, _5, _6, _7, _8));

 	cout << "debug here before spin" << endl;
 	ros::spin();

}

void Recorder::callback_quads(const nav_msgs::OdometryConstPtr& odom0, 
							   const geometry_msgs::WrenchStamped::ConstPtr& data_ft0,
							   const nav_msgs::OdometryConstPtr& odom1, 
							   const geometry_msgs::WrenchStamped::ConstPtr& data_ft1,
							   const nav_msgs::OdometryConstPtr& odom2, 
							   const geometry_msgs::WrenchStamped::ConstPtr& data_ft2,
							   const nav_msgs::OdometryConstPtr& odom3, 
							   const geometry_msgs::WrenchStamped::ConstPtr& data_ft3)
{
	writeDataFromMsg(odom0, data_ft0, v_data_combine_0, outFile_0);

	writeDataFromMsg(odom1, data_ft1, v_data_combine_1, outFile_1);

	writeDataFromMsg(odom2, data_ft2, v_data_combine_2, outFile_2);

	writeDataFromMsg(odom3, data_ft3, v_data_combine_3, outFile_3);
}

void Recorder::writeDataFromMsg(const nav_msgs::OdometryConstPtr& odom, 
							    const geometry_msgs::WrenchStamped::ConstPtr& data_ft, 
								vector<double>& v_data_combine, 
								ofstream& outFile)
{
	double temp_odom_ft_data[num_data] = 
					{
						odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z, 
						data_ft->wrench.force.x, data_ft->wrench.force.y, data_ft->wrench.force.z, 
						data_ft->wrench.torque.x, data_ft->wrench.torque.y, data_ft->wrench.torque.z
					};
	
	if (!v_data_combine.empty())
	{
		v_data_combine.clear();
	}
	v_data_combine.insert(v_data_combine.begin(), temp_odom_ft_data, temp_odom_ft_data + num_data);
	// cout << v_data_combine[0] << endl << v_data_combine[1];

	write_oneline(v_data_combine, outFile);
}

void Recorder::write_oneline(const vector<double>& f_t_data, ofstream& outFile)
{
	// outFile_0.open(filename_0, ios::out);
	for(auto i_vec : f_t_data){
		char c[50];
		sprintf(c, "%f", i_vec);
		outFile << c << ",";
	}
	outFile << endl;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "force_recorder");
	ros::NodeHandle nh;
	Recorder recorder(nh);
	return 0;
}
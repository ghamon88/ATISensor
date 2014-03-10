// author: Guillaume Hamon

#ifndef OROCOS_ATISENSOR_COMPONENT_HPP
#define OROCOS_ATISENSOR_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <rtnet.h>
#include <rtdm/rtdm.h>
#include <sys/types.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <tinyxml2.h>
#include <happyhttp.h>

#include <Eigen/Dense>
#include <kdl/chainfksolver.hpp>
#include <kdl/chain.hpp>
#include <directKinematics-types.hpp>
#include <fcntl.h>
#include <cmath>
#include <algorithm>



#define PORT 49152 /* Port the Net F/T always uses */
#define COMMAND 2 /* Command code 2 starts real time streaming */
#define NUM_SAMPLES 0 /* Will send infinite number of samples */



class ATISensor : public RTT::TaskContext{
  public:

	typedef struct response_struct {
		unsigned int rdt_sequence;
		unsigned int ft_sequence;
		unsigned int status;
		int FTData[6];
	} RESPONSE;

	RESPONSE resp;		/* The structured response received from the Net F/T. */
	int socketHandle;	/* Handle to UDP socket used to communicate with Net F/T. */
        unsigned char request[8];	/* The request data sent to the Net F/T. */
	unsigned char response[36];	/* The raw response data received from the Net F/T. */

	int cfgcpt;
	int cfgcpf;

	bool bias_done;
	bool bias_asked;
	bool calibration_ended;

	double Px,Py,Pz,P,Gx,Gy,Gz,Pcx,Pcy,Pcz;
        std::vector<DirectKinematicsData> robot_transforms;

	std::vector<double> FTvalues;
 	Eigen::Matrix<double,3,6> calibration_matrix;

	int webserver_connection; //property, se connecte au server web pour récupérer cfgcpt cfgcpf

        RTT::InputPort< Eigen::Matrix<double,3,6> > iport_FT_calibration_data;
	RTT::InputPort< std::vector<DirectKinematicsData> > iport_transforms;
	RTT::InputPort< bool > iport_bias;

	RTT::OutputPort< int > oport_FTData_Fx;
	RTT::OutputPort< int > oport_FTData_Fy;
	RTT::OutputPort< int > oport_FTData_Fz;
	RTT::OutputPort< int > oport_FTData_Tx;
	RTT::OutputPort< int > oport_FTData_Ty;
	RTT::OutputPort< int > oport_FTData_Tz;
	RTT::OutputPort< float > oport_Fnorm;
	RTT::OutputPort< std::vector<double> > oport_FTvalues;


    	ATISensor(std::string const& name);
   	bool configureHook();
    	bool startHook();
    	void updateHook();
   	void stopHook();
    	void cleanupHook();
	void setWebserver(int);


};
#endif

#ifndef OROCOS_ATISENSOR_COMPONENT_HPP
#define OROCOS_ATISENSOR_COMPONENT_HPP

#include <rtt/RTT.hpp>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <sys/types.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <tinyxml2.h>
#include <happyhttp.h>
#include <Eigen/Dense>
#include <kdl/chainfksolver.hpp>
#include <directKinematics-types.hpp>

#define PORT 49152 /* Port the Net F/T always uses */
#define COMMAND 2 /* Command code 2 starts real time streaming */
#define NUM_SAMPLES 0 /* Will send infinite number of samples */



class ATISensor : public RTT::TaskContext{
  public:

    /* Typedefs used so integer sizes are more explicit */
	//typedef unsigned int uint32;
	//typedef int int32;
	//typedef unsigned short uint16;
	//typedef short int16;
	//typedef unsigned char byte;
	typedef struct response_struct {
		unsigned int rdt_sequence;
		unsigned int ft_sequence;
		unsigned int status;
		int FTData[6];
	} RESPONSE;
	
	int cfgcpt;
	int cfgcpf;

	double Px,Py,Pz,P,Gx,Gy,Gz;
        std::vector<DirectKinematicsData> robot_transforms;

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
	bool bias_done;
	bool bias_asked;

	bool calibration_ended;
	int socketHandle;	/* Handle to UDP socket used to communicate with Net F/T. */
        unsigned char request[8];	/* The request data sent to the Net F/T. */
	RESPONSE resp;		/* The structured response received from the Net F/T. */
	unsigned char response[36];	/* The raw response data received from the Net F/T. */
        //std::string AXES[6];

    	ATISensor(std::string const& name);
   	bool configureHook();
    	bool startHook();
    	void updateHook();
   	void stopHook();
    	void cleanupHook();
	void setWebserver(int);
	int webserver_connection; //property, se connecte au server web pour récupérer cfgcpt cfgcpf
};
#endif

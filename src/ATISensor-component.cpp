// copyright 2014 ISIR-CNRS
// author Guillaume Hamon

#include "ATISensor-component.hpp"

FILE* fichier;
int compteur = 0;

void onBegin( const happyhttp::Response* r, void* userdata );
void onData( const happyhttp::Response* r, void* userdata, const unsigned char* data, int n );
void onComplete( const happyhttp::Response* r, void* userdata );


ATISensor::ATISensor(std::string const& name) : TaskContext(name){

  this->addPort("FT_calibration_data_i", iport_FT_calibration_data);
  this->addPort("transforms_i", iport_transforms);
  this->addPort("bias_i",iport_bias);

  this->addPort("Fnorm", oport_Fnorm);
  this->addPort("FTvalues", oport_FTvalues);

  this->addOperation("setWebserver", &ATISensor::setWebserver, this, RTT::OwnThread); //specify if the component should connect to the sensor webserver to retrieve configuration parameters values (not feasible in real-time communication)

  std::cerr << "ATISensor constructed !" <<std::endl;
}

bool ATISensor::configureHook(){
	//RTT::Logger::In in("ATI");
	//RTT::Logger::log().allowRealTime();

	// variables initialization
	calibration_ended = false;
	bias_done=false;
	bias_asked=false;
	robot_transforms.resize(8);
	FTvalues.resize(6);

	*(unsigned short*)&request[0] = htons(0x1234); // standard header.
	*(unsigned short*)&request[2] = htons(COMMAND); // per table 9.1 in Net F/T user manual.
	*(unsigned int*)&request[4] = htonl(NUM_SAMPLES); // see section 9.1 in Net F/T user manual.

	/* Use of happyhttp library */
	if(webserver_connection)
	{
		fichier=fopen("/home/kuka/src/groovy_workspace/orocos/ATISensor/xmlget.xml","w");
		std::cout<< "tentative de connexion http ..." << std::endl;
		happyhttp::Connection conn( "192.168.100.103", 80 );
		conn.setcallbacks(onBegin, onData, onComplete, 0 );
		conn.request( "GET", "/netftapi2.xml?index=0", 0, 0,0 );
		while( conn.outstanding() )
			conn.pump();
		fclose(fichier);

		/* Use of Tinyxml library */

		char docName[]="/home/kuka/src/groovy_workspace/orocos/ATISensor/xmlget.xml";
        	tinyxml2::XMLDocument doc;
		std::cout << "LOAD "  << docName << std::endl;
        	if(doc.LoadFile(docName)) return 0;
        	tinyxml2::XMLHandle docHandle(&doc );

		tinyxml2::XMLElement* child = docHandle.FirstChildElement("netft").FirstChildElement("cfgcpf").ToElement();
		if(!child) return 0;
		cfgcpf = atoi(child->GetText());

		child = docHandle.FirstChildElement("netft").FirstChildElement("cfgcpt").ToElement();
		if(!child) return 0;
		cfgcpt = atoi(child->GetText());
	}else
	{
		// Hardcoded for now, webserver is not accessible in realtime communication, TODO: set these as properties to avoid recompiling
		cfgcpf=1000000;
		cfgcpt=1000000;
	}
	std::cout << "cfgcpf = " << cfgcpf <<std::endl;
 	std::cout << "cfgcpt = " << cfgcpt <<std::endl;

  std::cout << "ATISensor configured !" <<std::endl;
  return true;
}

bool ATISensor::startHook(){

  /* Start request to get sensor data */


	struct sockaddr_in addr;	/* Address of Net F/T. */
	int err;			/* Error status of operations. */

	/* rtnet */
	/* Open socket here. */
	socketHandle = rt_dev_socket(AF_INET, SOCK_DGRAM, 0);
	if (socketHandle == -1){
		std::cout << "Socket could not be opened" << std::endl;
		exit(1);
	}

	addr.sin_addr.s_addr = inet_addr("192.168.100.103");
	addr.sin_family = AF_INET;
	addr.sin_port = htons(PORT);

	err = rt_dev_connect(socketHandle, (struct sockaddr *)&addr, sizeof(addr) );
	if(err==-1){
		std::cout << "connection failed" << std::endl;
		exit(2);
	}
	rt_dev_send(socketHandle, (const char *)request, 8, 0 ); // send start command to the sensor net box

  std::cout << "ATISensor started !" <<std::endl;

  return true;
}

void ATISensor::updateHook(){
	float Fnorm;
        int i; /* Generic loop/array index. */

	if(iport_bias.connected()){
		RTT::FlowStatus bias_fs=iport_bias.read(bias_asked);
		if(bias_fs==RTT::NewData){
			if(!bias_done && bias_asked){
				*(unsigned short*)&request[2] = htons(0); // stop, per table 9.1 in Net F/T user manual.
				rt_dev_send(socketHandle, (const char *)request, 8, 0 );
				*(unsigned short*)&request[2] = htons(66); // bias, per table 9.1 in Net F/T user manual.
				rt_dev_send(socketHandle, (const char *)request, 8, 0 );
				*(unsigned short*)&request[2] = htons(2); // start, per table 9.1 in Net F/T user manual.
				rt_dev_send(socketHandle, (const char *)request, 8, 0 );
				std::cout<< "bias order sent "<<std::endl;
				bias_done=true;
			}
		}
	}
	if(calibration_ended){
		RTT::FlowStatus transforms_fs=iport_transforms.read(robot_transforms);
		if(transforms_fs==RTT::NewData){

			/* Computations with the sensor uniform density hypothesis */
			//Px=-P*(2*robot_transforms[7].rotation[0]*robot_transforms[7].rotation[2]-2*robot_transforms[7].rotation[1]*robot_transforms[7].rotation[3]);
			//Py=-P*(2*robot_transforms[7].rotation[1]*robot_transforms[7].rotation[2]+2*robot_transforms[7].rotation[0]*robot_transforms[7].rotation[3]);
			//Pz=-P*(1-2*robot_transforms[7].rotation[0]*robot_transforms[7].rotation[0]-2*robot_transforms[7].rotation[1]*robot_transforms[7].rotation[1]);

			/* Computations with the sensor non uniform density hypothesis */
			Px=-Pcx*(2*robot_transforms[7].rotation[0]*robot_transforms[7].rotation[2]-2*robot_transforms[7].rotation[1]*robot_transforms[7].rotation[3]);
			Py=-Pcy*(2*robot_transforms[7].rotation[1]*robot_transforms[7].rotation[2]+2*robot_transforms[7].rotation[0]*robot_transforms[7].rotation[3]);
			Pz=-Pcz*(1-2*robot_transforms[7].rotation[0]*robot_transforms[7].rotation[0]-2*robot_transforms[7].rotation[1]*robot_transforms[7].rotation[1]);

			Px=-Px; //Xwrist & Xsensor are in opposite directions
			Py=-Py; //Ywrist & Ysensor are in opposite directions

			//std::cout << "P = "<< P << " Px = " << Px << " Py = " << Py << "Pz = " << Pz << " Gx = "<< Gx << " Gy = " << Gy << " Gz = "<< Gz <<std::endl;
		}
	}

	/* rtnet */
	/* Receiving the response. */
	rt_dev_recv(socketHandle, (char *)response, 36, 0 );

	resp.rdt_sequence = ntohl(*(unsigned int*)&response[0]);
	resp.ft_sequence = ntohl(*(unsigned int*)&response[4]);
	resp.status = ntohl(*(unsigned int*)&response[8]);
	for( i = 0; i < 6; i++ ) {
		resp.FTData[i] = ntohl(*(int*)&response[12 + i * 4]);
	}

	/* Converting to the right units e.g N and Nm */
	float Fx=(float)resp.FTData[0]/(float)cfgcpf;
	float Fy=(float)resp.FTData[1]/(float)cfgcpf;
	float Fz=(float)resp.FTData[2]/(float)cfgcpf;

	float Tx=(float)resp.FTData[3]/(float)cfgcpt;
	float Ty=(float)resp.FTData[4]/(float)cfgcpt;
	float Tz=(float)resp.FTData[5]/(float)cfgcpt;


	if(calibration_ended){
		Fx-=Px;
		Fy-=Py;

		/* uniform density */
		//Fz=Fz-Pz+P; //offset(bias)=-P , compensation= Value_read - Pz in sensor frame - offset
		//Tx=Tx-Gy*Pz+Gz*Py+Gy*P;
		//Ty=Ty+Gx*Pz+Gz*Px-Gx*P;

		/* non uniform density */
		Fz=Fz-Pz+Pcz;
		Tx=Tx-Gy*Pz+Gz*Py+Gy*Pcz;
                Ty=Ty+Gx*Pz+Gz*Px-Gx*Pcz;
		Tz=Tz-Gy*Px-Gx*Py;
	}

	/* Norme of the forces */
	Fnorm=sqrt(Fx*Fx+Fy*Fy+Fz*Fz);

	FTvalues[0]=Fx;
	FTvalues[1]=Fy;
	FTvalues[2]=Fz;
	FTvalues[3]=Tx;
	FTvalues[4]=Ty;
	FTvalues[5]=Tz;

	oport_FTvalues.write(FTvalues);
	oport_Fnorm.write(Fnorm);

	if(iport_FT_calibration_data.connected()){
		RTT::FlowStatus calibration_matrix_fs = iport_FT_calibration_data.read(calibration_matrix);
		if(calibration_matrix_fs == RTT::NewData && !calibration_ended){
			std::cout<< calibration_matrix(0,0) << " " << calibration_matrix(0,1) << " " << calibration_matrix(0,2) << std::endl;
			std::cout<< calibration_matrix(1,0) << " " << calibration_matrix(1,1) << " " << calibration_matrix(1,2) << std::endl;
			std::cout<< calibration_matrix(2,0) << " " << calibration_matrix(2,1) << " " << calibration_matrix(2,2) << std::endl;

		/* uniform density */
		//P=(std::abs(calibration_matrix(1,0))+std::abs(calibration_matrix(1,2))+std::abs(calibration_matrix(2,1))+std::abs(calibration_matrix(2,2)))/4;
		//Gx=(calibration_matrix(2,4)+std::abs(calibration_matrix(2,5)))/(2*P);
		//Gy=(-calibration_matrix(1,5)-calibration_matrix(1,3))/(2*P);
		//Gz=(-Gx*P+Gy*P+calibration_matrix(2,3)+calibration_matrix(1,4))/(2*P);

			/* non uniform density */
			Pcx=std::abs(calibration_matrix(1,0));
			Pcy=std::abs(calibration_matrix(2,1));
			Pcz=(std::abs(calibration_matrix(1,2))+std::abs(calibration_matrix(2,2)))/2;

			Gx=((calibration_matrix(2,4)/Pcz)+(std::abs(calibration_matrix(2,5))/Pcy))/2;
                	Gy=((-calibration_matrix(1,5)/Pcx)-(calibration_matrix(1,3)/Pcz))/2;
                	Gz=(((calibration_matrix(1,4)-Gx*Pcz)/Pcx)+((calibration_matrix(2,3)+Gy*Pcz)/Pcy))/2;

			calibration_ended=true;
		}
	}

  // std::cerr << Fnorm << " " << Fx << " " << Fy << " " << Fz << " "<< Tx << " " << Ty << " "<< Tz <<std::endl;
  // RTT::Logger::log()<<RTT::Logger::Info<<" UpdateHook executed " <<RTT::Logger::endl;
  // std::cout << "Fnorm = "<< Fnorm << " Fx = " << Fx << " Fy = " << Fy << "Fz = " << Fz << " Tx = "<< Tx << " Ty = " << Ty << " Tz = "<< Tz <<std::endl;
  // std::cout << "P = "<< P << " Px = " << Px << " Py = " << Py << "Pz = " << Pz << " Gx = "<< Gx << " Gy = " << Gy << " Gz = "<< Gz <<std::endl;
  // std::cout << "calibration_ended = "<< calibration_ended << " bias_done = " << bias_done << " bias_asked = " << bias_asked <<std::endl;
}

void ATISensor::stopHook() {

	/* stop request getting data sensor */
	*(unsigned short*)&request[2] = htons(0); // stop,  per table 9.1 in Net F/T user manual.

	/* rtnet */
	 rt_dev_send(socketHandle, (const char *)request, 8, 0 );
	 rt_dev_close(socketHandle);

	std::cout << "ATISensor executes stopping !" <<std::endl;
}

void ATISensor::cleanupHook() {

  shutdown(socketHandle,2);

  std::cout << "shutdown ok" <<std::endl;
  std::cout << "ATISensor cleaning up !" <<std::endl;

}

void onBegin( const happyhttp::Response* r, void* userdata )
{
	//printf( "BEGIN (%d %s)\n", r->getstatus(), r->getreason() );
	compteur = 0;
}

void onData( const happyhttp::Response* r, void* userdata, const unsigned char* data, int n )
{
	fwrite( data,1,n, fichier );
	compteur += n;
}

void onComplete( const happyhttp::Response* r, void* userdata )
{
	//printf( "COMPLETE (%d bytes)\n",compteur );
}

void ATISensor::setWebserver(int m){
	webserver_connection=m;
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(ATISensor)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(ATISensor)

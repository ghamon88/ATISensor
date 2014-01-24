#include "ATISensor-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <fcntl.h>
#include <happyhttp.h>


FILE* fichier;
int compteur = 0;

void onBegin( const happyhttp::Response* r, void* userdata );
void onData( const happyhttp::Response* r, void* userdata, const unsigned char* data, int n );
void onComplete( const happyhttp::Response* r, void* userdata );

ATISensor::ATISensor(std::string const& name) : TaskContext(name){
  this->addPort("FTData_Fx", oport_FTData_Fx);
  this->addPort("FTData_Fy", oport_FTData_Fy);
  this->addPort("FTData_Fz", oport_FTData_Fz);
  this->addPort("FTData_Tx", oport_FTData_Tx);
  this->addPort("FTData_Ty", oport_FTData_Ty);
  this->addPort("FTData_Tz", oport_FTData_Tz);
  std::cout << "ATISensor constructed !" <<std::endl;
}

bool ATISensor::configureHook(){

	/* The names of the force and torque axes for display */
	/*AXES[0] = "Fx";
	AXES[1] = "Fy";
	AXES[2] = "Fz";
	AXES[3] = "Tx";
	AXES[4] = "Ty";
	AXES[5] = "Tz";*/			

	*(unsigned short*)&request[0] = htons(0x1234); // standard header. 
	*(unsigned short*)&request[2] = htons(COMMAND); // per table 9.1 in Net F/T user manual. 
	*(unsigned int*)&request[4] = htonl(NUM_SAMPLES); // see section 9.1 in Net F/T user manual. 		

	/* Use of happyhttp library */

	fichier=fopen("/home/kuka/src/groovy_workspace/orocos/ATISensor/xmlget.xml","w");
	happyhttp::Connection conn( "192.168.1.1", 80 );
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

	//Plutôt utiliser un port d'entrée pour les valeurs de calibrations si celles-ci sont changées en dynamique
	
	std::cout << "cfgcpf = " << cfgcpf <<std::endl;
 	std::cout << "cfgcpt = " << cfgcpt <<std::endl;

  std::cout << "ATISensor configured !" <<std::endl;
  return true;
}

bool ATISensor::startHook(){

  /* Start request to get sensor data */

  /* Calculate number of samples, command code, and open socket here. */

	struct sockaddr_in addr;	/* Address of Net F/T. */
	int err;			/* Error status of operations. */

	socketHandle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (socketHandle == -1) {
		std::cout << "Socket could not be opened" <<std::endl;
		exit(1);
	}

	addr.sin_addr.s_addr=inet_addr("192.168.1.1");
	addr.sin_family = AF_INET;
	addr.sin_port = htons(PORT);

	err = connect( socketHandle, (struct sockaddr *)&addr, sizeof(addr) );
	if (err == -1) {
		std::cout << "connection failed" <<std::endl;
		exit(2);
	}

  send( socketHandle, (const char *)request, 8, 0 );

  std::cout << "ATISensor started !" <<std::endl;

  return true;
}

void ATISensor::updateHook(){
        int i; /* Generic loop/array index. */
  	/* Receiving the response. */
	recv( socketHandle, (char *)response, 36, 0 );
	resp.rdt_sequence = ntohl(*(unsigned int*)&response[0]);
	resp.ft_sequence = ntohl(*(unsigned int*)&response[4]);
	resp.status = ntohl(*(unsigned int*)&response[8]);
	for( i = 0; i < 6; i++ ) {
		resp.FTData[i] = ntohl(*(int*)&response[12 + i * 4]);
	}
	
	/* Output the response data */
	//printf( "Status: 0x%08x\n", resp.status );
	/*for (i =0;i < 6;i++) {
		printf("%s: %d\n", AXES[i], resp.FTData[i]);
	}*/

	resp.FTData[0]/=cfgcpf;
	resp.FTData[1]/=cfgcpf;
	resp.FTData[2]/=cfgcpf;
	
	resp.FTData[3]/=cfgcpt;
	resp.FTData[4]/=cfgcpt;
	resp.FTData[5]/=cfgcpt;

	oport_FTData_Fx.write(resp.FTData[0]);
	oport_FTData_Fy.write(resp.FTData[1]);
	oport_FTData_Fz.write(resp.FTData[2]);
	oport_FTData_Tx.write(resp.FTData[3]);
	oport_FTData_Ty.write(resp.FTData[4]);
	oport_FTData_Tz.write(resp.FTData[5]);

  /*std::cout << "ATISensor executes updateHook !" <<std::endl;*/
}

void ATISensor::stopHook() {

  /* stop request getting data sensor */

  *(unsigned short*)&request[2] = htons(0); // per table 9.1 in Net F/T user manual. 
  send( socketHandle, (const char *)request, 8, 0 );

  close(socketHandle);

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

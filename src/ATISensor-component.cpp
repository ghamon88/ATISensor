#include "ATISensor-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <fcntl.h>
#include <happyhttp.h>


FILE* fichier;
int compteur = 0;

void Begin( const happyhttp::Response* r, void* userdata );
void Data( const happyhttp::Response* r, void* userdata, const unsigned char* data, int n );
void Complete( const happyhttp::Response* r, void* userdata );

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

	struct sockaddr_in addr;	/* Address of Net F/T. */
	int err;			/* Error status of operations. */
	const char* resp_cfgcpf;
	const char* resp_cfgcpt;
	resp_cfgcpf=(char*)malloc(10*sizeof(char));
	resp_cfgcpt=(char*)malloc(10*sizeof(char));

	AXES[0] = "Fx";
	AXES[1] = "Fy";
	AXES[2] = "Fz";
	AXES[3] = "Tx";
	AXES[4] = "Ty";
	AXES[5] = "Tz";			/* The names of the force and torque axes. */

	*(unsigned short*)&request[0] = htons(0x1234); // standard header. 
	*(unsigned short*)&request[2] = htons(COMMAND); // per table 9.1 in Net F/T user manual. 
	*(unsigned int*)&request[4] = htonl(NUM_SAMPLES); // see section 9.1 in Net F/T user manual. 		

	/* Use of happyhttp library */

	fichier=fopen("/home/kuka/src/groovy_workspace/orocos/ATISensor/xmlget.xml","w");
	happyhttp::Connection conn( "192.168.1.1", 80 );
	conn.setcallbacks(Begin, Data, Complete, 0 );
	conn.request( "GET", "/netftapi2.xml?index=0", 0, 0,0 );
	while( conn.outstanding() )
		conn.pump();
	fclose(fichier);

	/* Use of Tinyxml library */

	char docName[]="/home/kuka/src/groovy_workspace/orocos/ATISensor/xmlget.xml";
        TiXmlDocument doc(docName);
	std::cout << "LOAD " << docName << std::endl;
        if(!doc.LoadFile()) return 0;
        TiXmlHandle docHandle(&doc );				
	
	TiXmlElement* child = docHandle.FirstChild("netft").FirstChild("cfgcpf").ToElement();
	if(!child) return 0;
	resp_cfgcpf = child->GetText();
	cfgcpf=atoi(resp_cfgcpf);

	child = docHandle.FirstChild("netft").FirstChild("cfgcpt").ToElement();
	if(!child) return 0;
	resp_cfgcpt = child->GetText();
	cfgcpt=atoi(resp_cfgcpt);

	//Plutôt utiliser un port d'entrée pour les valeurs de calibrations si celles-ci sont changées en dynamique
	
	std::cout << "cfgcpf = " << cfgcpf <<std::endl;
 	std::cout << "cfgcpt = " << cfgcpt <<std::endl;

	/* Calculate number of samples, command code, and open socket here. */
	
	socketHandle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (socketHandle == -1) {
		std::cout << "Socket could not be opened" <<std::endl;
		exit(1);
	}
	addr.sin_port = htons(PORT);
	err = connect( socketHandle, (struct sockaddr *)&addr, sizeof(addr) );
	if (err == -1) {
		std::cout << "connection failed" <<std::endl;
		exit(2);
	}
	
	free(fichier);
	
  std::cout << "ATISensor configured !" <<std::endl;
  return true;
}

bool ATISensor::startHook(){

  /* Start request to get sensor data */

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
	/*std::cout << "Fx = " <<resp.FTData[0]<<std::endl;
	std::cout << "Fy = " <<resp.FTData[1]<<std::endl;
	std::cout << "Fz = " <<resp.FTData[2]<<std::endl;
	std::cout << "Tx = " <<resp.FTData[3]<<std::endl;
	std::cout << "Ty = " <<resp.FTData[4]<<std::endl;
	std::cout << "Tz = " <<resp.FTData[5]<<std::endl;

  std::cout << "ATISensor executes updateHook !" <<std::endl;*/
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

void Begin( const happyhttp::Response* r, void* userdata )
{
	//printf( "BEGIN (%d %s)\n", r->getstatus(), r->getreason() );
	compteur = 0;
}

void Data( const happyhttp::Response* r, void* userdata, const unsigned char* data, int n )
{
	fwrite( data,1,n, fichier );
	compteur += n;
}

void Complete( const happyhttp::Response* r, void* userdata )
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

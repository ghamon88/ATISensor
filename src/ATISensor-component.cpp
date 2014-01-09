#include "ATISensor-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>


ATISensor::ATISensor(std::string const& name) : TaskContext(name){
  this->addPort("FTData", oport_FTData);
  std::cout << "ATISensor constructed !" <<std::endl;
}

bool ATISensor::configureHook(){

	struct sockaddr_in addr;	/* Address of Net F/T. */
	struct hostent *he;		/* Host entry for Net F/T. */
	int err;			/* Error status of operations. */
	char ip[4];
	char requestConf[]="GET /netftapi2.xml?cfgcpf HTTP/1.1\r\nHOST:192.168.1.1\r\n\r\n"; //diviseurs
	char requestConf2[]="GET /netftapi2.xml?cfgcpt HTTP/1.1\r\nHOST:192.168.1.1\r\n\r\n";
	char* responseConf;

	ip[0]=char(192);
	ip[1]=char(168);
	ip[2]=char(1);
	ip[3]=char(100);

	AXES[0] = "Fx";
	AXES[1] = "Fy";
	AXES[2] = "Fz";
	AXES[3] = "Tx";
	AXES[4] = "Ty";
	AXES[5] = "Tz";/* The names of the force and torque axes. */


	/* Calculate number of samples, command code, and open socket here. */
	socketHandle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (socketHandle == -1) {
		std::cout << "Socket could not be opened" <<std::endl;
		exit(1);
	}
	
	*(unsigned short*)&request[0] = htons(0x1234); /* standard header. */
	*(unsigned short*)&request[2] = htons(COMMAND); /* per table 9.1 in Net F/T user manual. */
	*(unsigned int*)&request[4] = htonl(NUM_SAMPLES); /* see section 9.1 in Net F/T user manual. */
	
	/* Sending the request. */
	//he = gethostbyname("192.168.1.100");
	he->h_length=4;
	he->h_addr_list[0]=ip;
	memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(PORT);
	
	err = connect( socketHandle, (struct sockaddr *)&addr, sizeof(addr) );
	if (err == -1) {
		exit(2);
	}

	send( socketHandle, (const char *)requestConf, strlen(requestConf), 0 ); 
	recv( socketHandle, (char *)responseConf, 36, 0 ); // 36?...
	cfgcpf=atoi(responseConf);

	send( socketHandle, (const char *)requestConf2, strlen(requestConf2), 0 ); 
	recv( socketHandle, (char *)responseConf, 36, 0 );
	cfgcpt=atoi(responseConf);

	//Plutôt utiliser un port d'entrée pour les valeurs de calibrations si celles-ci sont changées en dynamique

  std::cout << "ATISensor configured !" <<std::endl;
  return true;
}

bool ATISensor::startHook(){
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
	
	/* Output the response data
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

	oport_FTData.write(resp.FTData);
  std::cout << "ATISensor executes updateHook !" <<std::endl;
}

void ATISensor::stopHook() {
  close(socketHandle);
  std::cout << "ATISensor executes stopping !" <<std::endl;
}

void ATISensor::cleanupHook() {
  std::cout << "ATISensor cleaning up !" <<std::endl;
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

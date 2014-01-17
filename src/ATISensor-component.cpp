#include "ATISensor-component.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <fcntl.h>
using namespace std;

int recv_timeout(int s, int timeout);

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
	char requestConf[]="GET /netftapi2.xml?index=0 HTTP/1.1\r\nHOST:192.168.1.1\r\n\r\n"; //diviseurs
	char responseConf[1000];
	std::string resp_cfgcpf;
	char XmlString[5000];
	memset(XmlString, 0,5000);

	int i=0;	

	AXES[0] = "Fx";
	AXES[1] = "Fy";
	AXES[2] = "Fz";
	AXES[3] = "Tx";
	AXES[4] = "Ty";
	AXES[5] = "Tz";								/* The names of the force and torque axes. */


	/* Calculate number of samples, command code, and open socket here. */
	//socketHandle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	
	socketHandle = socket(AF_INET, SOCK_STREAM, 0);
	if (socketHandle == -1) {
		std::cout << "Socket could not be opened" <<std::endl;
		exit(1);
	}
	

	*(unsigned short*)&request[0] = htons(0x1234); // standard header. 
	*(unsigned short*)&request[2] = htons(COMMAND); // per table 9.1 in Net F/T user manual. 
	*(unsigned int*)&request[4] = htonl(NUM_SAMPLES); // see section 9.1 in Net F/T user manual. 			

	
	/* Sending the request. */
	std::cout << "avant length" <<std::endl;
	addr.sin_addr.s_addr=inet_addr("192.168.1.1");
	addr.sin_family = AF_INET;
	addr.sin_port = htons(80);

	std::cout << addr.sin_port <<std::endl;

	std::cout << "avant connect" <<std::endl;

	err = connect( socketHandle, (struct sockaddr *)&addr, sizeof(addr) );
	if (err == -1) {
		exit(2);
	}							
	
	std::cout << (const char *)requestConf <<std::endl;
	int retour=send( socketHandle, (const char *)requestConf, strlen(requestConf), 0 );
	std::cout<< "retour = "<< retour << std::endl; 

	//int total_recv = recv_timeout(socketHandle, 2);
	for (int i=1; i<6;i++){
	memset(responseConf, 0,1000);
	recv( socketHandle, (char *)responseConf, 1000, 0 ); 
	strcat(XmlString, responseConf);
	}

	std::cout << "XmlString = " <<(const char *)XmlString <<std::endl;	

/*	TiXmlDocument doc;
	std::cout << "avant parse" <<std::endl;
	doc.Parse((const char*)XmlString, 0, TIXML_ENCODING_UTF8);
	doc.Print();
	std::cout << "avant docHandle" <<std::endl;
	TiXmlHandle docHandle(&doc);
	std::cout << "avant firstchild" <<std::endl;			*/		
	
	/* cfgcpf */
//	TiXmlElement* child = docHandle.FirstChild("netft").FirstChild("cfgcpf").ToElement();
	//if(!child) return 0;
//	std::cout << "avant attribute" <<std::endl;
	//resp_cfgcpf = child->GetText();
	//std::cout << resp_cfgcpf <<std::endl;
	//cfgcpf=atoi(responseConf);
	
	/* cfgcpt */
	/*TiXmlElement* child = docHandle.FirstChild("netft").FirstChild("cfgcpt").ToElement();
	//if(!child) return 0;
	std::cout << "avant attribute" <<std::endl;
	resp_cfgcpt = child->GetText();
	std::cout << resp_cfgcpt <<std::endl;
	//cfgcpt=atoi(resp_cfgcpt);*/

	//Plutôt utiliser un port d'entrée pour les valeurs de calibrations si celles-ci sont changées en dynamique
  std::cout << "cfgcpf = " << cfgcpf <<std::endl;
  std::cout << "cfgcpt = " << cfgcpt <<std::endl;
	
	close(socketHandle);
	shutdown(socketHandle,2);
	socketHandle = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (socketHandle == -1) {
		std::cout << "Socket could not be opened" <<std::endl;
		exit(1);
	}
	addr.sin_port = htons(PORT);
	err = connect( socketHandle, (struct sockaddr *)&addr, sizeof(addr) );
	if (err == -1) {
		exit(2);
	}
	
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
	
	/* Output the response data */
	//printf( "Status: 0x%08x\n", resp.status );
	/*for (i =0;i < 6;i++) {
		printf("%s: %d\n", AXES[i], resp.FTData[i]);
	}*/
	cfgcpf=1;
	cfgcpt=1;
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
  close(socketHandle);
  std::cout << "ATISensor executes stopping !" <<std::endl;
}

void ATISensor::cleanupHook() {
  shutdown(socketHandle,2);
  std::cout << "shutdown ok" <<std::endl;
  std::cout << "ATISensor cleaning up !" <<std::endl;
}

int recv_timeout(int s , int timeout)
{
    int size_recv , total_size= 0;
    struct timeval begin , now;
    char chunk[1000];
    double timediff;
    char XmlString[5000];
	
    strcpy(XmlString,"\O");
     
    //make socket non blocking
    fcntl(s, F_SETFL, O_NONBLOCK);
     
    //beginning time
    gettimeofday(&begin , NULL);
     
    while(1)
    {
	memset(chunk ,0 , 1000);  //clear the variable
        gettimeofday(&now , NULL);
         
        //time elapsed in seconds
        timediff = (now.tv_sec - begin.tv_sec) + 1e-6 * (now.tv_usec - begin.tv_usec);
         
        //if you got some data, then break after timeout
        if( total_size > 0 && timediff > timeout )
        {
            break;
        }
         
        //if you got no data at all, wait a little longer, twice the timeout
        else if( timediff > timeout*2)
        {
            break;
        }
         
        
        if((size_recv =  recv(s , chunk , 1000 , 0) ) < 0)
        {
            //if nothing was received then we want to wait a little before trying again, 0.1 seconds
            usleep(500000);
        }
        else
        {
		std::cout<< "chunk = "<< chunk <<std::endl;
	   // c'est là qu'on change --> concatener ce qu'il y a dans chunk avec une string (5000) , remplacer chunk par des \O
            total_size += size_recv;
	    strcat(XmlString,chunk);
            //printf("%s" , chunk);
            //reset beginning time
            gettimeofday(&begin , NULL);
        }
    }
    std::cout<< "XmlString = "<< XmlString <<std::endl;
    return total_size;
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

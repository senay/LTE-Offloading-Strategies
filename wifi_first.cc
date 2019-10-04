/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include <ns3/buildings-module.h>
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store.h"
#include "ns3/netanim-module.h"
#include "ns3/wifi-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/propagation-module.h"
//#include <ns3/buildings-module.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include <cfloat>
//#include "ns3/gtk-config-store.h"

NS_LOG_COMPONENT_DEFINE("MyLteSimulator");

using namespace ns3;
using namespace std;

// creat an application which generate traffic for Ue 

class MyApp: public Application {
public:

	MyApp();
	virtual ~MyApp();

	void Setup(Ptr<Socket> socket, Address address, uint32_t packetSize,
			uint32_t nPackets, DataRate dataRate, bool qos, bool active);

	virtual void StartApplication(void);
	virtual void StopApplication(void);

	void PauseApplication(void);
	void ResumeApplication(void);

	Address GetRemoteAddress(void);
	bool GetQos(void);
	bool IsActive(void);

private:
	void ScheduleTx(void);
	void SendPacket(void);

	Ptr<Socket> m_socket;
	Address m_peer;
	uint32_t m_packetSize;
	uint32_t m_nPackets;
	DataRate m_dataRate;
	EventId m_sendEvent;
	bool m_running;
	uint32_t m_packetsSent;
	bool m_active;
	bool m_qos;
};

MyApp::MyApp() :
		m_socket(0), m_peer(), m_packetSize(0), m_nPackets(0), m_dataRate(0), m_sendEvent(), m_running(
				false), m_packetsSent(0), m_qos(false), m_active(false) {
}

MyApp::~MyApp() {
	m_socket = 0;
}

void MyApp::Setup(Ptr<Socket> socket, Address address, uint32_t packetSize,
		uint32_t nPackets, DataRate dataRate, bool qos, bool active) {
	m_socket = socket;
	m_peer = address;
	m_packetSize = packetSize;
	m_nPackets = nPackets;
	m_dataRate = dataRate;
	m_qos = qos;
	m_active = active;
}
// check the activity of connection
void MyApp::StartApplication(void) {
	m_running = true;
	m_packetsSent = 0;
	m_socket->Bind();
	m_socket->Connect(m_peer);
	if (m_active)
		SendPacket();
}
// stop apllication (stop sending and close socket)
void MyApp::StopApplication(void) {
	m_running = false;

	if (m_sendEvent.IsRunning()) {
		Simulator::Cancel(m_sendEvent);
	}

	if (m_socket) {
		m_socket->Close();
		m_socket = NULL;
	}
}

void MyApp::ResumeApplication(void) {
	m_running = true;
	SendPacket();
}
// puase applicatin(stop sending and remain connection

void MyApp::PauseApplication(void) {
	m_running = false;
	if (m_sendEvent.IsRunning()) {
		Simulator::Cancel(m_sendEvent);
	}
}

void MyApp::SendPacket(void) {
	Ptr<Packet> packet = Create<Packet>(m_packetSize);
	if (m_qos) {
		QosTag qosTag;
		uint8_t tid = AC_VI;
		qosTag.SetTid(tid);
		packet->AddPacketTag(qosTag);
	}
	m_socket->Send(packet);

	if (m_nPackets == 0) {
		ScheduleTx();
	} else if (++m_packetsSent < m_nPackets) {
		ScheduleTx();
	}
}

void MyApp::ScheduleTx(void) {
	if (m_running) {
		Time tNext(
				Seconds(
						m_packetSize * 8
								/ static_cast<double>(m_dataRate.GetBitRate())));
		m_sendEvent = Simulator::Schedule(tNext, &MyApp::SendPacket, this);
	}
}

Address MyApp::GetRemoteAddress(void) {
	return m_peer;
}

bool MyApp::GetQos(void) {
	return m_qos;
}

bool MyApp::IsActive(void) {
	return m_running;
}
// create the nodecontainer for wifi and lte

NodeContainer enbNodes;
NodeContainer ueNodes;
NodeContainer remoteHostContainer; // for generating traffic for both tecnology using udp application
NodeContainer wifiApNodes;
NodeContainer wifiPgwContainer;

Ptr<Node> remoteHost;
Ptr<Node> pgw;
Ptr<Node> wifiPgw;

Ptr<LteHelper> lteHelper;
Ptr<PointToPointEpcHelper> epcHelper;

Ipv4StaticRoutingHelper ipv4RoutingHelper;

ApplicationContainer serverApps;
ApplicationContainer clientApps;
uint16_t dlPort = 1234;

//ofstream netStatsOut;
ofstream ueThroughputOut;
//ofstream ueThroughputTotalOut;
ofstream completionTime;
ofstream logFile;

uint32_t packetSize; //in bytes
DataRate rate;

//used to switch to LTE when throughput Wifi is under this limit,
//calculated in a temporal window
double sogliaWifi;
double timeWindow;
int    initial = 0;
double  TX_POWER_VALUE = 43;
int numUes;
uint32_t numberOfEnbs;

//struct that contains infos about UE
struct Info {
	bool lte;
	bool qos;
	bool started;
	double startTime;
	double firstRxTime;
	double timeCoverageWifi;
	double timeStartCoverageWifi;
	double timeStartCoverageLte;
	uint32_t rxWifi;
	uint32_t rxLte;
        uint32_t rxTot;
	uint32_t rxTempWifi;
        int      whichapp;
        double finishedat;
        double fileTxcomplete;
        uint32_t totTraffic;
        int stop;
	bool check;
	EventId checkEvent;
};

struct xValue{
  double pthroughput;
  double cthroughput;
  int numPoAues;
  int tile;
  float previous;
  float current;
  int lte;
  int orientation;
};

struct pcValue{
  float previous;
  float current;
};
//struct candenb{
//uint32_t enb;
//double rsrp;
//};

struct candENB{
  uint32_t enb1;
  double rsrp1;
  uint32_t enb2;
  double rsrp2;
  uint32_t enb3;
  double rsrp3;
};


multimap<uint32_t, xValue> PoAxValueMap;//PoAInfo, xValues
map<uint32_t, candENB> ueEnbpowerMap;
multimap<uint32_t, uint32_t> PoAUeMap;//ue , enb/ap
multimap<uint32_t, uint32_t>PoAtile; //tile, PoA



map<uint32_t, Info> ueMap;
map<uint32_t, Ipv4Address> wifiCoverage; //
map<Mac48Address, uint32_t> wifiApMacMap; //
map<uint32_t, uint32_t> lteCoverage; // ueId, enbId
map<uint32_t, uint32_t> lteUeImsi; // IMSI, ueId

uint32_t wifiUes = 0;
uint32_t lteUes = 0;
uint32_t qosUes = 0;
uint32_t started = 0;
uint32_t wifiAssoc = 0;
uint32_t apchanges = 0;
uint32_t enbchanges = 0;
uint32_t lteAssoc = 0;
uint32_t deassociatons = 0;
uint32_t reassociatons = 0;
uint32_t switched = 0;
list<uint32_t> ueDeassociated;

uint32_t MacTxDropCount = 0, MacRxDropCount = 0, PhyTxDropCount = 0,
		PhyRxDropCount = 0;

//to enable more output on log file
bool verbose;

void PrintDrop() {
	logFile << "MacTxDropCount: " << MacTxDropCount << "\t"
			<< "MacRxDropCount: " << MacRxDropCount << "\t"
			<< "PhyTxDropCount: " << PhyTxDropCount << "\t"
			<< "PhyRxDropCount: " << PhyRxDropCount << "\n";
}

void MacTxDrop(Ptr<const Packet> p) {
	NS_LOG_INFO("Packet Drop");
	MacTxDropCount++;
}

void MacRxDrop(Ptr<const Packet> p) {
	NS_LOG_INFO("Packet Drop");
	MacRxDropCount++;
}

void PhyTxDrop(Ptr<const Packet> p) {
	NS_LOG_INFO("Packet Drop");
	PhyTxDropCount++;
}
void PhyRxDrop(Ptr<const Packet> p) {
	NS_LOG_INFO("Packet Drop");
	PhyRxDropCount++;
}

//if enabled through Config::Connect this function is called when a packet in Wifi queue
// of an AP has been retransmitted the maximum number of times (default 7) and so is discarded
void MacTxFailed(string context, Mac48Address maddr) {
	vector<string> tokens;
	istringstream iss(context);
	string token;
	while (getline(iss, token, '/')) {
		tokens.push_back(token);
	}
	uint32_t nodeId = atoi(tokens[2].c_str());
	logFile << "MacTxFailed: Node" << nodeId << " to MAC: " << maddr
			<< " at time=" << Simulator::Now().GetSeconds() << endl;
	for (auto it = wifiApNodes.Begin(); it != wifiApNodes.End(); it++) {
		Ptr<Node> apNode = *it;
		if (apNode->GetId() == nodeId) {
			Ptr<NetDevice> dev = apNode->GetDevice(2);
//			manager->RecordDisassociated(maddr);
			Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(dev);
			Ptr<WifiRemoteStationManager> manager =
					wifiDev->GetRemoteStationManager();
			manager->RecordDisassociated(maddr);
		}
	}
}

void PrintGnuplottableUeListToFile(string filename) {
	ofstream outFile;
	outFile.open(filename.c_str(), ios_base::out | ios_base::trunc);
	if (!outFile.is_open()) {
		NS_LOG_ERROR ("Can't open file " << filename);
		return;
	}
	for (NodeList::Iterator it = NodeList::Begin(); it != NodeList::End();
			++it) {
		Ptr<Node> node = *it;
		int nDevs = node->GetNDevices();
		for (int j = 0; j < nDevs; j++) {
			Ptr<LteUeNetDevice> uedev = node->GetDevice(j)->GetObject<
					LteUeNetDevice>();
			if (uedev) {
				Vector pos = node->GetObject<MobilityModel>()->GetPosition();
				outFile << "set label \"" << "UE " << "\" at " << pos.x << ","
						<< pos.y
						<< " left font \"Helvetica,10\" textcolor rgb \"black\" front point pt 5 ps 0.3 lc rgb \"black\" offset 0,0"
						<< endl;
			}
		}
	}
}

void PrintGnuplottableEnbListToFile(string filename) {
	ofstream outFile;
	outFile.open(filename.c_str(), ios_base::out | ios_base::trunc);
	if (!outFile.is_open()) {
		NS_LOG_ERROR ("Can't open file " << filename);
		return;
	}
	for (NodeList::Iterator it = NodeList::Begin(); it != NodeList::End();
			++it) {
		Ptr<Node> node = *it;
		int nDevs = node->GetNDevices();
		for (int j = 0; j < nDevs; j++) {
			Ptr<LteEnbNetDevice> enbdev = node->GetDevice(j)->GetObject<
					LteEnbNetDevice>();
			if (enbdev) {
				Vector pos = node->GetObject<MobilityModel>()->GetPosition();
				outFile << "set label \"" << "eNB" << "\" at " << pos.x << ","
						<< pos.y
						<< " left font \"Helvetica,16\" textcolor rgb \"green\" front  point pt 5 ps 0.5 lc rgb \"green\" offset 0,0"
						<< endl;
			}
		}
	}
}

void PrintGnuplottableWifiApListToFile(string filename) {
	ofstream outFile;
	outFile.open(filename.c_str(), ios_base::out | ios_base::trunc);
	if (!outFile.is_open()) {
		NS_LOG_ERROR ("Can't open file " << filename);
		return;
	}
	for (uint32_t i = 0; i < wifiApNodes.GetN(); i++) {
		Ptr<Node> node = wifiApNodes.Get(i);
		Vector pos = node->GetObject<MobilityModel>()->GetPosition();
		outFile << "set label \"" << "AP " << "\" at " << pos.x << "," << pos.y
				<< " left font \"Helvetica,14\" textcolor rgb \"red\" front  point pt 5 ps 0.5 lc rgb \"red\" offset 0,0"
				<< endl;
	}
}

static bool isCoveredWifi(Ptr<Node> ue) {
	auto iter = wifiCoverage.find(ue->GetId());
	if (iter != wifiCoverage.end()) {
		return true;
	} else
		return false;
}

static bool isCoveredLte(Ptr<Node> ue) {
	auto iter = lteCoverage.find(ue->GetId());
	if (iter != lteCoverage.end()) {
		return true;
	} else
		return false;
}

int ChangeToWifi(Ptr<Node> ue) {
//	logFile << "Change to wifi :" << endl;
	Ptr<Ipv4> ipv4;
	Ipv4Address addr;
	Ptr<Application> app;
	Ptr<MyApp> clientWifi;
	Ptr<MyApp> clientLte;

	bool found = false;

	if (isCoveredWifi(ue)) {

		ipv4 = ue->GetObject<Ipv4>();
		addr = ipv4->GetAddress(2, 0).GetLocal(); //interface 2 is lte address
		Ptr<Node> remoteHost = remoteHostContainer.Get(0);

		for (uint32_t i = 0; i < remoteHost->GetNApplications() && !found;
				i++) {
			app = remoteHost->GetApplication(i);
			Address remote;
			clientLte = app->GetObject<MyApp>();
			remote = clientLte->GetRemoteAddress();
			InetSocketAddress inet = InetSocketAddress::ConvertFrom(remote);
			Ipv4Address ip = inet.GetIpv4();
			if (addr.IsEqual(ip)) {
//				logFile << "App to " << ip << " changing to wifi" << endl;
				found = true;
			}
		}

		found = false;
		addr = ipv4->GetAddress(1, 0).GetLocal(); //interface 1 is wifi address
		for (uint32_t i = 0; i < remoteHost->GetNApplications() && !found;
				i++) {
			app = remoteHost->GetApplication(i);
			Address remote;
			clientWifi = app->GetObject<MyApp>();
			remote = clientWifi->GetRemoteAddress();
			InetSocketAddress inet = InetSocketAddress::ConvertFrom(remote);
			Ipv4Address ip = inet.GetIpv4();
			if (addr.IsEqual(ip)) {
//				logFile << "App to " << ip << " enabling" << endl;
				found = true;
			}
		}

		if (clientLte->IsActive()) {
			clientLte->PauseApplication();
		}
		clientWifi->ResumeApplication();
                return 1;

	} else {
		logFile << "Unable to change to wifi: node is not covered" << endl;
                return 0;
	}
}



static void ChangeToLte(Ptr<Node> ue) {
//	logFile << "Change to lte :" << endl;
	Ptr<Ipv4> ipv4;
	Ipv4Address addr;
	Ptr<Application> app;
	Ptr<MyApp> clientWifi;
	Ptr<MyApp> clientLte;
	bool found = false;
	ipv4 = ue->GetObject<Ipv4>();
	addr = ipv4->GetAddress(1, 0).GetLocal(); //interface 1 is wifi address
	Ptr<Node> remoteHost = remoteHostContainer.Get(0);

	for (uint32_t i = 0; i < remoteHost->GetNApplications() && !found; i++) {
		app = remoteHost->GetApplication(i);
		Address remote;
		clientWifi = app->GetObject<MyApp>();
		remote = clientWifi->GetRemoteAddress();
		InetSocketAddress inet = InetSocketAddress::ConvertFrom(remote);
		Ipv4Address ip = inet.GetIpv4();
		if (addr.IsEqual(ip)) {
			//				logFile << "App to " << ip << " changing to lte" << endl;
			found = true;
		}
	}
	if (clientWifi->IsActive()) {
		clientWifi->PauseApplication();
	}

	if (isCoveredLte(ue)) {

		found = false;
		addr = ipv4->GetAddress(2, 0).GetLocal(); //interface 2 is lte address
		for (uint32_t i = 0; i < remoteHost->GetNApplications() && !found;
				i++) {
			app = remoteHost->GetApplication(i);
			Address remote;
			clientLte = app->GetObject<MyApp>();
			remote = clientLte->GetRemoteAddress();
			InetSocketAddress inet = InetSocketAddress::ConvertFrom(remote);
			Ipv4Address ip = inet.GetIpv4();
			if (addr.IsEqual(ip)) {
//				logFile << "App to " << ip << " enabling" << endl;
				found = true;
			}
		}

		clientLte->ResumeApplication();

	} else {
		logFile << "Unable to change to lte: node is not covered" << endl;
	}
}

void startAgain(Ptr<Node> ue) {//to request for additional GR traffic
//	logFile << "CheckRate at " << Simulator::Now().GetSeconds() << endl;
	auto ueInfo = ueMap.find(ue->GetId());
	if (Simulator::Now().GetSeconds()- ueInfo->second.finishedat >= 0.2) {
                ueInfo->second.stop=0;
	        Ptr<Ipv4> ipv4;
	        Ipv4Address addr;
                Ipv4Address addr1;
	        Ptr<Application> app;
	        Ptr<MyApp> clientWifi;
	        Ptr<MyApp> clientLte;
	        bool found = false;
                ipv4 = ue->GetObject<Ipv4>();
                if(ueInfo->second.whichapp==0){
	                addr = ipv4->GetAddress(1, 0).GetLocal(); //interface 1 is wifi address
	                Ptr<Node> remoteHost = remoteHostContainer.Get(0);
                        for (uint32_t i = 0; i < remoteHost->GetNApplications() && !found; i++) {
		                app = remoteHost->GetApplication(i);
		                Address remote;
		                clientWifi = app->GetObject<MyApp>();
		                remote = clientWifi->GetRemoteAddress();
		                InetSocketAddress inet = InetSocketAddress::ConvertFrom(remote);
		                Ipv4Address ip = inet.GetIpv4();
		                        if (addr.IsEqual(ip)) {
			                        //				logFile << "App to " << ip << " changing to lte" << endl;
			                        found = true;
		                        }
	                }
                        
	                if ((!clientWifi->IsActive())&&found) {
		                clientWifi->ResumeApplication();
				//logFile << "startAgainWifi" << endl; 
                                
	                }
                }
                if(ueInfo->second.whichapp==1){
                        addr1 = ipv4->GetAddress(2, 0).GetLocal(); //interface 2 is lte address
		        Ptr<Node> remoteHost1 = remoteHostContainer.Get(0);

		        for (uint32_t i = 0; i < remoteHost1->GetNApplications() && !found;
				        i++) {
			        app = remoteHost->GetApplication(i);
			        Address remote;
			        clientLte = app->GetObject<MyApp>();
			        remote = clientLte->GetRemoteAddress();
			        InetSocketAddress inet = InetSocketAddress::ConvertFrom(remote);
			        Ipv4Address ip = inet.GetIpv4();
			        if (addr1.IsEqual(ip)) {
        //				logFile << "App to " << ip << " changing to wifi" << endl;
				        found = true;
			        }
		        }
                         if ((!clientLte->IsActive())&&found) {
		                clientLte->ResumeApplication();
				//logFile << "startAgainLte" << endl; 
	                }	
                } 
		      
        }
        else
                ueInfo->second.checkEvent = Simulator::Schedule(Seconds(0.2),
					&startAgain, ue);
}

static void finished(Ptr<Node> ue){
        auto ueInfo = ueMap.find(ue->GetId());
        if(ueInfo->second.qos&&(ueInfo->second.rxTot>=524288/8)){

          
		ueInfo->second.rxTot = 0;
                Ptr<Ipv4> ipv4;
	        Ipv4Address addr;
                Ipv4Address addr1;
	        Ptr<Application> app;
	        Ptr<MyApp> clientWifi;
	        Ptr<MyApp> clientLte;
	        bool found = false;
	        ipv4 = ue->GetObject<Ipv4>();
	        addr = ipv4->GetAddress(1, 0).GetLocal(); //interface 1 is wifi address
	        Ptr<Node> remoteHost = remoteHostContainer.Get(0);
                
                for (uint32_t i = 0; i < remoteHost->GetNApplications() && !found; i++) {
		app = remoteHost->GetApplication(i);
		Address remote;
		clientWifi = app->GetObject<MyApp>();
		remote = clientWifi->GetRemoteAddress();
		InetSocketAddress inet = InetSocketAddress::ConvertFrom(remote);
		Ipv4Address ip = inet.GetIpv4();
		        if (addr.IsEqual(ip)) {
			        //				logFile << "App to " << ip << " changing to lte" << endl;
			        found = true;
		        }
	        }
                
	        if (clientWifi->IsActive()&&found&&(ueInfo->second.rxWifi>81920/8)) {
		        clientWifi->PauseApplication();
                        ueInfo->second.whichapp = 0;
                        //logFile << "finishedWifi" << endl;
                        ueInfo->second.finishedat = Simulator::Now().GetSeconds();
                        ueInfo->second.stop = 1;

	        }
                found = false;
                addr1 = ipv4->GetAddress(2, 0).GetLocal(); //interface 2 is lte address
		Ptr<Node> remoteHost1 = remoteHostContainer.Get(0);

		for (uint32_t i = 0; i < remoteHost1->GetNApplications() && !found;
				i++) {
			app = remoteHost->GetApplication(i);
			Address remote;
			clientLte = app->GetObject<MyApp>();
			remote = clientLte->GetRemoteAddress();
			InetSocketAddress inet = InetSocketAddress::ConvertFrom(remote);
			Ipv4Address ip = inet.GetIpv4();
			if (addr1.IsEqual(ip)) {
//				logFile << "App to " << ip << " changing to wifi" << endl;
				found = true;
			}
		}
                 if (clientLte->IsActive()&&found&&(ueInfo->second.rxLte>81920/8)) {
		        clientLte->PauseApplication();
                        ueInfo->second.whichapp = 1;
                        //logFile << "finishedLte" << endl;
                        ueInfo->second.finishedat = Simulator::Now().GetSeconds();
                        ueInfo->second.stop = 1;
	        }
                if(ueInfo->second.totTraffic >= 3145728/8){
                        ueInfo->second.fileTxcomplete = Simulator::Now().GetSeconds();
                        //logFile << "UE completed file download" << endl;
                }
                if(ueInfo->second.totTraffic < 3145728/8)                
                        ueInfo->second.checkEvent = Simulator::Schedule(Seconds(0.2),
					&startAgain, ue);
        }
}


static void SinkRx(string context, Ptr<const Packet> p,
		const Address& addr) {
	//Returns an InetSocketAddress which corresponds to the input Address
	InetSocketAddress iaddr = InetSocketAddress::ConvertFrom(addr);
	Ipv4Address sourceIp = iaddr.GetIpv4();
	//take the Node number of the context
	vector<string> tokens;
	istringstream iss(context);
	string token;
	while (getline(iss, token, '/')) {
		tokens.push_back(token);
	}
        //QosTag qosTag;
	//if(p->PeekPacketTag(qosTag)){
        //uint8_t tid = uint8_t(QosUtilsGetTidForPacket(p));
        //if(tid == 5)
        //std::cout<<"type-1 traffic: "<<tid<<std::endl;
        //}
	//split(tokens, context, '/');
	uint32_t nodeId = atoi(tokens[2].c_str());
	Ptr<Node> ue = ueNodes.Get(nodeId);

	auto ueInfo = ueMap.find(nodeId);
        if (ueInfo->second.firstRxTime == -1)
		        ueInfo->second.firstRxTime = Simulator::Now().GetSeconds();
        if(ueInfo->second.stop == 0){
                ueInfo->second.rxTot += p->GetSize();
                ueInfo->second.totTraffic += p->GetSize();
	        if (sourceIp == Ipv4Address("1.0.0.2")) {
		        ueInfo->second.rxLte += p->GetSize();
                        auto rr = lteCoverage.find(nodeId);
                        auto pp = PoAxValueMap.find(rr->second);
                        pp->second.cthroughput += p->GetSize();
	        } 
                else if (sourceIp == Ipv4Address("2.0.0.2")) {
		        ueInfo->second.rxWifi += p->GetSize();
                        auto xx = PoAUeMap.find(nodeId);
                        auto pp = PoAxValueMap.find(xx->second);
                        pp->second.cthroughput += p->GetSize();
		        if (ueInfo->second.check)
			        ueInfo->second.rxTempWifi += p->GetSize();
	        } else {
		        logFile << "strange packet" << endl;
	        }
                if(!ueInfo->second.qos){

                        if(ueInfo->second.totTraffic >= 2097152/4){
                                ueInfo->second.fileTxcomplete = Simulator::Now().GetSeconds();
                                logFile << "UE completed file download" << endl;
                        }
                }

        }


        finished(ue);
}

void CheckRate(Ptr<Node> ue) {
//	logFile << "CheckRate at " << Simulator::Now().GetSeconds() << endl;
	auto ueInfo = ueMap.find(ue->GetId());
	if ((ueInfo->second.rxTempWifi * 8) < sogliaWifi) {
//		logFile << "UE " << ue->GetId() << " not enough wifi rate: only "
//				<< ueInfo->second.rxTempWifi * 8 << endl;
		switched++;
//		logFile << "wifi switched: " << switched << endl;
		ueInfo->second.check = false;
		ChangeToLte(ue);
	} else {
		ueInfo->second.rxTempWifi = 0;
		ueInfo->second.checkEvent = Simulator::Schedule(Seconds(timeWindow),
				&CheckRate, ue);
	}
}



void SetupApp(Ptr<Node> ue, Ipv4Address addr, bool qos, bool active) {
	uint32_t nPackets = 0;

	Ptr<Socket> ns3UdpSocket = Socket::CreateSocket(remoteHost,
			UdpSocketFactory::GetTypeId());
	Ptr<MyApp> app = CreateObject<MyApp>();
	app->Setup(ns3UdpSocket, InetSocketAddress(addr, dlPort), packetSize,
			nPackets, rate, qos, active);
	remoteHost->AddApplication(app);
	app->SetStartTime(Seconds(0));
	clientApps.Add(app);
	if (active) {
//		logFile << "sender app to " << addr << endl;
		started++;
	}

}

void NotifyConnectionEstablishedUe(string context, uint64_t imsi,
		uint16_t cellid, uint16_t rnti) {
	vector<string> tokens;
	istringstream iss(context);
	string token;
	while (getline(iss, token, '/')) {
		tokens.push_back(token);
	}
	uint32_t nodeId = atoi(tokens[2].c_str());
	Ptr<Node> ue = ueNodes.Get(nodeId);

	Ptr<Ipv4> ipv4;
	ipv4 = ue->GetObject<Ipv4>();
	Ipv4Address ipUe = ipv4->GetAddress(2, 0).GetLocal();
	pair<uint32_t, uint32_t> elem = { imsi, nodeId };
	lteUeImsi.insert(elem);
//	logFile << "Node " << nodeId << " (" << ipUe << ") UE IMSI " << imsi
//			<< ": connected to CellId " << cellid << " with RNTI " << rnti
//			<< " at time=" << Simulator::Now().GetSeconds() << endl;
}

void NotifyConnectionEstablishedEnb(string context, uint64_t imsi,
		uint16_t cellid, uint16_t rnti) {
	vector<string> tokens;
	istringstream iss(context);
	string token;
	while (getline(iss, token, '/')) {
		tokens.push_back(token);
	}
	uint32_t nodeId = atoi(tokens[2].c_str());

//	logFile << "Node " << nodeId << " eNB CellId " << cellid
//			<< ": successful connection of UE with IMSI " << imsi << " RNTI "
//			<< rnti << " at time=" << Simulator::Now().GetSeconds()
//			<< endl;

	auto iter = lteUeImsi.find(imsi);
	Ptr<Node> ue = ueNodes.Get(iter->second);
	auto ueInfo = ueMap.find(ue->GetId());

	ueInfo->second.timeStartCoverageLte = Simulator::Now().GetSeconds();

	pair<uint32_t, uint32_t> elem = { iter->second, nodeId };
	lteCoverage.insert(elem);
	lteAssoc++;
//	logFile << "lte associated: " << lteAssoc << endl;

	Ptr<Ipv4> ipv4;
	ipv4 = ue->GetObject<Ipv4>();
	Ipv4Address ipUeLte = ipv4->GetAddress(2, 0).GetLocal(); //interface 2 is lte address
	Ipv4Address ipUeWifi = ipv4->GetAddress(1, 0).GetLocal(); //interface 1 is wifi address
	bool qos = ueMap.find(ue->GetId())->second.qos;
	if (!ueInfo->second.started) { // new Node
//		logFile << "New Lte Node -> starting app" << endl;
		ueInfo->second.started = true;
		ueMap.find(ue->GetId())->second.startTime =
				Simulator::Now().GetSeconds();
		SetupApp(ue, ipUeLte, qos, true);
		SetupApp(ue, ipUeWifi, qos, false);
                auto kkk = PoAxValueMap.find(nodeId);
                kkk->second.numPoAues += 1;
                pair<uint32_t, uint32_t> elem = {ue->GetId(), nodeId };
                PoAUeMap.insert(elem);
	} else { //no new node: if wifi disconnected change to lte
		Ptr<Application> app;
		Ptr<MyApp> client;
		bool found = false;
		for (uint32_t i = 0; i < remoteHost->GetNApplications() && !found;
				i++) {
			app = remoteHost->GetApplication(i);
			Address remote;
			client = app->GetObject<MyApp>();
			//socket = client->GetObject<Socket>();
			remote = client->GetRemoteAddress();
			InetSocketAddress inet = InetSocketAddress::ConvertFrom(remote);
			Ipv4Address ip = inet.GetIpv4();
			if (ip.IsEqual(ipUeWifi)) {
				found = true;
				if (!client->IsActive()) {
//					logFile << "App to " << ip << " not active -> starting app"
//							<< endl;
					ChangeToLte(ue);
				}
			}
		}

	}
}

//void NotifyHandoverStartUe(string context, uint64_t imsi, uint16_t cellid,
//		uint16_t rnti, uint16_t targetCellId) {
//	logFile << context << " UE IMSI " << imsi
//			<< ": previously connected to CellId " << cellid << " with RNTI "
//			<< rnti << ", doing handover to CellId " << targetCellId
//			<< endl;
//}

void NotifyHandoverEndOkUe(string context, uint64_t imsi, uint16_t cellid,
		uint16_t rnti) {
	logFile << "Handover IMSI " << imsi << endl;

}

//void NotifyHandoverStartEnb(string context, uint64_t imsi, uint16_t cellid,
//		uint16_t rnti, uint16_t targetCellId) {
//	logFile << context << " eNB CellId " << cellid
//			<< ": start handover of UE with IMSI " << imsi << " RNTI " << rnti
//			<< " to CellId " << targetCellId << endl;
//}
//
//void NotifyHandoverEndOkEnb(string context, uint64_t imsi, uint16_t cellid,
//		uint16_t rnti) {
//	logFile << context << " eNB CellId " << cellid
//			<< ": completed handover of UE with IMSI " << imsi << " RNTI "
//			<< rnti << endl;
//}

static
void StaMacAssoc(string context, Mac48Address maddr) {
//	logFile << "Wifi assoc" << endl;
	vector<string> tokens;
	istringstream iss(context);
	string token;
	while (getline(iss, token, '/')) {
		tokens.push_back(token);
	}
	uint32_t nodeId = atoi(tokens[2].c_str());
	Ptr<Node> ue;
	Ipv4Address ipAp;
	uint32_t apId = -1;
	auto it = wifiApMacMap.find(maddr);
        uint32_t poaId = wifiApNodes.Get(it->second)->GetId();
	if (it != wifiApMacMap.end())
		apId = it->second;
                
	else
		logFile << "Mac Access Point  not found!" << endl;
	if (apId != -1) {
		Ptr<Ipv4> ipv4;
		Ipv4Address addr;
		Ptr<Node> ue = ueNodes.Get(nodeId);
		ipv4 = ue->GetObject<Ipv4>();
		Ipv4Address ipUeWifi = ipv4->GetAddress(1, 0).GetLocal();
		Ipv4Address ipUeLte = ipv4->GetAddress(2, 0).GetLocal(); //interface 2 is lte address
		ipv4 = wifiApNodes.Get(apId)->GetObject<Ipv4>();
		Ipv4Address ipAp = ipv4->GetAddress(2, 0).GetLocal();
		Ipv4Address ipNextHop = ipv4->GetAddress(1, 0).GetLocal();
		pair<uint32_t, Ipv4Address> elem = { nodeId, ipAp };
		wifiCoverage.insert(elem);
		wifiAssoc++;
                //std::cout<<"STA Associated: Node"<<nodeId<<"("<<ipUeWifi<<") to MAC: " <<maddr<<" ("<<ipAp<<") at time="<<Simulator::Now().GetSeconds() <<std::endl;
		if (verbose) {
			logFile << "STA Associated: Node" << nodeId << " (" << ipUeWifi
					<< ") to MAC: " << maddr << " (" << ipAp << ") at time="
					<< Simulator::Now().GetSeconds() << endl;
		}
//		logFile << "wifi associated: " << wifiAssoc << endl;

		Ptr<Ipv4StaticRouting> wifiPgwStaticRouting =
				ipv4RoutingHelper.GetStaticRouting(wifiPgw->GetObject<Ipv4>());
		Ipv4Address nextHopNetwork = ipNextHop.CombineMask(
				Ipv4Mask("255.255.255.252"));
		uint32_t interface;
		for (uint32_t i = 0; i < wifiPgwStaticRouting->GetNRoutes(); i++) {
			if (wifiPgwStaticRouting->GetRoute(i).GetDest() == nextHopNetwork)
				interface = wifiPgwStaticRouting->GetRoute(i).GetInterface();
		}
		wifiPgwStaticRouting->AddHostRouteTo(ipUeWifi, ipNextHop, interface);
		Ptr<Ipv4StaticRouting> ueStaticRouting =
				ipv4RoutingHelper.GetStaticRouting(ue->GetObject<Ipv4>());
		ueStaticRouting->AddNetworkRouteTo(Ipv4Address("2.0.0.0"),
				Ipv4Mask("255.0.0.0"), ipAp, 1);
		auto ueInfo = ueMap.find(ue->GetId());
		ueInfo->second.timeStartCoverageWifi = Simulator::Now().GetSeconds();

		bool found = false;
		for (auto it = ueDeassociated.begin();
				it != ueDeassociated.end() && !found; it++) {
			if (nodeId == *it) {
				reassociatons++;
				ueDeassociated.remove(*it);
				found = true;
               
	}
		}

		if (ueInfo->second.started) {
			//app already started
			ChangeToWifi(ue);

			ueInfo->second.rxTempWifi = 0;
			ueInfo->second.check = true;
			ueInfo->second.checkEvent = Simulator::Schedule(Seconds(timeWindow),
					&CheckRate, ue);

		} else { //new UE connected
//			logFile << "New Wifi Node -> starting app" << endl;
			ueInfo->second.started = true;
			ueMap.find(ue->GetId())->second.startTime =
					Simulator::Now().GetSeconds();
                         
			bool qos = ueMap.find(ue->GetId())->second.qos;
			SetupApp(ue, ipUeLte, qos, false);
			SetupApp(ue, ipUeWifi, qos, true);
                        auto rr = PoAxValueMap.find(poaId);
                        rr->second.numPoAues++;
                        pair<uint32_t, uint32_t> elem = {ue->GetId(), poaId };
                        PoAUeMap.insert(elem);
                        
                        

			ueInfo->second.rxTempWifi = 0;
			ueInfo->second.check = true;
			ueInfo->second.checkEvent = Simulator::Schedule(Seconds(timeWindow),
					&CheckRate, ue);

		}
	} else {
		logFile << "Access Point with mac " << maddr << " not found!"
				<< endl;
	}
}

static
void StaMacDeAssoc(string context, Mac48Address maddr) {
//	logFile << "Wifi de-assoc" << endl;
	vector<string> tokens;
	istringstream iss(context);
	string token;
	while (getline(iss, token, '/')) {
		tokens.push_back(token);
	}
	uint32_t nodeId = atoi(tokens[2].c_str());
	Ipv4Address ipAp;
	int apId = -1;
	Ptr<Node> ue;
	auto it = wifiApMacMap.find(maddr);
	if (it != wifiApMacMap.end())
		apId = it->second;
	if (apId != -1) {
		Ptr<Ipv4> ipv4;
		Ptr<Node> ue = ueNodes.Get(nodeId);
		ipv4 = ue->GetObject<Ipv4>();
		Ipv4Address ipUe = ipv4->GetAddress(1, 0).GetLocal();
		ipv4 = wifiApNodes.Get(apId)->GetObject<Ipv4>();
		Ipv4Address ipAp = ipv4->GetAddress(2, 0).GetLocal();
		auto iter = wifiCoverage.find(nodeId);
		if (iter != wifiCoverage.end()) {
			wifiCoverage.erase(iter);
		}
		if (verbose) {
			logFile << "STA De-Associated: Node" << nodeId << " (" << ipUe
					<< ") to MAC: " << maddr << " (" << ipAp << ") at time="
					<< Simulator::Now().GetSeconds() << endl;
		}
		deassociatons++;
		ueDeassociated.push_back(nodeId);

		Ptr<Ipv4StaticRouting> wifiPgwStaticRouting =
				ipv4RoutingHelper.GetStaticRouting(wifiPgw->GetObject<Ipv4>());
		for (uint32_t i = 0; i < wifiPgwStaticRouting->GetNRoutes(); i++) {
			if (wifiPgwStaticRouting->GetRoute(i).GetDest() == ipUe)
				wifiPgwStaticRouting->RemoveRoute(i);
		}
		Ptr<Ipv4StaticRouting> ueStaticRouting =
				ipv4RoutingHelper.GetStaticRouting(ue->GetObject<Ipv4>());
		for (uint32_t i = 0; i < ueStaticRouting->GetNRoutes(); i++) {
			if (ueStaticRouting->GetRoute(i).GetDest()
					== Ipv4Address("2.0.0.0"))
				ueStaticRouting->RemoveRoute(i);
		}

		auto ueInfo = ueMap.find(ue->GetId());
		ueInfo->second.timeCoverageWifi += (Simulator::Now().GetSeconds()
				- ueInfo->second.timeStartCoverageWifi);
		ueInfo->second.timeStartCoverageWifi = 0;

		if (ueInfo->second.checkEvent.IsRunning())
			Simulator::Cancel(ueInfo->second.checkEvent);
		ueInfo->second.check = false;

		ChangeToLte(ue);
	} else
		logFile << "Access Point with mac " << maddr << " not found!"
				<< endl;
}

void Throughput() // in Mbps calculated every 0.5s
{
	for (auto it = ueMap.begin(); it != ueMap.end(); it++) {
		Ptr<Ipv4> ipv4;
		Ipv4Address ipLte;
		Ipv4Address ipWifi;
		ipv4 = ueNodes.Get(it->first)->GetObject<Ipv4>();
		ipWifi = ipv4->GetAddress(1, 0).GetLocal();
		ipLte = ipv4->GetAddress(2, 0).GetLocal();

		ueThroughputOut << Simulator::Now().GetSeconds() << "\t" << it->first
				<< "\t" << ipWifi << "\t" << ipLte << "\t"
				<< it->second.rxWifi * 8 << "\t" << it->second.rxLte * 8
				<< endl;
                /*auto itr = PoAUeMap.find(it->first);
                auto iter = PoAxValueMap.find(itr->second);
                if(iter->second.lte == 0)
                iter->second.cthroughput += it->second.rxWifi;
                if(iter->second.lte == 0)
                iter->second.cthroughput += it->second.rxLte;*/
		it->second.rxWifi = 0;
		it->second.rxLte = 0;
	}
	Simulator::Schedule(Seconds(0.5), &Throughput); // Callback every 0.5s
        

}

////trace report of user
void 
NotifyRecvMeasurementReportTrace (std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti, LteRrcSap::MeasurementReport msg)
  {
        uint8_t measId = msg.measResults.measId;
        auto iter = lteUeImsi.find(imsi);
        Ptr<Node> ue = ueNodes.Get(iter->second);
        uint32_t nodeId = ue->GetId();
        auto ueInfo = ueEnbpowerMap.find(ue->GetId());
        //ueInfo->second.rsrqser=(uint16_t) msg.measResults.rsrqResult;
        //ueInfo->second.rsrpser=(uint16_t) msg.measResults.rsrpResult;
        int i=1;
        for (std::list <LteRrcSap::MeasResultEutra>::iterator it = msg.measResults.measResultListEutra.begin(); 
        it != msg.measResults.measResultListEutra.end(); ++it)
        {
                if(i==1){
                ueInfo->second.rsrp1=(it->haveRsrpResult ? (uint16_t) it->rsrpResult : 255);
                //ueInfo->second.rsrp1=(it->haveRsrqResult ? (uint16_t) it->rsrqResult : 255);
                ueInfo->second.enb1= it->physCellId-1;
                i++;
                }
                if(i==2){
                ueInfo->second.rsrp2=(it->haveRsrpResult ? (uint16_t) it->rsrpResult : 255);
                //ueInfo->second.rsrp2=(it->haveRsrqResult ? (uint16_t) it->rsrqResult : 255);
                ueInfo->second.enb2= it->physCellId-1;
                i++;
                }
                if(i==3){
                ueInfo->second.rsrp3=(it->haveRsrpResult ? (uint16_t) it->rsrpResult : 255);
                //ueInfo->second.rsrp3=(it->haveRsrqResult ? (uint16_t) it->rsrqResult : 255);
                ueInfo->second.enb3= it->physCellId-1;
                i++;
                }
        }
}
  
//**************end




double
ReceivedPower(Ptr<Node> ue, Ptr<Node> enbNode)
{
 
Ptr<MobilityModel> user = ue->GetObject<MobilityModel>();
Ptr<MobilityModel> enb = enbNode->GetObject<MobilityModel>();
Ptr<FriisPropagationLossModel> propagationLossModel =  CreateObject<FriisPropagationLossModel> ();
double RxPower = propagationLossModel-> CalcRxPower(TX_POWER_VALUE, enb, user);
  //Uncomment for printing on screen the Received Power Value
// std::cout << "User" << ueNode->GetId() << " receives a signal strength of " << RxPower << " dBm from eNB" << enbNode->GetId() <<std::endl;
  return RxPower;
}

static void bubblesortAscend(float array1[],int array2[], int N){
	 //Sorts an array of data, and a second array that represents the cluster/access point number
	int i,j,temp2=0;
	float temp1=0;
		for (i=0;i<N;i++){
			for (j=0;j<N-1;j++){
				if(array1[j]>array1[j+1]){
				temp1 = array1[j+1];
				array1[j+1]=array1[j];
				array1[j]=temp1;
				temp2 = array2[j+1];
				array2[j+1]=array2[j];
				array2[j]=temp2;
			}
		}
	}
}
void xValues(){
        

        float payload = 1024;
        float avgPeruserthroughput;
        int userTile;
        float x, y, gamma1;
        float throughput,tmp,T, cx, cxOpt;  
	if(initial == 0){
		            
		for(uint32_t tile = 1; tile<=7; tile++){
                        avgPeruserthroughput = 0;
                        userTile = 0; 
                        std::pair <std::multimap<uint32_t, uint32_t>::iterator, std::multimap<uint32_t, uint32_t>::iterator> ret;
                        ret = PoAtile.equal_range(tile);
                        for (std::multimap<uint32_t, uint32_t>::iterator itt=ret.first; itt!=ret.second; ++itt){
                                auto it = PoAxValueMap.find(itt->second);
                                int x = it->second.numPoAues;                                
                                if(x>0){
			                avgPeruserthroughput +=it->second.cthroughput/it->second.numPoAues;
                                        userTile += it->second.numPoAues; 
                                }

                        }
                        
                      	for(std::multimap<uint32_t, uint32_t>::iterator itt=ret.first; itt!=ret.second; ++itt){
                                auto it = PoAxValueMap.find(itt->second);
                                x = 0;
                                y = 0;
                                gamma1 = 0.01;
                                throughput = 0,tmp=0,T=0, cx=0, cxOpt=0;
                                x = it->second.numPoAues;
                                y = it->second.cthroughput;
                                while(gamma1<0.1){
                                        
                                                                                           
                                        if(userTile > 0){
                                                if(x > 0)
	                                        cx=x/userTile+gamma1*(y/x)/avgPeruserthroughput;
                                        }
                                        
                                        T = it->second.current*userTile*(packetSize+38/*ACK*/)/y/*rate*/+250/*overhead*/;
                                        tmp = payload*userTile*cx/T;
                                        if(tmp > throughput){
		                                throughput = tmp;
                                                cxOpt = cx; 
                                        } 
                                        gamma1 += 0.02;                                             
                                        
                                }
                                if(userTile>0)
                                it->second.previous = x/userTile;
                                it->second.current = cxOpt; 
                                //std::cout<<"xcurrento"<<it->second.current<<std::endl;
                                //std::cout<<"xcurrentopt"<<cxOpt<<std::endl;
                        }
                }
                initial = 1;
	}

        
        if(initial == 1){
		float cummChange;
		float xChange;
		float tChange,gamma1=0.01,gamma2=0.01;           
		for(uint32_t tile = 1; tile<=7; tile++){
                        avgPeruserthroughput = 0;
                        userTile = 0; 
                        std::pair <std::multimap<uint32_t, uint32_t>::iterator, std::multimap<uint32_t, uint32_t>::iterator> ret;
                        ret = PoAtile.equal_range(tile);
                        for (std::multimap<uint32_t, uint32_t>::iterator itt=ret.first; itt!=ret.second; ++itt){
                                auto it = PoAxValueMap.find(itt->second);
                                int x = it->second.numPoAues;                                
                                if(x>0){
			                avgPeruserthroughput +=it->second.cthroughput/it->second.numPoAues;
                                        userTile += it->second.numPoAues; 
                                }

                        }
                        //std::cout<<"userTile1"<<userTile<<std::endl;
                        //std::cout<<"avgPeruserthroughput"<<avgPeruserthroughput<<std::endl;
                      	for(std::multimap<uint32_t, uint32_t>::iterator itt=ret.first; itt!=ret.second; ++itt){
                                auto it = PoAxValueMap.find(itt->second);
                                x = 0;
                                y = 0;
                                gamma1 = 0.01;
                                throughput = 0,tmp=0,T=0, cx=0, cxOpt=0;
                                x = it->second.numPoAues;
                                y = it->second.cthroughput;
                                cummChange = 0;
                                xChange = it->second.current-it->second.previous; 
                                tChange = it->second.cthroughput-it->second.pthroughput; 
                                if(xChange>0&&tChange>0||xChange<0&&tChange<0)
	                                cummChange = gamma2;
                                if((xChange<0&&tChange>0)||(xChange>0&&tChange<0))
	                                cummChange = -gamma2;
                                if(xChange==0||tChange==0)
	                                cummChange = 0;
                                //std::cout<<"cummChange"<<cummChange<<std::endl;
                                while(gamma1<0.1){
                                        gamma2=0.01;
                        	        while(gamma2<0.1){
                                                
		                                if(it->second.numPoAues>0){
		                                        cx = it->second.current + gamma1*(it->second.cthroughput/it->second.numPoAues)/avgPeruserthroughput+cummChange; 
                                                        //std::cout<<"cx"<<cx<<std::endl;	
                                                }
                                                
                                                T = cx*userTile*(packetSize+38/*ACK*/)/y/*rate*/+250/*overhead*/;
                                                tmp = payload*userTile*cx/T;
                                                if(tmp > throughput){
			                                throughput = tmp;
                                                        cxOpt = cx; 
                                                }     
                                                gamma2 += 0.02;                                          
                                        }
                                        gamma1 += 0.02; 
                                }
                                it->second.previous = it->second.current;
                                it->second.current = cxOpt;
                                it->second.pthroughput = it->second.cthroughput;
                                it->second.cthroughput = 0; 
                                //std::cout<<"xcurrento"<<it->second.current<<std::endl;
                                //std::cout<<"xcurrentopt"<<cxOpt<<std::endl;
                        }
                }
	}

         Simulator::Schedule(Seconds(0.5), &xValues);               
}


void xchangeTraffic(NetDeviceContainer &staDevices, NetDeviceContainer &apDevices, NetDeviceContainer &ueDevs, NetDeviceContainer &enbDevs){

        int tileId;
        
        Ptr<WifiPhy> apPhy;
        Ptr<WifiPhy> clPhy;
        int k = 0, xSum=0, nSum=0, mxTemp=0, mnTemp=0, mx = 0, mn = 0;
        Vector pos;
        float dif, dif_r=0; 
        int r = 1;
        while(r>=0){     
        for(tileId=1;tileId<=7;tileId++){
                std::cout<<"tileId"<<tileId<<std::endl;
                uint16_t tempChan=0;
                std::pair <std::multimap<uint32_t, uint32_t>::iterator, std::multimap<uint32_t, uint32_t>::iterator> ret;
                ret = PoAtile.equal_range(tileId);
                int ct1 = std::distance(ret.first, ret.second);               
                double poas[ct1];
                double npoas[ct1];
                double dpoas[ct1];
                double idpoas[ct1];
                int k = 0,  nSum=0, mxTemp=0, mnTemp=0, mx = 0, mn = 0;
                double xSum=0;
                uint32_t apid;
                double temp = 0;
                for (std::multimap<uint32_t, uint32_t>::iterator it=ret.first; it!=ret.second; ++it){
                        auto tt = PoAxValueMap.find(it->second); 
                        idpoas[k] = tt->first; 
                        poas[k] = tt->second.current;
                        npoas[k] = tt->second.numPoAues;                
                        k++;                        
                }
                
                for(int i= 0; i<ct1; i++){
                       xSum += poas[i];
                       nSum += npoas[i];                              
                }
                if(r==1){
                         for(int i= 0; i<ct1; i++){
                               double x=0, y=0;
                               if (xSum != 0) 
                                        x = poas[i]/xSum;
                               if (nSum != 0) 
                                        y = npoas[i]/nSum;
                               dpoas[i]= y - x; 
                               std::cout<<"xxxx ="<<x<<std::endl;
                               if(dpoas[i]>mxTemp){                  
                                       mxTemp = dpoas[i];
                                       mx = i;         
                               }
                               if(dpoas[i]<mnTemp){
                                       mnTemp = dpoas[i]; 
                                       mn = i;
                               }                                                
                        }
                }
                if(r==0){
                        for(int i= 0; i<ct1; i++){
                                if(idpoas[i]>numUes+numberOfEnbs){
                                       double x=0, y=0;
                                       if (xSum != 0) 
                                                x = poas[i]/xSum;
                                       if (nSum != 0) 
                                                y = npoas[i]/nSum;
                                       dpoas[i]= y - x; 
                                       std::cout<<"xxxx ="<<x<<std::endl;
                                       if(dpoas[i]>mxTemp){                  
                                               mxTemp = dpoas[i];
                                               mx = i; 
                                               //logFile << "mx" << mx << endl;        
                                       }
                                       if(dpoas[i]<mnTemp){
                                               mnTemp = dpoas[i]; 
                                               mn = i;
                                               //logFile << "mn" << mn << endl;
                                       }                                                
                                }                
                        }
                        dif_r = -1;
                }
                //logFile << "idpoas[mx]" << idpoas[mx] << endl;
                //logFile << "idpoas[mn]" << idpoas[mn] << endl;              

                        
                std::pair <std::multimap<uint32_t, uint32_t>::iterator, std::multimap<uint32_t, uint32_t>::iterator> rt;
                auto rrt =PoAxValueMap.find(idpoas[mx]);
                auto itt =PoAxValueMap.find(idpoas[mn]);
                if(itt->second.lte==0){
                        apPhy = ((WifiNetDevice*)PeekPointer(apDevices.Get(idpoas[mn]-numUes-numberOfEnbs)))->GetPhy();
                        tempChan = apPhy->GetChannelNumber();
                        std::cout<<tempChan<<std::endl;


                }
                int n = rrt->second.numPoAues;
               
                dif = n*dpoas[mx];
                logFile << "dif" << dif << endl;
                float dist[n];
                uint32_t ue2mv;                 
                Ptr<Node> UE;
                uint32_t iit;
                int d ;
                uint32_t nodeId;
                while(dif>dif_r){   
                        temp = 0;
                        d = 0;
                        int f=0; 
                        nodeId = -1; 
                        double x1,y1;          
                        for (auto iter = PoAUeMap.begin(); iter != PoAUeMap.end(); iter++){
                                d++;
                                if(iter->second == idpoas[mx]){                                        
                                        f++;                                       
                                        Ptr<Node> ue = ueNodes.Get(iter->first);                                        
                                        pos = ue->GetObject<MobilityModel>()->GetPosition();
                                        if(rrt->second.lte==0){                                             
                                                Ptr<Node> ap = wifiApNodes.Get(idpoas[mx]-numUes-numberOfEnbs);
                                                Vector pos_a = ap->GetObject<MobilityModel>()->GetPosition();
                                                if (temp < (pos_a.x-pos.x)*(pos_a.x-pos.x)+(pos_a.y-pos.y)*(pos_a.y-pos.y)){
                                                       temp = (pos_a.x-pos.x)*(pos_a.x-pos.x)+(pos_a.y-pos.y)*(pos_a.y-pos.y);
                                                       UE = ueNodes.Get(iter->first);
						       x1 = pos.x;
						       y1 = pos.y;
                                                       iit = iter->first;
                                                       nodeId = ueNodes.Get(iter->first)->GetId();
                                                }                        
                                        }
                                        if(rrt->second.lte==1){
                                                Ptr<Node> enb = enbNodes.Get(idpoas[mx]-numUes);
                                                Vector pos_a = enb->GetObject<MobilityModel>()->GetPosition();
                                                if (temp < (pos_a.x-pos.x)*(pos_a.x-pos.x)+(pos_a.y-pos.y)*(pos_a.y-pos.y)){
                                                       temp = (pos_a.x-pos.x)*(pos_a.x-pos.x)+(pos_a.y-pos.y)*(pos_a.y-pos.y);
                                                       UE = ueNodes.Get(iter->first);
						       x1 = pos.x;
						       y1 = pos.y;
                                                       iit = iter->first;
                                                       nodeId = ueNodes.Get(iter->first)->GetId();
                                                }
                                        }                        
                                }
                        }
                                               
                        auto it = PoAUeMap.find(nodeId);
                        if(nodeId != -1){                                               
                                if(rrt->second.lte==0 && itt->second.lte==0){
					double temp = 250000;
					uint32_t apid;
					for (uint32_t i = 0; i < wifiApNodes.GetN(); i++) {
						Ptr<Node> ap = wifiApNodes.Get(i);
                                                Vector pos_a = ap->GetObject<MobilityModel>()->GetPosition();
                                                if (temp > (pos_a.x-x1)*(pos_a.x-x1)+(pos_a.y-y1)*(pos_a.y-y1)){
                                                       temp = (pos_a.x-x1)*(pos_a.x-x1)+(pos_a.y-y1)*(pos_a.y-y1);
                                                       apid = i;
                                                }  
					}
					apPhy = ((WifiNetDevice*)PeekPointer(apDevices.Get(apid)))->GetPhy();
                        		tempChan = apPhy->GetChannelNumber();

                                        logFile << "User changing ap...." << endl;                                
          	  	                clPhy = ((WifiNetDevice*)PeekPointer(staDevices.Get(nodeId)))->GetPhy();
                                        int tempChan1 = clPhy->GetChannelNumber();
                                        if(tempChan != tempChan1){  
                  	  	                clPhy->SetChannelNumber(tempChan);
                                                it->second = apid + numUes + numberOfEnbs;                                
                                                rrt->second.numPoAues--;
                                                auto h = PoAxValueMap.find(apid + numUes + numberOfEnbs);
                                                h->second.numPoAues++;
                                                //itt->second.numPoAues++;  
                                                apchanges++;              
                                        }                                      
                                }
                                if(rrt->second.lte==0 && itt->second.lte==1){
                                        logFile << "user changing to lte" << endl;
                                        Ptr<Node> Ue = ueNodes.Get(nodeId); 
                                        ChangeToLte(Ue);
                                        auto gg =  ueEnbpowerMap.find(nodeId);
                                        rrt->second.numPoAues--;
                                        auto enbg = PoAxValueMap.find( gg->second.enb1);
                                        enbg->second.numPoAues++;
                                        it->second = gg->second.enb1;
                                        deassociatons++;
		                        ueDeassociated.push_back(nodeId);

                                }  
                                if(rrt->second.lte==1 && itt->second.lte==0){
                                        logFile << "user changing to wifi" << endl;
          	  	                clPhy = ((WifiNetDevice*)PeekPointer(staDevices.Get(nodeId)))->GetPhy();
                                        int tempChan1 = clPhy->GetChannelNumber();
                                        if(tempChan != tempChan1)              
          	  	                clPhy->SetChannelNumber(tempChan);
                                        Ptr<Node> Ue = ueNodes.Get(nodeId); 
                                        int i = ChangeToWifi(Ue);
                                        if(i == 1){
                                                rrt->second.numPoAues--;
                                                itt->second.numPoAues++;
                                                it->second = idpoas[mn];
                                        }
                                        reassociatons++;
                                }  
                                if(rrt->second.lte==1 && itt->second.lte==1){
                                        
                                        uint32_t ndid;
                                        uint16_t enbid = uint16_t(lteCoverage.find(nodeId)->second-numUes);//uint16_t(idpoas[mx]-numUes);
                                        uint16_t targetnodeid = -1;
                                         
                                        auto gg =  ueEnbpowerMap.find(nodeId);



                                        if((gg->second.rsrp1 >=25)&&(gg->second.rsrp1 != 255)&&(gg->second.rsrp1 < 75)/*&&(gg->second.enb1==idpoas[mn]-numUes)*/){                                        
                                                        targetnodeid = uint16_t(gg->second.enb1);
                                        }
                                        if(targetnodeid==-1){
                                        if((gg->second.rsrp2 >=25)&&(gg->second.rsrp2 != 255)&&(gg->second.rsrp2 < 75)/*&&(gg->second.enb2==idpoas[mn]-numUes)*/){                                        
                                                        targetnodeid = uint16_t(gg->second.enb2);
                                                        //logFile << "gg->second.enb2" << gg->second.enb2 << endl;
                                                        //logFile << targetnodeid << endl;
                                        }                                                                                               

                                        }  
                                        if(targetnodeid==-1){
                                        if((gg->second.rsrp3 >=25)&&(gg->second.rsrp3 != 255)&&(gg->second.rsrp3 < 75)/*&&(gg->second.enb3==idpoas[mn]-numUes)*/){                                        
                                                        targetnodeid = uint16_t(gg->second.enb3);
                                        }                                                                                               

                                        }  
                                                //logFile << "nodeId" << nodeId << endl;          
                                                //logFile << "enbid" << enbid << endl;
                                                //logFile << "targetnodeid" << targetnodeid << endl;                            
                                        
                                        if((targetnodeid != -1)&&(mx != mn)&&(targetnodeid<21)&&(gg->second.rsrp1 >=25)/*&&(gg->second.rsrp1 != 255)*/){
                                                                                                 
                                                lteHelper->HandoverRequest (Seconds(0),
                                                                    ueDevs.Get (nodeId),
                                                                    enbDevs.Get (enbid),
                                                                   enbDevs.Get (targetnodeid));

                                                logFile << "user start  handover to another enb with node id"<<nodeId<<"at time"<<Simulator::Now().GetSeconds()<<endl;
                                                rrt->second.numPoAues--;  
                                                auto enbg = PoAxValueMap.find(uint32_t(targetnodeid) + numUes);
                                                enbg->second.numPoAues++;
                                                it->second = uint32_t(targetnodeid); 
                                                enbchanges++;
                                        }
                                         
                                }
                        }
                        dif--;
                }      
                                     
        }        
        r--;
        }
        Simulator::Schedule(Seconds(0.5), &xchangeTraffic,staDevices,apDevices,ueDevs,enbDevs); // Callback every 0.5s
                                                
 }

static ns3::GlobalValue g_probLteWifi("probLteWifi",
		"probability UE use Lte or Wifi", ns3::DoubleValue(0.0),
		ns3::MakeDoubleChecker<double>());

static ns3::GlobalValue g_probQos("probQos", "probability UE download DT or GR",
		ns3::DoubleValue(0.0), ns3::MakeDoubleChecker<double>());
static ns3::GlobalValue g_macroEnbDlEarfcn("macroEnbDlEarfcn",
		"DL EARFCN used by macro eNBs", ns3::UintegerValue(2800),
		ns3::MakeUintegerChecker<uint16_t>());

static ns3::GlobalValue g_macroEnbBandwidth("macroEnbBandwidth",
		"bandwidth [num RBs] used by macro eNBs", ns3::UintegerValue(50),
		ns3::MakeUintegerChecker<uint16_t>());

static ns3::GlobalValue g_nMacroEnbSites("nMacroEnbSites",
		"How many macro sites there are", ns3::UintegerValue(7),
		ns3::MakeUintegerChecker<uint32_t>());

static ns3::GlobalValue g_enbHeight("enbHeight", "enb height [m]",
		ns3::DoubleValue(25.0), ns3::MakeDoubleChecker<double>());

static ns3::GlobalValue g_ueHeight("ueHeight", "ue height [m]",
		ns3::DoubleValue(1.5), ns3::MakeDoubleChecker<double>());

static ns3::GlobalValue g_interSiteDistance("interSiteDistance",
		"min distance between two nearby macro cell sites",
		ns3::DoubleValue(500), ns3::MakeDoubleChecker<double>());

static ns3::GlobalValue g_areaMarginFactor("areaMarginFactor",
		"how much the UE area extends outside the macrocell grid, "
				"expressed as meters", ns3::DoubleValue(250),
		ns3::MakeDoubleChecker<double>());

static ns3::GlobalValue g_macroEnbTxPowerDbm("macroEnbTxPowerDbm",
		"TX power [dBm] used by macro eNBs", ns3::DoubleValue(43.0),
		ns3::MakeDoubleChecker<double>());

static ns3::GlobalValue g_seed("seed", "Seed", ns3::UintegerValue(3),
		ns3::MakeUintegerChecker<uint32_t>());

static ns3::GlobalValue g_simTime("simTime",
		"Total duration of the simulation [s]", ns3::DoubleValue(4.3),
		ns3::MakeDoubleChecker<double>());

static ns3::GlobalValue g_macroUeDensity("numUePerCell",
		"How many macrocell UEs there are per cell", ns3::DoubleValue(60),
		ns3::MakeDoubleChecker<double>());

static ns3::GlobalValue g_numApPerCell("numApPerCell",
		"How many AP there are per cell", ns3::UintegerValue(4),
		ns3::MakeUintegerChecker<uint32_t>());

static ns3::GlobalValue g_generateRem("generateRem",
		"if true, will generate a REM and then abort the simulation;"
				"if false, will run the simulation normally (without generating any REM)",
		ns3::BooleanValue(false), ns3::MakeBooleanChecker());

static ns3::GlobalValue g_fadingTrace("fadingTrace",
		"The path of the fading trace (by default no fading trace "
				"is loaded, i.e., fading is not considered)",
		ns3::StringValue(""), ns3::MakeStringChecker());

static ns3::GlobalValue g_numBearersPerUe("numBearersPerUe",
		"How many bearers per UE there are in the simulation",
		ns3::UintegerValue(1), ns3::MakeUintegerChecker<uint16_t>());

static ns3::GlobalValue g_srsPeriodicity("srsPeriodicity",
		"SRS Periodicity (has to be at least greater than the number of UEs per eNB)",
		ns3::UintegerValue(320), ns3::MakeUintegerChecker<uint16_t>());

static ns3::GlobalValue g_outdoorUeMinSpeed("outdoorUeMinSpeed",
		"Minimum speed value of macro UE with random waypoint model [m/s].",
		ns3::DoubleValue(0.01), ns3::MakeDoubleChecker<double>());

static ns3::GlobalValue g_outdoorUeMaxSpeed("outdoorUeMaxSpeed",
		"Maximum speed value of macro UE with random waypoint model [m/s].",
		ns3::DoubleValue(5.0), ns3::MakeDoubleChecker<double>());

static ns3::GlobalValue g_wifiMacQueue("wifiMacQueue",
		"Max dimension Wifi Mac Queue", ns3::UintegerValue(400),
		ns3::MakeUintegerChecker<uint32_t>());

static ns3::GlobalValue g_wifiDelay("wifiQueueDelay",
		"Delay wifi mac queue [s]", ns3::DoubleValue(0.5),
		ns3::MakeDoubleChecker<double>());

static ns3::GlobalValue g_numBeacon("numBeacon", "num max missed beacon",
		ns3::UintegerValue(2), ns3::MakeUintegerChecker<uint32_t>());

static ns3::GlobalValue g_verbose("verbose",
		"if true, will generate debug info", ns3::BooleanValue(false),
		ns3::MakeBooleanChecker());

int main(int argc, char *argv[]) {

	logFile.open("log.txt");

	StringValue wifiRate = StringValue("OfdmRate72_2MbpsBW20MHz");

//string animFile = "mylte-animation.xml"; // Name of file for animation output

// Command line arguments
	CommandLine cmd;
	cmd.Parse(argc, argv);

//LogComponentEnable("MyLteSimulator", level);
//	LogComponentEnable("MyLteSimulator", LOG_LEVEL_INFO);
//	LogComponentEnable("StaWifiMac", LOG_LEVEL_LOGIC);
//LogComponentEnable("LteHelper", LOG_LEVEL_ALL);
//LogComponentEnable("LteUeRrc", LOG_LEVEL_ALL);
//LogComponentEnable("EdcaTxopN", LOG_LEVEL_DEBUG);
//LogComponentEnable("RrFfMacScheduler", LOG_LEVEL_ALL);
	ConfigStore inputConfig;
	inputConfig.ConfigureDefaults();

// parse again so you can override default values from the command line
	cmd.Parse(argc, argv);

// the scenario parameters get their values from the global attributes defined above
	UintegerValue uintegerValue;
	DoubleValue doubleValue;
	BooleanValue booleanValue;
	StringValue stringValue;
	GlobalValue::GetValueByName("probLteWifi", doubleValue);
	double probLteWifi = doubleValue.Get();
	GlobalValue::GetValueByName("probQos", doubleValue);
	double probQos = doubleValue.Get();
	GlobalValue::GetValueByName("macroEnbDlEarfcn", uintegerValue);
	uint16_t macroEnbDlEarfcn = uintegerValue.Get();
	GlobalValue::GetValueByName("macroEnbBandwidth", uintegerValue);
	uint16_t macroEnbBandwidth = uintegerValue.Get();
	GlobalValue::GetValueByName("nMacroEnbSites", uintegerValue);
	uint32_t nMacroEnbSites = uintegerValue.Get();
	GlobalValue::GetValueByName("interSiteDistance", doubleValue);
	double interSiteDistance = doubleValue.Get();
	GlobalValue::GetValueByName("enbHeight", doubleValue);
	double enbHeight = doubleValue.Get();
	GlobalValue::GetValueByName("ueHeight", doubleValue);
	double ueHeight = doubleValue.Get();
	GlobalValue::GetValueByName("areaMarginFactor", doubleValue);
	double areaMarginFactor = doubleValue.Get();
	GlobalValue::GetValueByName("macroEnbTxPowerDbm", doubleValue);
	double macroEnbTxPowerDbm = doubleValue.Get();
	GlobalValue::GetValueByName("seed", uintegerValue);
	uint32_t seed = uintegerValue.Get();
	GlobalValue::GetValueByName("simTime", doubleValue);
	double simTime = doubleValue.Get();
	GlobalValue::GetValueByName("numUePerCell", doubleValue);
	double numUePerCell = doubleValue.Get();
	GlobalValue::GetValueByName("numApPerCell", uintegerValue);
	uint32_t numApPerCell = uintegerValue.Get();
	GlobalValue::GetValueByName("generateRem", booleanValue);
	bool generateRem = booleanValue.Get();
	GlobalValue::GetValueByName("fadingTrace", stringValue);
	string fadingTrace = stringValue.Get();
	GlobalValue::GetValueByName("numBearersPerUe", uintegerValue);
	uint16_t numBearersPerUe = uintegerValue.Get();
	GlobalValue::GetValueByName("srsPeriodicity", uintegerValue);
	uint16_t srsPeriodicity = uintegerValue.Get();
	GlobalValue::GetValueByName("outdoorUeMinSpeed", doubleValue);
	double outdoorUeMinSpeed = doubleValue.Get();
	GlobalValue::GetValueByName("outdoorUeMaxSpeed", doubleValue);
	double outdoorUeMaxSpeed = doubleValue.Get();
	GlobalValue::GetValueByName("wifiMacQueue", uintegerValue);
	uint32_t wifiQueue = uintegerValue.Get();
	GlobalValue::GetValueByName("wifiQueueDelay", doubleValue);
	double wifiDelay = doubleValue.Get();
	GlobalValue::GetValueByName("numBeacon", uintegerValue);
	uint32_t nBeacons = uintegerValue.Get();
	GlobalValue::GetValueByName("verbose", booleanValue);
	verbose = booleanValue.Get();

	RngSeedManager::SetSeed(seed);
	logFile << "seed: " << seed << endl;
	numberOfEnbs = nMacroEnbSites * 3;
	uint32_t numberOfAps = numberOfEnbs * numApPerCell;
	numUes = numberOfEnbs * numUePerCell;
	double enbZ = enbHeight;
	double ueZ = ueHeight;
	double apZ = 2.5;
	packetSize = 1024;
	timeWindow = 0.5;
        
	rate = DataRate("1Mib/s");//change the offered load here
	sogliaWifi = rate * ns3::Seconds(timeWindow) / 5;
//	double rangeLte = 250;
	double rangeWifi = 70;
//	contentDim = 0.5 * 1024 * 1024 / 8; //in bytes
	logFile << "wifi_first " << endl;
	logFile << "num Ues = " << numUes << endl;
	logFile << "simTime (s) = " << simTime << endl;
	logFile << "speed (m/s) = " << outdoorUeMaxSpeed << endl;
	logFile << "AP per cell = " << numApPerCell << endl;
	logFile << "packetSize (bytes) = " << packetSize << endl;
	logFile << "dataRate = " << rate << endl;
	logFile << "timeWindow to check wifi rate (s) = " << timeWindow
			<< endl;
	logFile << "sogliaWifi (bitrate bit/s) = " << sogliaWifi / timeWindow
			<< endl;
//	logFile << "rangeLte (m) = " << rangeLte << endl;
	logFile << "rangeWifi (m) = " << rangeWifi << endl;

	Config::SetDefault("ns3::ArpCache::WaitReplyTimeout",
			TimeValue(Seconds(0.1)));
	Config::SetDefault("ns3::ArpCache::MaxRetries", UintegerValue(4));
	Config::SetDefault("ns3::ArpCache::PendingQueueSize", UintegerValue(4));

	Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize",
			UintegerValue(10 * 1024));
	Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(true));
	Config::SetDefault("ns3::RrFfMacScheduler::HarqEnabled",
			BooleanValue(true));
	Config::SetDefault("ns3::LteEnbRrc::SrsPeriodicity",
			UintegerValue(srsPeriodicity));
	Config::SetDefault("ns3::LteEnbPhy::TxPower",
			DoubleValue(macroEnbTxPowerDbm));
	Config::SetDefault("ns3::LteEnbPhy::NoiseFigure", DoubleValue(5));
	Config::SetDefault("ns3::LteUePhy::TxPower", DoubleValue(24));
	Config::SetDefault("ns3::LteUePhy::NoiseFigure", DoubleValue(7));
	Config::SetDefault("ns3::LteEnbRrc::DefaultTransmissionMode",
			UintegerValue(0));
//	Config::SetDefault("ns3::LteUePhy::UeMeasurementsFilterPeriod",
//				TimeValue(MilliSeconds (100)));

	Config::SetDefault("ns3::WifiMacQueue::MaxPacketNumber",
			UintegerValue(wifiQueue));
	Config::SetDefault("ns3::WifiMacQueue::MaxDelay",
			TimeValue(Seconds(wifiDelay)));
	Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold",
			StringValue("990000"));
	// disable/enable rts cts.
	Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold",
			StringValue("99000"));
	Config::SetDefault("ns3::WifiRemoteStationManager::MaxSlrc",
			UintegerValue(7));
	// Fix non-unicast data rate to be the same as that of unicast
	Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode",
			wifiRate);
	Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode",
			wifiRate);
	Config::SetDefault("ns3::ApWifiMac::EnableBeaconJitter",
			BooleanValue(true));
	Config::SetDefault("ns3::StaWifiMac::MaxMissedBeacons",
			UintegerValue(nBeacons));

	InternetStackHelper internet;

	ueNodes.Create(numUes);
	internet.Install(ueNodes);
	enbNodes.Create(numberOfEnbs);
	wifiApNodes.Create(numberOfAps);
	internet.Install(wifiApNodes);

//	string animFile = "mylte-animation.xml"; // Name of file for animation output

	lteHelper = CreateObject<LteHelper>();
	epcHelper = CreateObject<PointToPointEpcHelper>();
	lteHelper->SetEpcHelper(epcHelper);
	lteHelper->SetSchedulerType("ns3::RrFfMacScheduler");

	lteHelper->SetHandoverAlgorithmType("ns3::A2A4RsrqHandoverAlgorithm");
	lteHelper->SetHandoverAlgorithmAttribute("ServingCellThreshold",
			UintegerValue(25));
//	lteHelper->SetHandoverAlgorithmAttribute("NeighbourCellOffset",
//			UintegerValue(1));
        lteHelper->SetHandoverAlgorithmType ("ns3::A3RsrpHandoverAlgorithm");
        lteHelper->SetHandoverAlgorithmAttribute ("Hysteresis",
                                          DoubleValue (3.0));
        lteHelper->SetHandoverAlgorithmAttribute ("TimeToTrigger",
                                          TimeValue (MilliSeconds (256)));

	lteHelper->SetAttribute("PathlossModel",
			StringValue("ns3::FriisPropagationLossModel"));
//	lteHelper->SetAttribute("PathlossModel",
//			StringValue("ns3::RangePropagationLossModel"));
	lteHelper->SetPathlossModelAttribute("Frequency", DoubleValue(2.6e9));

	lteHelper->SetEnbAntennaModelType("ns3::ParabolicAntennaModel");
	lteHelper->SetEnbAntennaModelAttribute("Beamwidth", DoubleValue(70));
	lteHelper->SetEnbAntennaModelAttribute("MaxAttenuation", DoubleValue(20.0));
	lteHelper->SetEnbDeviceAttribute("DlEarfcn",
			UintegerValue(macroEnbDlEarfcn));
	lteHelper->SetEnbDeviceAttribute("UlEarfcn",
			UintegerValue(macroEnbDlEarfcn + 18000));
	lteHelper->SetEnbDeviceAttribute("DlBandwidth",
			UintegerValue(macroEnbBandwidth));
	lteHelper->SetEnbDeviceAttribute("UlBandwidth",
			UintegerValue(macroEnbBandwidth));

	lteHelper->SetUeDeviceAttribute("DlEarfcn",
			UintegerValue(macroEnbDlEarfcn));

	MobilityHelper mobility;
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	Ptr<ListPositionAllocator> positionAllocEnb = CreateObject<
			ListPositionAllocator>();
	Ptr<ListPositionAllocator> positionAllocWifiAp = CreateObject<
			ListPositionAllocator>();
	Ptr<PositionAllocator> positionAlloc;

	// Macro eNBs in 3-sector hex grid
	string line;
	fstream f("bts_list.txt");
	if (!f.is_open())
		perror("error while opening file enb position");
	logFile << "Reading file enb position" << endl;
	double minX = DBL_MAX;
	double maxX = DBL_MIN;
	double minY = DBL_MAX;
	double maxY = DBL_MIN;

	while (getline(f, line)) {
		int i = 0;
		double x;
		double y;
		istringstream iss(line);
		string token;
		while (getline(iss, token, ',')) {
			if (i == 0) {
				x = atof(token.c_str());
				if (x > maxX)
					maxX = x;
				if (x < minX)
					minX = x;
			} else if (i == 1) {
				y = atof(token.c_str());
				if (y > maxY)
					maxY = y;
				if (y < minY)
					minY = y;
			}
			i++;
		}
		Vector v(x, y, enbZ);
		positionAllocEnb->Add(v);
	}
	mobility.SetPositionAllocator(positionAllocEnb);
	mobility.Install(enbNodes);

	NetDeviceContainer enbDevs;
	for (uint32_t u = 0; u < nMacroEnbSites; ++u) {
		for (uint32_t i = 0; i < 3; i++) {
			lteHelper->SetEnbAntennaModelAttribute("Orientation",
					DoubleValue(120 * i + 30));
			enbDevs.Add(lteHelper->InstallEnbDevice(enbNodes.Get(u * 3 + i)));
		}
	}

	fstream f1("pico_cell_list.txt");
	if (!f1.is_open())
		perror("error while opening file ap wifi position");
	logFile << "Reading file wifi ap position" << endl;

	while (getline(f1, line)) {
		int i = 0;
		double x;
		double y;
		istringstream iss(line);
		string token;
		while (getline(iss, token, ',')) {
			if (i == 0) {
				x = atof(token.c_str());
			} else if (i == 1) {
				y = atof(token.c_str());
			}
			i++;
		}
		Vector v(x, y, apZ);
		positionAllocWifiAp->Add(v);
	}
	mobility.SetPositionAllocator(positionAllocWifiAp);
	mobility.Install(wifiApNodes);

	Box macroUeBox;
	if (nMacroEnbSites > 0) {

		macroUeBox = Box(minX - areaMarginFactor, maxX + areaMarginFactor,
				minY - areaMarginFactor, maxY + areaMarginFactor, ueZ, ueZ);
	}

	double macroUeAreaSize = (macroUeBox.xMax - macroUeBox.xMin)
			* (macroUeBox.yMax - macroUeBox.yMin);

	NS_LOG_LOGIC ("randomly allocating macro UEs in " << macroUeBox << " speedMin " << outdoorUeMinSpeed << " speedMax " << outdoorUeMaxSpeed);

	if (outdoorUeMaxSpeed != 0.0) {

		mobility.SetMobilityModel(
				"ns3::SteadyStateRandomWaypointMobilityModel");

		Config::SetDefault("ns3::SteadyStateRandomWaypointMobilityModel::MinX",
				DoubleValue(macroUeBox.xMin));
		Config::SetDefault("ns3::SteadyStateRandomWaypointMobilityModel::MinY",
				DoubleValue(macroUeBox.yMin));
		Config::SetDefault("ns3::SteadyStateRandomWaypointMobilityModel::MaxX",
				DoubleValue(macroUeBox.xMax));
		Config::SetDefault("ns3::SteadyStateRandomWaypointMobilityModel::MaxY",
				DoubleValue(macroUeBox.yMax));
		Config::SetDefault("ns3::SteadyStateRandomWaypointMobilityModel::Z",
				DoubleValue(ueZ));
		Config::SetDefault(
				"ns3::SteadyStateRandomWaypointMobilityModel::MaxSpeed",
				DoubleValue(outdoorUeMaxSpeed));
		Config::SetDefault(
				"ns3::SteadyStateRandomWaypointMobilityModel::MinSpeed",
				DoubleValue(outdoorUeMinSpeed));
		mobility.Install(ueNodes);

		// forcing initialization so we don't have to wait for Nodes to
		// start before positions are assigned (which is needed to
		// output node positions to file and to make AttachToClosestEnb work)
		for (NodeContainer::Iterator it = ueNodes.Begin(); it != ueNodes.End();
				++it) {
			(*it)->Initialize();
		}
	} else {
		positionAlloc = CreateObject<RandomBoxPositionAllocator>();
		Ptr<UniformRandomVariable> xVal = CreateObject<UniformRandomVariable>();
		xVal->SetAttribute("Min", DoubleValue(macroUeBox.xMin));
		xVal->SetAttribute("Max", DoubleValue(macroUeBox.xMax));
		positionAlloc->SetAttribute("X", PointerValue(xVal));
		Ptr<UniformRandomVariable> yVal = CreateObject<UniformRandomVariable>();
		yVal->SetAttribute("Min", DoubleValue(macroUeBox.yMin));
		yVal->SetAttribute("Max", DoubleValue(macroUeBox.yMax));
		positionAlloc->SetAttribute("Y", PointerValue(yVal));
		Ptr<UniformRandomVariable> zVal = CreateObject<UniformRandomVariable>();
		zVal->SetAttribute("Min", DoubleValue(macroUeBox.zMin));
		zVal->SetAttribute("Max", DoubleValue(macroUeBox.zMax));
		positionAlloc->SetAttribute("Z", PointerValue(zVal));
		mobility.SetPositionAllocator(positionAlloc);
		mobility.Install(ueNodes);
	}

	NetDeviceContainer ueDevs = lteHelper->InstallUeDevice(ueNodes);

	for (NodeContainer::Iterator it = ueNodes.Begin(); it != ueNodes.End();
			++it) {
		Ptr<Ipv4StaticRouting> ueStaticRouting =
				ipv4RoutingHelper.GetStaticRouting((*it)->GetObject<Ipv4>());
		ueStaticRouting->SetDefaultRoute(
				epcHelper->GetUeDefaultGatewayAddress(), 2);
		ueStaticRouting->AddNetworkRouteTo(Ipv4Address("1.0.0.0"),
				Ipv4Mask("255.0.0.0"), epcHelper->GetUeDefaultGatewayAddress(),
				2);
//		ueStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
//						Ipv4Mask("255.0.0.0"), epcHelper->GetUeDefaultGatewayAddress(),
//						2);
	}

	Ipv4AddressHelper ipv4h;
	PointToPointHelper p2ph;
	p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
	p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
//p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
	NetDeviceContainer internetDevices;

	logFile << "setting up wifi" << endl;

	wifiPgwContainer.Create(1);
	wifiPgw = wifiPgwContainer.Get(0);
	internet.Install(wifiPgw);

	ipv4h.SetBase("3.0.0.0", "255.255.255.252");
        //std::cout<<"wifiApNodes.GetN()"<<wifiApNodes.GetN()<<std::endl;
	for (uint32_t i = 0; i < wifiApNodes.GetN(); i++) {
		internetDevices = p2ph.Install(wifiPgw, wifiApNodes.Get(i));
		ipv4h.Assign(internetDevices);
		ipv4h.NewNetwork();
	}

//create wifi
//	YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
	YansWifiChannelHelper channel;
	channel.AddPropagationLoss("ns3::FriisPropagationLossModel", "Frequency",
			DoubleValue(5.0e9));
	channel.AddPropagationLoss("ns3::RangePropagationLossModel", "MaxRange",
			DoubleValue(rangeWifi));
	channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

	YansWifiPhyHelper phy = YansWifiPhyHelper::Default();
	phy.SetPcapDataLinkType(YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
//	phy.Set("TxGain", DoubleValue(2.0));
//	phy.Set("RxGain", DoubleValue(2.0));
	phy.Set("TxPowerEnd", DoubleValue(15.0));
	phy.Set("TxPowerStart", DoubleValue(15.0));
//	phy.Set("RxNoiseFigure", DoubleValue(6.0));
	phy.SetChannel(channel.Create());
	phy.Set("ShortGuardEnabled", BooleanValue(true));
	phy.Set("GreenfieldEnabled", BooleanValue(true));
	HtWifiMacHelper mac = HtWifiMacHelper::Default();
//QosWifiMacHelper mac = QosWifiMacHelper::Default();
	WifiHelper wifi;
	wifi.SetStandard(WIFI_PHY_STANDARD_80211n_5GHZ);
	wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode",
			wifiRate, "ControlMode", wifiRate);
	ipv4h.SetBase("192.168.0.0", "255.255.0.0");
	Ssid ssid = Ssid("default-wifi");

	mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
	/* setting blockack threshold for AP's BE queue */
//	mac.SetBlockAckThresholdForAc(AC_BE, 50);
	/* setting block inactivity timeout to 3*1024 = 3072 microseconds */
//	mac.SetBlockAckInactivityTimeoutForAc(AC_BE, 3);
	NetDeviceContainer apDevices;
        
	apDevices = wifi.Install(phy, mac, wifiApNodes);
          

        
        

	mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing",
			BooleanValue(false));
//	mac.SetBlockAckThresholdForAc(AC_BE, 0);
	NetDeviceContainer staDevices;
	staDevices = wifi.Install(phy, mac, ueNodes);

       
        for (uint32_t i = 0; i < ueNodes.GetN(); i++) {
                //Ptr<Node> node = ueNodes.Get(i);
                candENB candenb;
                candenb.rsrp1=0;
                candenb.enb1=0;
                candenb.rsrp2=0;
                candenb.enb2=0;
                candenb.rsrp3=0;
                candenb.enb3=0;
                pair<uint32_t, candENB> elm = { i, candenb };
                ueEnbpowerMap.insert(elm);              

        }

        uint32_t tileid;
        char c;
        int r=0;
        
        
        
	for (uint32_t i = 0; i < wifiApNodes.GetN(); i++) {
		//uint32_t nodeId = wifiApNodes.Get(i)->GetId();
                Ptr<Node> node = wifiApNodes.Get(i);
		Vector pos = node->GetObject<MobilityModel>()->GetPosition();
                uint32_t nodeId = node->GetId();
                xValue xvalue;
                pcValue pcvalue;
                xvalue.cthroughput = 0;
                xvalue.pthroughput = 0;
   	        xvalue.numPoAues = 0;                
                //xvalue.content=c;
                xvalue.previous=0;
                xvalue.current=0;
                xvalue.lte = 0;
                xvalue.orientation = 0;
                
                if((pos.x - 0)*(pos.x - 0) + (pos.y - 0)*(pos.y - 0)<250000){
                        tileid=1;
                        xvalue.tile=tileid;                  
                        pair<uint32_t, xValue> elem = { nodeId, xvalue };
                        PoAxValueMap.insert(elem);               
                        pair<uint32_t, uint32_t> el = { tileid, nodeId };
                        PoAtile.insert(el);

                }
                if((pos.x - 0)*(pos.x - 0) + (pos.y + 500)*(pos.y + 500)<250000){
                        tileid=2;                        
                        xvalue.tile=tileid;                  
                        pair<uint32_t, xValue> elem = { nodeId, xvalue };
                        PoAxValueMap.insert(elem);               
                     
                        pair<uint32_t, uint32_t> el = { tileid, nodeId };
                        PoAtile.insert(el);
                }
                if((pos.x - 0)*(pos.x - 0) + (pos.y - 500)*(pos.y - 500)<250000){

                        tileid=3;
                        xvalue.tile=tileid;                  
                        pair<uint32_t, xValue> elem = { nodeId, xvalue };
                        PoAxValueMap.insert(elem);               
                      
                        pair<uint32_t, uint32_t> el = { tileid, nodeId };
                        PoAtile.insert(el);
                }              
                if((pos.x + 433.01)*(pos.x + 433.01) + (pos.y + 250)*(pos.y + 250)<250000){

                        tileid=4;
                        xvalue.tile=tileid;                  
                        pair<uint32_t, xValue> elem = { nodeId, xvalue };
                        PoAxValueMap.insert(elem);               
                      
                        pair<uint32_t, uint32_t> el = { tileid, nodeId };
                        PoAtile.insert(el);
                }
                if((pos.x - 433.01)*(pos.x - 433.01) + (pos.y + 250)*(pos.y + 250)<250000){

                        tileid=5;
                        xvalue.tile=tileid;                  
                        pair<uint32_t, xValue> elem = { nodeId, xvalue };
                        PoAxValueMap.insert(elem);               
                       
                        pair<uint32_t, uint32_t> el = { tileid, nodeId };
                        PoAtile.insert(el);
                }
                if((pos.x + 433.01)*(pos.x + 433.01) + (pos.y - 250)*(pos.y - 250)<250000){
                        tileid=6;
                        xvalue.tile=tileid;                  
                        pair<uint32_t, xValue> elem = { nodeId, xvalue };
                        PoAxValueMap.insert(elem);               
                       
                        pair<uint32_t, uint32_t> el = { tileid, nodeId };
                        PoAtile.insert(el);
                }
                if((pos.x - 433.01)*(pos.x - 433.01) + (pos.y + 250)*(pos.y + 250)<250000){
                        tileid=7;
                        xvalue.tile=tileid;                  
                        pair<uint32_t, xValue> elem = { nodeId, xvalue };
                        PoAxValueMap.insert(elem);               
                    
                        pair<uint32_t, uint32_t> el = { tileid, nodeId };
                        PoAtile.insert(el);
                }
                
        }
        
       
        for (uint32_t i = 0; i < enbNodes.GetN(); i++) {
                uint32_t nodeId = enbNodes.Get(i)->GetId();
                Ptr<Node> node = enbNodes.Get(i);
                Vector pos = node->GetObject<MobilityModel>()->GetPosition();
        
              
                xValue xvalue;
                pcValue pcvalue;
                uint32_t tileid;
                xvalue.cthroughput = 0;
                xvalue.pthroughput = 0;
                xvalue.numPoAues = 0;
                xvalue.tile=tileid;
                xvalue.previous=0;
                xvalue.current=0;
                xvalue.lte = 1;
                xvalue.orientation = 0;
                if(pos.x == 0 && pos.y == 0){
                        tileid=1;
                        xvalue.tile=tileid;                  
                        pair<uint32_t, xValue> elem = { nodeId, xvalue };
                        PoAxValueMap.insert(elem);
                        pair<uint32_t, uint32_t> el = { tileid, nodeId };
                        PoAtile.insert(el);

                }
                if(pos.x == 0 && pos.y == -500){
                        tileid=2;                        
                        xvalue.tile=tileid;                  
                        pair<uint32_t, xValue> elem = { nodeId, xvalue };
                        PoAxValueMap.insert(elem); 
                        pair<uint32_t, uint32_t> el = { tileid, nodeId };
                        PoAtile.insert(el);
                }
                if(pos.x == 0 && pos.y == 500){

                        tileid=3;
                        xvalue.tile=tileid;                  
                        pair<uint32_t, xValue> elem = { nodeId, xvalue };
                        PoAxValueMap.insert(elem);   
                        pair<uint32_t, uint32_t> el = { tileid, nodeId };
                        PoAtile.insert(el);
                }

                            
                if(pos.x == -433.01 && pos.y == -250){

                        tileid=4;
                        xvalue.tile=tileid;                  
                        pair<uint32_t, xValue> elem = { nodeId, xvalue };
                        PoAxValueMap.insert(elem);   
                        pair<uint32_t, uint32_t> el = { tileid, nodeId };
                        PoAtile.insert(el);
                }
                if(pos.x == 433.01 && pos.y == -250){

                        tileid=5;
                        xvalue.tile=tileid;                  
                        pair<uint32_t, xValue> elem = { nodeId, xvalue };
                        PoAxValueMap.insert(elem); 
                        pair<uint32_t, uint32_t> el = { tileid, nodeId };
                        PoAtile.insert(el);
                }
                if(pos.x == -433.01 && pos.y == 250){
                        tileid=6;
                        xvalue.tile=tileid;                  
                        pair<uint32_t, xValue> elem = { nodeId, xvalue };
                        PoAxValueMap.insert(elem); 
                        pair<uint32_t, uint32_t> el = { tileid, nodeId };
                        PoAtile.insert(el);
                }
                if(pos.x == 433.01 && pos.y == 250){
                        tileid=7;
                        xvalue.tile=tileid;                  
                        pair<uint32_t, xValue> elem = { nodeId, xvalue };
                        PoAxValueMap.insert(elem);
                        pair<uint32_t, uint32_t> el = { tileid, nodeId };
                        PoAtile.insert(el);
                }
                
                                
        }

        
	
	for (uint32_t i = 0; i < wifiApNodes.GetN(); ++i) {

		ipv4h.Assign(apDevices);
		Ptr<Ipv4StaticRouting> wifiApStaticRouting =
				ipv4RoutingHelper.GetStaticRouting(
						wifiApNodes.Get(i)->GetObject<Ipv4>());
		wifiApStaticRouting->AddNetworkRouteTo(Ipv4Address("2.0.0.0"),
				Ipv4Mask("255.0.0.0"), 1);
		wifiApStaticRouting->AddNetworkRouteTo(Ipv4Address("192.168.0.0"),
				Ipv4Mask("255.255.0.0"), 2);
	}

	ipv4h.Assign(staDevices);

	Ipv4Address remoteHostAddrLte;
	Ipv4Address remoteHostAddrWifi;
	Ipv4InterfaceContainer ueIpIfaces;

	logFile << "setting up internet and remote host" << endl;

	Ptr<MobilityModel> MM;
// Create a single RemoteHost
	remoteHostContainer.Create(1);
	remoteHost = remoteHostContainer.Get(0);

	mobility.Install(remoteHost);
	MM = remoteHost->GetObject<MobilityModel>();
	MM->SetPosition(Vector(-1500, -1500, 0));
	internet.Install(remoteHostContainer);

// Create the Internet
	pgw = epcHelper->GetPgwNode();
	mobility.Install(pgw);
	mobility.Install(wifiPgw);
	MM = pgw->GetObject<MobilityModel>();
	MM->SetPosition(Vector(-1400, -1400, 0));
	MM = wifiPgw->GetObject<MobilityModel>();
	MM->SetPosition(Vector(-1400, -1500, 0));
	internetDevices = p2ph.Install(pgw, remoteHost);
	ipv4h.SetBase("1.0.0.0", "255.0.0.0");
	Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
// in this container, interface 0 is the pgw, 1 is the remoteHost
	remoteHostAddrLte = internetIpIfaces.GetAddress(1);
	ipv4h.SetBase("2.0.0.0", "255.0.0.0");
	internetDevices = p2ph.Install(wifiPgw, remoteHost);
	internetIpIfaces = ipv4h.Assign(internetDevices);
// in this container, interface 0 is the wifiPgw, 1 is the remoteHost
	remoteHostAddrWifi = internetIpIfaces.GetAddress(1);

	Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
			ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
	remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
			Ipv4Mask("255.0.0.0"), 1);
	remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("192.168.0.0"),
			Ipv4Mask("255.255.0.0"), 2);

	//Map Ap Mac Address
	for (uint32_t i = 0; i < wifiApNodes.GetN(); i++) {
		Ptr<NetDevice> dev = wifiApNodes.Get(i)->GetDevice(2);
		Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice>(dev);
		Ptr<WifiMac> mac = wifi_dev->GetMac();
		Mac48Address macAddr = mac->GetAddress();
		pair<Mac48Address, uint32_t> elem = { macAddr, i };
		wifiApMacMap.insert(elem);
	}

// Install the IP stack on the UEs
	ueIpIfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

	//Random variable for QoS service and other policies, but not used in this simulation
	Ptr<UniformRandomVariable> rand;
	rand = CreateObject<UniformRandomVariable>();
	for (uint32_t i = 0; i < ueNodes.GetN(); i++) {
		if (rand->GetValue(0, 1) < probLteWifi) {
			if (rand->GetValue(0, 1) >= probQos) {
				//Lte no qos
				Info info;
				info.lte = true;
				info.qos = false;
				info.started = false;
				info.startTime = -1;
				info.firstRxTime = -1;
				info.timeCoverageWifi = 0;
				info.timeStartCoverageWifi = 0;
				info.timeStartCoverageLte = simTime;
				info.rxLte = 0;
				info.rxWifi = 0;
                                info.rxTot = 0;
				info.check = false;
                                info.fileTxcomplete = -1;
				pair<uint32_t, Info> elem =
						{ ueNodes.Get(i)->GetId(), info };
				ueMap.insert(elem);
			}

			else {
				//Lte qos
				Info info;
				info.lte = true;
				info.qos = true;
				info.started = false;
				info.startTime = -1;
				info.firstRxTime = -1;
				info.timeCoverageWifi = 0;
				info.timeStartCoverageWifi = 0;
				info.timeStartCoverageLte = simTime;
				info.rxLte = 0;
				info.rxWifi = 0;
                                info.rxTot = 0;
				info.check = false;
                                info.fileTxcomplete = -1;
				pair<uint32_t, Info> elem =
						{ ueNodes.Get(i)->GetId(), info };
				ueMap.insert(elem);
				qosUes++;
			}
			lteUes++;
		} else {
			if (rand->GetValue(0, 1) >= probQos) {
				//Wifi no qos
				Info info;
				info.lte = false;
				info.qos = false;
				info.started = false;
				info.startTime = -1;
				info.firstRxTime = -1;
				info.timeCoverageWifi = 0;
				info.timeStartCoverageWifi = 0;
				info.timeStartCoverageLte = simTime;
				info.rxLte = 0;
				info.rxWifi = 0;
                                info.rxTot = 0;
				info.check = false;
                                info.fileTxcomplete = -1;
				pair<uint32_t, Info> elem =
						{ ueNodes.Get(i)->GetId(), info };
				ueMap.insert(elem);
			} else {
				//Wifi qos
				Info info;
				info.lte = false;
				info.qos = true;
				info.started = false;
				info.startTime = -1;
				info.firstRxTime = -1;
				info.timeCoverageWifi = 0;
				info.timeStartCoverageWifi = 0;
				info.timeStartCoverageLte = simTime;
				info.rxLte = 0;
				info.rxWifi = 0;
                                info.rxTot = 0;
				info.check = false;
                                info.fileTxcomplete = -1;
				pair<uint32_t, Info> elem =
						{ ueNodes.Get(i)->GetId(), info };
				ueMap.insert(elem);
				qosUes++;
			}
			wifiUes++;
		}
	}

// attachment (needs to be done after IP stack configuration)
// macro UEs attached to the best (strongest signal) macro eNB
	lteHelper->Attach(ueDevs);
        EpsBearer bearer(EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
        EpsBearer bearerq(EpsBearer::GBR_CONV_VIDEO);
        Ptr<EpcTft> tft = Create<EpcTft>();
        EpcTft::PacketFilter dlpf;
        dlpf.localPortStart = dlPort;
        dlpf.localPortEnd = dlPort;
        tft->Add(dlpf);
        for (uint32_t u = 0; u < ueNodes.GetN (); ++u){
	        
                if(ueMap.find(u)->second.qos){
                        lteHelper->ActivateDedicatedEpsBearer(ueDevs.Get(u), bearerq, tft);
                }
                if(!ueMap.find(u)->second.qos){
                        lteHelper->ActivateDedicatedEpsBearer(ueDevs.Get(u), bearer, tft);
                }
        }      

// this enables handover for macro eNBs
	lteHelper->AddX2Interface(enbNodes);

	PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
			InetSocketAddress(Ipv4Address::GetAny(), dlPort));
	serverApps.Add(dlPacketSinkHelper.Install(ueNodes));
	serverApps.Start(Seconds(0));

	Ptr<RadioEnvironmentMapHelper> remHelper;
	if (generateRem) {

		remHelper = CreateObject<RadioEnvironmentMapHelper>();
		remHelper->SetAttribute("ChannelPath", StringValue("/ChannelList/0"));
		remHelper->SetAttribute("OutputFile", StringValue("rem.out"));
		remHelper->SetAttribute("XMin", DoubleValue(macroUeBox.xMin));
		remHelper->SetAttribute("XMax", DoubleValue(macroUeBox.xMax));
		remHelper->SetAttribute("YMin", DoubleValue(macroUeBox.yMin));
		remHelper->SetAttribute("YMax", DoubleValue(macroUeBox.yMax));
		remHelper->SetAttribute("Z", DoubleValue(ueZ));
		remHelper->SetAttribute("XRes", UintegerValue(500));
		remHelper->SetAttribute("YRes", UintegerValue(500));
		remHelper->SetAttribute("Earfcn", UintegerValue(macroEnbDlEarfcn));
		remHelper->SetAttribute("Bandwidth", UintegerValue(macroEnbBandwidth));
//Noise power lte = noiseFigure(7 db) + kT(-174 dbm) + 10*log(bandwidth(180kHz * 50))
//		remHelper->SetAttribute("NoisePower", DoubleValue(1.7957e-10));

		remHelper->Install();
// simulation will stop right after the REM has been generated
	} else {
		Simulator::Stop(Seconds(simTime));
	}

	PrintGnuplottableEnbListToFile("enbs.txt");
	PrintGnuplottableUeListToFile("ues.txt");
	PrintGnuplottableWifiApListToFile("wifiAps.txt");

	/*for (uint32_t u = 0; u < enbNodes.GetN(); u++) {
	 logFile << endl << "Assigned IP to EnB (node ID) "
	 << enbNodes.Get(u)->GetId() << ":" << endl;
	 ipv4 = enbNodes.Get(u)->GetObject<Ipv4>(); // Get Ipv4 instance of the node
	 for (uint32_t i = 0; i < ipv4->GetNInterfaces(); i++) {
	 addr = ipv4->GetAddress(i, 0).GetLocal(); // Get Ipv4InterfaceAddress of xth interface.
	 logFile << addr << " interface: " << i << endl;
	 }
	 }*/
	if (verbose) {
		Ptr<Ipv4> ipv4;
		Ipv4Address addr;
		for (uint32_t u = 0; u < ueNodes.GetN(); u++) {
			logFile << endl << "Assigned IP to UE (node ID) "
					<< ueNodes.Get(u)->GetId() << ":" << endl;
			ipv4 = ueNodes.Get(u)->GetObject<Ipv4>(); // Get Ipv4 instance of the node
			for (uint32_t i = 0; i < ipv4->GetNInterfaces(); i++) {
				addr = ipv4->GetAddress(i, 0).GetLocal(); // Get Ipv4InterfaceAddress of xth interface.
				logFile << addr << " interface: " << i << endl;
			}
		}
		for (uint32_t u = 0; u < wifiApNodes.GetN(); u++) {
			logFile << endl << "Assigned IP to wifi APs (node ID) "
					<< wifiApNodes.Get(u)->GetId() << ":" << endl;
			ipv4 = wifiApNodes.Get(u)->GetObject<Ipv4>(); // Get Ipv4 instance of the node
			for (uint32_t i = 0; i < ipv4->GetNInterfaces(); i++) {
				addr = ipv4->GetAddress(i, 0).GetLocal(); // Get Ipv4InterfaceAddress of xth interface.
				logFile << addr << " interface: " << i << endl;
			}
		}
		logFile << endl << "Assigned IP to remoteHost (node ID)"
				<< remoteHost->GetId() << ":" << endl;
		ipv4 = remoteHost->GetObject<Ipv4>(); // Get Ipv4 instance of the node
		for (uint32_t i = 0; i < ipv4->GetNInterfaces(); i++) {
			addr = ipv4->GetAddress(i, 0).GetLocal(); // Get Ipv4InterfaceAddress of xth interface.
			logFile << addr << " interface: " << i << endl;
		}

		logFile << endl << "Assigned IP to pgw (node ID):" << pgw->GetId()
				<< ":" << endl;
		ipv4 = pgw->GetObject<Ipv4>(); // Get Ipv4 instance of the node
		for (uint32_t i = 0; i < ipv4->GetNInterfaces(); i++) {
			addr = ipv4->GetAddress(i, 0).GetLocal(); // Get Ipv4InterfaceAddress of xth interface.
			logFile << addr << " interface: " << i << endl;
		}

		logFile << endl << "Assigned IP to wifi pgw (node ID):"
				<< wifiPgw->GetId() << ":" << endl;
		ipv4 = wifiPgw->GetObject<Ipv4>(); // Get Ipv4 instance of the node
		for (uint32_t i = 0; i < ipv4->GetNInterfaces(); i++) {
			addr = ipv4->GetAddress(i, 0).GetLocal(); // Get Ipv4InterfaceAddress of xth interface.
			logFile << addr << " interface: " << i << endl;
		}
		logFile << endl;
	}

//  Uncomment to enable LTE output
//	lteHelper->EnableDlMacTraces();
//	lteHelper->EnableRlcTraces();
//	Ptr<RadioBearerStatsCalculator> rlcStats = lteHelper->GetRlcStats();
//	rlcStats->SetAttribute("StartTime", TimeValue(Seconds(0)));
//	rlcStats->SetAttribute("EpochDuration", TimeValue(Seconds(simTime)));
        Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/RecvMeasurementReport", MakeCallback (&NotifyRecvMeasurementReportTrace));

	Config::Connect(
			"/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::StaWifiMac/Assoc",
			MakeCallback(&StaMacAssoc));
	Config::Connect(
			"/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::StaWifiMac/DeAssoc",
			MakeCallback(&StaMacDeAssoc));

	Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
			MakeCallback(&NotifyConnectionEstablishedEnb));
	Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
			MakeCallback(&NotifyConnectionEstablishedUe));

	Config::Connect("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx",
			MakeCallback(&SinkRx));

	// Trace Collisions
	Config::ConnectWithoutContext(
			"/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop",
			MakeCallback(&MacTxDrop));
	Config::ConnectWithoutContext(
			"/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRxDrop",
			MakeCallback(&MacRxDrop));
	Config::ConnectWithoutContext(
			"/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyRxDrop",
			MakeCallback(&PhyRxDrop));
	Config::ConnectWithoutContext(
			"/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxDrop",
			MakeCallback(&PhyTxDrop));

//	Config::Connect(
//			"/NodeList/*/DeviceList/*/RemoteStationManager/MacTxFinalDataFailed",
//			MakeCallback(&MacTxFailed));

	Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
			MakeCallback(&NotifyHandoverEndOkUe));

	AsciiTraceHelper ascii;
//	phy.EnableAsciiAll(ascii.CreateFileStream("myLteTraceAscii.tr"));

// Uncomment to enable PCAP tracing
//	NodeContainer pgwContainer;
//	pgwContainer.Add(pgw);
//	p2ph.EnablePcap("pcap/MyPcapP2pWifiPgw", wifiPgwContainer, false);
//p2ph.EnablePcap("pcap/MyPcapP2pPgw", pgwContainer, false);
//	p2ph.EnablePcap("pcap/MyPcapP2pRemotehost", remoteHostContainer, false);
//p2ph.EnablePcap("MyPcapP2pBackbone", backboneNodes, false);
	if (verbose) {
//		phy.EnablePcapAll("pcap/MyPcapWifi", false);
	}

//FlowMonitorHelper
	FlowMonitorHelper flowmon;
	Ptr<FlowMonitor> monitor;

	monitor = flowmon.Install(ueNodes);
	monitor = flowmon.Install(remoteHost);
	monitor->SetAttribute("DelayBinWidth", ns3::DoubleValue(0.001));
	monitor->SetAttribute("JitterBinWidth", ns3::DoubleValue(.001));
	monitor->SetAttribute("PacketSizeBinWidth", ns3::DoubleValue(20));

	ueThroughputOut.open("throughputInTime.txt"); //Write Network measurements to output file
	ueThroughputOut << "Time\tUE id\tIP wifi\tIP lte\tRxBits Wifi\tRxBits Lte"
			<< endl;

	Simulator::Schedule(Seconds(0.5), &Throughput);
        //Simulator::Schedule(Seconds(0.5), &xValues);
        //Simulator::Schedule(Seconds(0.5), &xchangeTraffic,staDevices,apDevices,ueDevs,enbDevs);


//Uncomment to enable NetAnim output
//	char c[11];
//	string str;
//	for (uint32_t u = 0; u < ueNodes.GetN(); ++u) {
//		Ptr<Node> ueNode = ueNodes.Get(u);
//		snprintf(c, sizeof c, "%lu", (unsigned long) ueNode->GetId());
//		str = c;
//		AnimationInterface::SetNodeDescription(ueNode, "UE: " + str); // Optional
//	}
//	for (uint32_t u = 0; u < enbNodes.GetN(); ++u) {
//		Ptr<Node> enbNode = enbNodes.Get(u);
//		snprintf(c, sizeof c, "%lu", (unsigned long) enbNode->GetId());
//		str = c;
//		AnimationInterface::SetNodeDescription(enbNode, "eNb: " + str); // Optional
//	}
//	snprintf(c, sizeof c, "%lu", (unsigned long) remoteHost->GetId());
//	str = c;
//	AnimationInterface::SetNodeDescription(remoteHost, "remoteHost: " + str); // Optional
//	snprintf(c, sizeof c, "%lu", (unsigned long) pgw->GetId());
//	str = c;
//	AnimationInterface::SetNodeDescription(pgw, "pgw: " + str); // Optional
//	for (uint32_t u = 0; u < wifiApNodes.GetN(); ++u) {
//		Ptr<Node> wifiApNode = wifiApNodes.Get(u);
//		snprintf(c, sizeof c, "%lu", (unsigned long) wifiApNode->GetId());
//		str = c;
//		AnimationInterface::SetNodeDescription(wifiApNode, "wifiAP: " + str); // Optional
//	}
//	snprintf(c, sizeof c, "%lu", (unsigned long) wifiPgw->GetId());
//	str = c;
//	AnimationInterface::SetNodeDescription(wifiPgw, "wifiPgw: " + str); // Optional
//
//	AnimationInterface::SetNodeColor(enbNodes, 0, 255, 0);
//	AnimationInterface::SetNodeColor(remoteHost, 0, 0, 255);
//	AnimationInterface::SetNodeColor(pgw, 255, 255, 0);
//	AnimationInterface::SetNodeColor(wifiPgw, 255, 255, 0);
//	AnimationInterface::SetNodeColor(wifiApNodes, 0, 255, 255);
//
//	AnimationInterface anim(animFile);
//	anim.EnableIpv4RouteTracking("routing_table.xml", Seconds(0),
//			Seconds(simTime), Seconds(0.25));
//anim.EnablePacketMetadata(true);
//	for (uint32_t i = 1; i < simTime; i++) {
//		Simulator::Schedule(Seconds(i), &StartApplications);
//	}

//MobilityHelper::EnableAsciiAll(ascii.CreateFileStream("myMobility.mob"));

	logFile << "Starting simulation" << endl;
	Simulator::Run();

	epcHelper = 0;
	lteHelper = 0;
//	monitor->CheckForLostPackets();
	monitor->SerializeToXmlFile("flowmon.xml", true, true);

	logFile << "Simulation end" << endl << endl;

	logFile << "wifi nodes: " << wifiUes << endl;
	logFile << "lte nodes: " << lteUes << endl;
	logFile << "Qos nodes: " << qosUes << endl;

	logFile << "wifi associated: " << wifiAssoc << endl;
	logFile << "lte associated: " << lteAssoc << endl;

	logFile << "wifi de-associated: " << deassociatons << endl;
	logFile << "wifi re-associated: " << reassociatons << endl;
        logFile << "lte manual handover: " << enbchanges << endl;
        logFile << "lte manual handover: " << apchanges << endl;
        

	logFile << "wifi switched to lte : " << switched
			<< endl;

	ofstream delayWifi;
	delayWifi.open("delayWifi.dat");
	ofstream delayLte;
	delayLte.open("delayLte.dat");
	ofstream jitterWifi;
	jitterWifi.open("jitterWifi.dat");
	ofstream jitterLte;
	jitterLte.open("jitterLte.dat");
	ofstream packetLostWifi;
	packetLostWifi.open("packetLostWifi.dat");
	ofstream packetLostLte;
	packetLostLte.open("packetLostLte.dat");

	Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(
			flowmon.GetClassifier());

	map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
	for (map<FlowId, FlowMonitor::FlowStats>::iterator flow =
			stats.begin(); flow != stats.end(); flow++) {
		Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow->first);
		if (t.sourceAddress == "1.0.0.2") {
			if (flow->second.txPackets > 0) {
				packetLostLte
						<< ((double) flow->second.txPackets
								- (double) flow->second.rxPackets)
								/ (double) flow->second.txPackets * 100
						<< endl;
			}
			if (flow->second.rxPackets > 0) {
				delayLte
						<< (flow->second.delaySum.GetMicroSeconds()
								/ flow->second.rxPackets) << endl;
			}
			if (flow->second.rxPackets > 1) {
				jitterLte
						<< (flow->second.jitterSum.GetMicroSeconds()
								/ (flow->second.rxPackets - 1)) << endl;
			}
		} else if (t.sourceAddress == "2.0.0.2") {
			if (flow->second.txPackets > 0) {
				packetLostWifi
						<< ((double) flow->second.txPackets
								- (double) flow->second.rxPackets)
								/ (double) flow->second.txPackets * 100
						<< endl;
			}
			if (flow->second.rxPackets > 0) {
				delayWifi
						<< (flow->second.delaySum.GetMicroSeconds()
								/ flow->second.rxPackets) << endl;
			}
			if (flow->second.rxPackets > 1) {
				jitterWifi
						<< (flow->second.jitterSum.GetMicroSeconds()
								/ (flow->second.rxPackets - 1)) << endl;
			}
		}
	}
        ofstream finishTimeFile;
        finishTimeFile.open("finishTime.dat");

	ofstream startTimeFile;
	startTimeFile.open("startTime.dat");
	ofstream coverageFile;
	coverageFile.open("coverage.dat");
	completionTime.open("timeStats.txt"); //Write time stats to output file
	completionTime
			<< "UEid\tStartTime\tTimeFirstRx\tTimeCovWifi (%)\tTimeCovLte (%)"
			<< endl;
        finishTimeFile
			<< "UEid\tfileTxcomplete\tqos"<< endl;
        
	for (auto it = ueMap.begin(); it != ueMap.end(); it++) {
		if (it->second.timeStartCoverageWifi != 0)
			it->second.timeCoverageWifi += (simTime
					- it->second.timeStartCoverageWifi);
		completionTime << it->first << "\t" << it->second.startTime << "\t"
				<< it->second.firstRxTime << "\t"
				<< (it->second.timeCoverageWifi / simTime) * 100 << "\t"
				<< ((simTime - it->second.timeStartCoverageLte) / simTime) * 100
				<< endl;
                finishTimeFile << it->first << "\t" << it->second.fileTxcomplete << "\t" << it->second.qos << endl;
		startTimeFile << it->second.startTime << endl;
		coverageFile << (it->second.timeCoverageWifi / simTime) * 100 << "\t"
				<< ((simTime - it->second.timeStartCoverageLte) / simTime) * 100
				<< endl;
	}

	PrintDrop();

	completionTime.close();
	ueThroughputOut.close();
        finishTimeFile.close();
	logFile.close();
	delayLte.close();
	delayWifi.close();
	jitterLte.close();
	jitterWifi.close();
	packetLostLte.close();
	packetLostWifi.close();
	startTimeFile.close();
	coverageFile.close();

	Simulator::Destroy();

	return 0;
}

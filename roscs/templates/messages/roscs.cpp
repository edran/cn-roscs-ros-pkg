#include "ros/ros.h"
#include <iostream>
#include <list>
#include <map>
#include "ros/time.h"
#include "ros/duration.h"
#include "ros/console.h"
//#include <dlfcn.h>
//#include "messageMapper.h"
#include "CsSubscriber.h"
<?messageIncludes?>

using namespace std;

//list<ros::Publisher> pubs;
list<ros::Subscriber> subs;

/*
void *dlhandle;


extern "C" void loadCode(char* lib) {	
	dlhandle = dlopen(lib,RTLD_NOW|RTLD_GLOBAL);	
	if (dlhandle == NULL) {
		cerr << "Unable to load "<<lib<<endl;
		exit(-1);
	}
}
*/

extern "C" uint64_t getRosNow() {
	ros::Time now = ros::Time::now();
	return ((uint64_t)now.sec)*1000000000ull+(uint64_t)now.nsec;
}
extern "C" void rosSleep(uint32_t ms) {
	ros::Duration d = ros::Duration(ms/1000, (ms%1000)*1000000);
	d.sleep();
}


extern "C" void rosInit(char* name, int argc, char** argv, uint32_t initOptions) {
	ros::init(argc, argv, name,initOptions);
	/*cout<<"Args: "<<argc<<endl;
	for(int i=0; i< argc; i++) {
		cout << argv[i]<<endl;
	}*/
}
extern "C" void rosShutdown() {
	ros::shutdown();
}
extern "C" bool getRosOk() {
	return ros::ok();
}
extern "C" bool checkMaster() {
	return ros::master::check();
}


extern "C" ros::NodeHandle* getNode(char* name) {
	return new ros::NodeHandle(name);
}

extern "C" void destroyNode(ros::NodeHandle* node) {	
	if(node!=NULL && node->ok()) {
		node->shutdown();	
		delete node;
	}
}

extern "C"  ros::Publisher* advertiseTopic(ros::NodeHandle* node, char* topic, uint64_t messageId, int queueSize) {
	ros::Publisher tmp;
	
	//RosCs::messageMapper::getTypeSpecificPublisher(node, tmp, topic,messageId,queueSize);
	<?cppMakePublisher?>
	//ros::Publisher* ptr = (ros::Publisher*)malloc(sizeof(tmp));
	//*ptr = tmp;
	ros::Publisher* ptr = new ros::Publisher(tmp);
	return ptr;
	//pubs.push_back(tmp);
	//return &(pubs.back());
}




<?cppGetSizeOfMessage?>

<?cppArray2RosMethods?>
extern "C" void sendmsg(ros::Publisher* pub, uint64_t messageId, int* data) {
	switch(messageId) {
		<?cppSendBody?>
		default:
			std::cerr << "Unknown message id in sendmsg: "<<messageId<<std::endl;
	}
}

<?cppMessageHandleMethods?>

<?cppSubscriptionMethods?>

extern "C" bool isNodeOk(ros::NodeHandle* node) {
	return (node!=NULL && node->ok());
}

extern "C" void rosDebug(char* str) {
	ROS_DEBUG_THROTTLE(5,"%s",str);
}
extern "C" void rosWarn(char* str) {
	ROS_WARN_THROTTLE(3,"%s",str);
}
extern "C" void rosError(char* str) {
	ROS_ERROR_THROTTLE(1,"%s",str);
}
extern "C" void rosFatal(char* str) {
	ROS_FATAL("%s",str);
}
extern "C" void rosInfo(char* str) {
	ROS_INFO_THROTTLE(10,"%s",str);
}

extern "C" void stopSubscriber(void** ptr) {
       ((RosCs::Runable*)(*ptr))->Shutdown();
}
extern "C" char* getParam(ros::NodeHandle* node, char* paramName) {
	//std::cout<<"here"<<std::endl;
	//std::cout<<"param is: "<<paramName<<std::endl;
	XmlRpc::XmlRpcValue val;	
	node->getParam(paramName,val);
	stringstream s;
	s<<val;
	//std::cout<<"called: "<<ok<<" | "<<val<<std::endl;
	std::string str = s.str();
	char* ret = (char*)malloc((str.size()+1)*sizeof(char));
	strcpy(ret,str.c_str());
	
	return ret;
}
extern "C" char* searchParam(ros::NodeHandle* node, char* paramName) {
	std::string val;	
	node->searchParam(paramName,val);	
	char* ret = (char*)malloc((val.size()+1)*sizeof(char));
	strcpy(ret,val.c_str());
	return ret;
}
extern "C" bool hasParam(ros::NodeHandle* node, char* paramName) {
	return node->hasParam(paramName);
}

extern "C" void deleteParam(ros::NodeHandle* node, char* paramName) {
	node->deleteParam(paramName);
	return;
}
extern "C" void setParamInt(ros::NodeHandle* node, char* key, int val) {
	node->setParam(key,val);
	return;
}
extern "C" void setParamBool(ros::NodeHandle* node, char* key, bool val) {
	node->setParam(key,val);
	return;
}
extern "C" void setParamDouble(ros::NodeHandle* node, char* key, double val) {
	node->setParam(key,val);
	return;
}
extern "C" void setParamString(ros::NodeHandle* node, char* key, char* val) {
	node->setParam(key,val);
	return;
}

extern "C" void freeCharPtr(char* p) {
	free(p);
}
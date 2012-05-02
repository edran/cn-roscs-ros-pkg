#include "ros/ros.h"
#include <iostream>
#include <list>
#include <map>
#include "ros/time.h"
#include "ros/duration.h"
#include "ros/console.h"
#include "CsAdvertiseService.h"
<?messageIncludes?>

using namespace std;

list<ros::ServiceClient> client;

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

extern "C" ros::NodeHandle* getService(char* name) {
	return new ros::NodeHandle(name);
}

extern "C" void destroyService(ros::NodeHandle* node) {	
	if(node!=NULL && node->ok()) {
		node->shutdown();	
		delete node;
	}
}

extern "C"  ros::ServiceClient* serviceClient(ros::NodeHandle* node, char* topic, uint64_t messageId ) {
	ros::ServiceClient tmp;
	
	<?cppServiceClient?>
	
	client.push_back(tmp);
	return &(client.back());
}

<?cppGetSizeOfMessage?>

<?cppArray2RosMethods?>

<?cppServiceHandleMethods?>

extern "C" bool callservice(ros::ServiceClient* service, uint64_t messageId, int* request, char** response) {
	switch(messageId) {
		<?cppCallServiceBody?>
		default:
			std::cerr << "Unknown message id in sendmsg: "<<messageId<<std::endl;
			return false;
	}
}

extern "C"  void advertiseService(ros::NodeHandle* node, char* topic, uint64_t messageId, bool (*handle)(char**,char**), void (*handleFree)(char*) ) {
	switch(messageId) {
		<?advertiseService?>
		default: std::cerr<<"Unknown message id: "<< messageId << std::endl;
	}
// 	RosCs::CsAdvertiseService<beginner_tutorials::AddTwoInts,beginner_tutorials::AddTwoIntsResponse,beginner_tutorials::AddTwoIntsRequest>(handle,translate,translate2Ros).AdvertiseAndListen(node,topic);
}

extern "C" void freePointer(char* pointer) {
	free(pointer);
}

extern "C" bool isServiceOk(ros::NodeHandle* node) {
	return (node!=NULL && node->ok());
}
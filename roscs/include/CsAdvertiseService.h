#ifndef CSADVERTISESERVICE
#define CSADVERTISESERVICE 1
#include "Runable.h"
#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <ros/transport_hints.h>

namespace RosCs {
	template<class M,class MRes,class MReq>
	class CsAdvertiseService : Runable {
		protected:
			bool (*cscallback)(char**,char**);
			void (*translator)(MReq &,char**);
			void (*translator2Ros)(MRes &,char**);
			void (*freeCS)(char*);
			ros::ServiceServer sub;
			ros::NodeHandle nh;
			ros::CallbackQueue queue;
		public:
			CsAdvertiseService(bool (*cshandle)(char**,char**),void (*translator)(MReq &,char**),void (*translator2Ros)(MRes &,char**),void (*freeCS)(char*)) {
				this->cscallback=cshandle;
				this->translator=translator;
				this->translator2Ros=translator2Ros;
				this->freeCS = freeCS;
				this->running = true;
			}
			
			bool HandleServiceCall(MReq &request, MRes &response) {
				char** requestP = (char**)malloc(sizeof(char*));
				
				char** responseP = (char**)malloc(sizeof(char*));
				translator(request,requestP);

				bool ret = cscallback(requestP,responseP);
				//std::cout << "Success : " << ret << std::endl;
				if( ret ) {
					translator2Ros(response,responseP);
				} else {
					ret = false;
				}

				free(*requestP);
				free(requestP);
				freeCS(*responseP);
				free(responseP);
				return ret;
			}
			void AdvertiseSubscribe(ros::NodeHandle* orignh, char* topic) {
				nh = ros::NodeHandle(*orignh);
				nh.setCallbackQueue(&queue);
					sub = nh.advertiseService(topic, &CsAdvertiseService<M,MRes,MReq>::HandleServiceCall,this);
			}
			void Listen() {
				while(nh.ok() && this->running) {
					queue.callAvailable(ros::WallDuration(1));
				}	
				sub.shutdown();
			}
			void AdvertiseAndListen(ros::NodeHandle* orignh, char* topic) {
				AdvertiseSubscribe(orignh,topic);
				Listen();
			}
			~CsAdvertiseService() {
				running = false;
				sub.shutdown();
			}
	};
};
#endif
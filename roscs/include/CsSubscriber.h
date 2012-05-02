#ifndef CSSUBSCRIBER
#define CSSUBSCRIBER 1
#include "Runable.h"
#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <ros/transport_hints.h>
namespace RosCs {
	template<class M>
	class CsSubscriber : Runable {
		public:
			
			CsSubscriber(void (*cshandle)(char*),char* (*translator)(const boost::shared_ptr<M const > &)) {
				this->cscallback=cshandle;
				this->translator=translator;
				this->running = true;
			}
			void HandleMessage(const boost::shared_ptr<M const >& message) {
				char* p = translator(message);
				cscallback(p);
				delete[] p;
			}
			void Subscribe(ros::NodeHandle* orignh, char* topic, uint32_t queueSize) {
				nh = ros::NodeHandle(*orignh);
				nh.setCallbackQueue(&queue);				
				sub = nh.subscribe<M,CsSubscriber<M> >(topic, queueSize,&CsSubscriber<M>::HandleMessage,this,ros::TransportHints().unreliable().reliable().tcpNoDelay());

			}
			void Listen() {
				while(nh.ok() && this->running) {
					queue.callAvailable(ros::WallDuration(1));
				}
				sub.shutdown();
				nh.setCallbackQueue(NULL);				
			}
			/*void Shutdown() {
				running = false;				
			}*/
			void SubscribeAndListen(ros::NodeHandle* orignh, char* topic, uint32_t queueSize) {
				Subscribe(orignh,topic,queueSize);
				Listen();
			}
			/*void SubscribeAndListen(ros::NodeHandle* orignh, char* topic, uint32_t queueSize) {
				nh = ros::NodeHandle(*orignh);
				//ros::CallbackQueue queue;
				nh.setCallbackQueue(&queue);				
				//sub = nh.subscribe<M,CsSubscriber<M> >(topic, queueSize,&CsSubscriber<M>::HandleMessage,this);
				sub = nh.subscribe<M,CsSubscriber<M> >(topic, queueSize,&CsSubscriber<M>::HandleMessage,this,ros::TransportHints().udp());
				//,ros::TransportHints().unreliable());
				
				
				while(this->running) {
					queue.callAvailable(ros::WallDuration(1));//6000));
				}
				//queue.callAvailable(ros::WallDuration(30));//6000));
				
				
			}*/
			~CsSubscriber() {
				//std::cout << "destructor"<<std::endl;
				running = false;				
				sub.shutdown();	
				nh.shutdown();			
				
			}
			
		protected:
			void (*cscallback)(char*);
			char* (*translator)(const boost::shared_ptr<M const > &);
			ros::Subscriber sub;
			//bool running;
			ros::NodeHandle nh;
			ros::CallbackQueue queue;
		
					
	};
	
	
};
#endif
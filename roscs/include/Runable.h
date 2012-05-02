#ifndef CSRUNABLE_H
#define CSRUNABLE_H 1
namespace RosCs {
	class Runable {
		public:
			
			void Shutdown() {
				running = false;				
			}
			
		protected:
			bool running;	
					
	};
	
	
};
#endif

using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;


namespace RosCS
{
	public class ServiceServer
	{
		Service n;
		
		public ServiceServer(Service n, string topic) { //,Callback {
			this.n = n;
			n.ServiceServer(topic);
		}
		
		public bool AdvertiseService() { //,Callback {
			//return n.CallService(p,s);
		}		
	}
}



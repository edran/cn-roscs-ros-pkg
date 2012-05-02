
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;


namespace RosCS
{
	public class ServiceClient
	{
		Service n;
		IntPtr p;
		ulong typeId;
		
		public ServiceClient(Service n, string topic, ulong typeId) {
			this.n = n;
			this.p = n.ServiceClient(topic,typeId);
			this.typeId = typeId;
			
		}
		
		public bool Call(AbstractService s) {
			if (s.MessageType != this.typeId) {
				throw new Exception(String.Format("Wrong Service Type in Send: {0}",s.MessageType));
			}
			return n.CallService(p,s);
		}		
	}
}


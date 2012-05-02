
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;


namespace RosCS
{

	public class Publisher
	{
		Node n;
		IntPtr p;
		ulong typeId;
		
		public Publisher(Node n, string topic, ulong typeId,int queueSize) {
			this.n = n;
			this.p = n.Advertise(topic,typeId,queueSize);
			this.typeId = typeId;
			
		}
		
		public void Send(Message m) {
			if (m.MessageType != this.typeId) {
				throw new Exception(String.Format("Wrong Message Type in Send: {0}",m.MessageType));
			}
			n.Send(p,m);
		}		
	}
}

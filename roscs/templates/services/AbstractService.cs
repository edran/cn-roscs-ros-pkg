
using System;

namespace RosCS
{
	public delegate bool InternalServiceCallBack(IntPtr request, IntPtr response);
	public delegate void InternalFreeCallBack(IntPtr free);

	public abstract class AbstractService : Message
	{
		public abstract void GetResponse(IntPtr p);
		public abstract void GetRequest(IntPtr p);
		public abstract void SetResponse(IntPtr p);
	}
}


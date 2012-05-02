
using System;

namespace RosCS
{
	public delegate void InternalCallBack(IntPtr data);

	public abstract class Message
	{
		public Message() {}
		
		public abstract ulong MessageType {get;}
		
		protected static System.Text.ASCIIEncoding AsciiEncoder = new System.Text.ASCIIEncoding();
		
		public abstract int GetSize();
		public abstract void GetData(IntPtr p);
	}
}

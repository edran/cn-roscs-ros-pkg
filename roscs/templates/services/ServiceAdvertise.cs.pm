
using System;
using System.Threading;
using System.Runtime.InteropServices;
namespace RosCS {
 <?m.advertisingDelegateTypes?>
}
namespace <?m.CsNameSpace?>
{
	
	public class ServiceAdvertise<?m.CsBaseClassName?>
	{
		protected event On<?m.CsBaseClassName?> callEvent;
		
		
		
		protected string topic;
		protected IntPtr node;
			
		protected Thread t;
		
		public ServiceAdvertise<?m.CsBaseClassName?>(IntPtr node, string topic, On<?m.CsBaseClassName?> callback) {
			this.callEvent += callback;
			this.topic = topic;
			this.node = node;		
			this.t = new Thread(new ThreadStart(this.Run));
			t.Start();
			
		}
		~ServiceAdvertise<?m.CsBaseClassName?>() {
			if (this.t != null) {				
				t.Abort();
				t = null;				
			}
			this.callEvent = null;			
		}
		public void Close() {		
			
			if (this.t != null) {
				t.Abort();
				t = null;
			}
			
			this.callEvent = null;
		}
		public bool IsEmpty() {
			return this.callEvent == null;
		}
		protected void Run() {
			
			DoAdvertising(this.node, this.topic,<?m.Id?>,this.HandleMessage,this.Free);
			
		}
		
		public void Add(On<?m.CsBaseClassName?> callback) {
			this.callEvent += callback;
		}
		public void Remove(On<?m.CsBaseClassName?> callback) {
			this.callEvent -= callback;
		}
		public bool HandleMessage(IntPtr request, IntPtr response) {
			bool ret = false;
			<?m.FullCsClassName?> msg = new <?m.FullCsClassName?>(true);
			msg.GetRequest(request);
			if(this.callEvent!=null) {
				ret = this.callEvent(msg.Request, msg.Response);
				msg.SetResponse(response);
			}
			return ret;
		}
		public void Free(IntPtr pointer)
		{
			Marshal.FreeHGlobal(pointer);
		}

		[DllImport("<?cppLibName?>",EntryPoint="advertiseService")]
		internal static extern void DoAdvertising(IntPtr node, string topic,ulong messageId,InternalServiceCallBack callback, InternalFreeCallBack free);
		
		[DllImport("<?cppLibName?>",EntryPoint="stopAdvertising")]
		private static extern void StopCSAdvertising(IntPtr csAdvertiser);

	}
}

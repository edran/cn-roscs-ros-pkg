
using System;
using System.Threading;
using System.Runtime.InteropServices;
namespace RosCS {
 <?m.SubscriptionDelegateType?>
}
namespace <?m.CsNameSpace?>
{
	
	public class Subscriber<?m.CsBaseClassName?>
	{
		protected event On<?m.CsBaseClassName?> callEvent;
		
		
		
		protected string topic;
		protected IntPtr node;
			
		protected Thread t;
		protected int queueSize;
		public IntPtr rosSubscriber;
		
		public Subscriber<?m.CsBaseClassName?>(IntPtr node, string topic, On<?m.CsBaseClassName?> callback,int queueSize) {
			this.callEvent += callback;
			this.topic = topic;
			this.node = node;		
			this.t = new Thread(new ThreadStart(this.Run));
			this.queueSize=queueSize;
			this.rosSubscriber = Marshal.AllocHGlobal(8);			
			t.Start();
			
		}
		public Subscriber<?m.CsBaseClassName?>(IntPtr node, string topic, On<?m.CsBaseClassName?> callback) : this(node,topic,callback,1) {
		}
		~Subscriber<?m.CsBaseClassName?>() {
			if (this.t != null) {	
				this.Close();			
				//t.Abort();
				//t.Join();				
				t = null;				
			}
			this.callEvent = null;			
		}
		public void Close() {		
			
			if (this.t != null) {
				if (this.rosSubscriber!=IntPtr.Zero) {
					StopCSSubscriber(this.rosSubscriber);
					Marshal.FreeHGlobal(this.rosSubscriber);
					this.rosSubscriber = IntPtr.Zero;
				}
				//Console.WriteLine("Closing subscriber {0}",this.topic);
				//Console.WriteLine("Waiting for ROS Thread to finish");				
				t.Join();
				//Console.WriteLine("ROS Thread finished");
				t = null;
			}
			
			this.callEvent = null;
		}
		public bool IsEmpty() {
			return this.callEvent == null;
		}
		protected void Run() {
			
			//RosSharp.DoSubscription(this.node, this.topic,<?m.Id?>,this.HandleMessage,this.queueSize);
			DoSubscription(this.node, this.topic,<?m.Id?>,this.HandleMessage,this.queueSize,rosSubscriber);
			
		}
		
		public void Add(On<?m.CsBaseClassName?> callback) {
			this.callEvent += callback;
		}
		public void Remove(On<?m.CsBaseClassName?> callback) {
			this.callEvent -= callback;
		}
		public void HandleMessage(IntPtr data) {		
			//Console.WriteLine("Called CS HandleMessage of <?m.FullName?>");
			<?m.FullCsClassName?> msg = new <?m.FullCsClassName?>(data);
			if(this.callEvent!=null) {
				try {
					this.callEvent(msg);
				} catch(Exception e) {
					RosError("Exception in Message Handler: "+e.ToString());
				}
			}
		}
		[DllImport("<?cppLibName?>",EntryPoint="subscribe")]
		internal static extern void DoSubscription(IntPtr node, string topic,ulong messageId,InternalCallBack callback,int queueSize, IntPtr subscriptPtr);
		
		[DllImport("<?cppLibName?>",EntryPoint="stopSubscriber")]
		private static extern void StopCSSubscriber(IntPtr csSubscriber);
		
		[DllImport("<?cppLibName?>",EntryPoint="rosError")]
		private static extern void RosError(string str);

	}
}

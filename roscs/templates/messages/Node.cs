
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Mono.Unix;
using Mono.Unix.Native;

namespace RosCS
{
	/*
	<?subscriptionDelegateTypes?>
	*/

	public class Node
	{
		
		
		
		protected static Node mainNode;
		protected bool open;
		protected static object lockobj = new object();
		<?subscriberStructures?>

		public static Node MainNode {
			get {
				if (mainNode == null) {
					lock(lockobj) {
						if (mainNode == null) {
							mainNode = new Node();
						}
					}
				}
				return mainNode;
			}
		}
		protected Node() {
			this.open = true;
			this.node = GetNode("");
			
		}
		IntPtr node;
		
		public Node (string name)
		{
			#pragma warning disable 0219
			Node mainNode = MainNode;
			#pragma warning restore 0219
			this.node = GetNode(name);			
			this.open = true;
		}
		~Node() {
			if(this.open) {
			//	DestroyNode(this.node);
			}
		}
		public void Close() {
			if (this.open) {
				this.open = false;
				//Console.WriteLine("Closing Node");
				<?subscriberClosingCalls?>			
				DestroyNode(this.node);
				this.node = IntPtr.Zero;
			}
		}
		/*
		protected void RegisterSignals() {
			Stdlib.signal(Signum.SIGTERM, 	new SignalHandler (OnSignal));
			Stdlib.signal(Signum.SIGINT, 	new SignalHandler (OnSignal));
			Stdlib.signal (Signum.SIGQUIT,	new SignalHandler (OnSignal));

		}
		
		public void OnSignal(int signal) {
			Console.WriteLine("Received Signal {0}",signal);
		}*/
		public IntPtr Advertise(string topic,ulong messageType, int queueSize) {
				return AdvertiseTopic(this.node,topic, messageType, queueSize);			
		}
		<?subscriptionMethods?>
		<?unsubscriptionMethods?>
		
		public void Send(IntPtr pub,Message msg) {
			IntPtr p = IntPtr.Zero;
			try {
				int len = msg.GetSize();
				p = Marshal.AllocHGlobal(len);			
				msg.GetData(p);
				SendMessage(pub,msg.MessageType,p);
			}
			catch(Exception e) { Console.Error.WriteLine("Error while sending: "+e); }
			finally { if(p!=IntPtr.Zero) Marshal.FreeHGlobal(p); }
		}
		public bool Ok() {
			return IsNodeOk(this.node);
		}
		
		public string GetParam(string key) {
			IntPtr ptr = GetParam(this.node,key);
			string s = Marshal.PtrToStringAuto(ptr);
			FreeCharPtr(ptr);
			return s;
		}
		public string SearchParam(string key) {
			IntPtr ptr = GetParam(this.node,key);
			string s = Marshal.PtrToStringAuto(ptr);
			FreeCharPtr(ptr);
			return s;
		}
		public bool HasParam(string key) {
			return HasParam(this.node,key);
		}
		public void DeleteParam(string key) {
			DeleteParam(this.node,key);
			return;
		}
		public void SetParam(string key, int val) {
			SetParam(this.node,key,val);
		}
		public void SetParam(string key, double val) {
			SetParam(this.node,key,val);
		}
		public void SetParam(string key, bool val) {
			SetParam(this.node,key,val);
		}
		public void SetParam(string key, string val) {
			SetParam(this.node,key,val);
		}

		
		public T GetTypedParam<T>(string key) {
			if (typeof(T) == typeof(bool)) {
				return (T)(object)(GetParam(key).Equals("1"));
			}
			return (T)Convert.ChangeType(GetParam(key),typeof(T));
		}

		[DllImport("<?cppLibName?>",EntryPoint="advertiseTopic")]
		protected extern static IntPtr AdvertiseTopic(IntPtr node, string topic, ulong messageType, int queueSize);
		
		
		[DllImport("<?cppLibName?>", EntryPoint="getNode")]
		protected extern static IntPtr GetNode(string name);
		
		[DllImport("<?cppLibName?>", EntryPoint="destroyNode")]
		protected extern static void DestroyNode(IntPtr node);
		
		[DllImport("<?cppLibName?>", EntryPoint="isNodeOk")]
		protected static extern bool IsNodeOk(IntPtr node);

		
		[DllImport("<?cppLibName?>", EntryPoint="sendmsg")]
		protected extern static void SendMessage(IntPtr publisher, ulong messageId, IntPtr data);
		
		public void RosDebug(string str) { Node.RosDebugInt(str); }
		public void RosWarn(string str) { Node.RosWarnInt(str); }
		public void RosError(string str) { Node.RosErrorInt(str); }
		public void RosFatal(string str) { Node.RosFatalInt(str); }
		public void RosInfo(string str) { Node.RosInfoInt(str); }
		
		
		[DllImport("<?cppLibName?>", EntryPoint="rosDebug")]
		protected static extern void RosDebugInt(string str);
		
		[DllImport("<?cppLibName?>", EntryPoint="rosWarn")]
		protected static extern void RosWarnInt(string str);

		[DllImport("<?cppLibName?>", EntryPoint="rosError")]
		protected static extern void RosErrorInt(string str);
		
		[DllImport("<?cppLibName?>", EntryPoint="rosFatal")]
		protected static extern void RosFatalInt(string str);
		
		[DllImport("<?cppLibName?>", EntryPoint="rosInfo")]
		protected static extern void RosInfoInt(string str);
		
		[DllImport("<?cppLibName?>", EntryPoint="getParam")]		
		protected static extern IntPtr GetParam(IntPtr node, string key);
		
		
		[DllImport("<?cppLibName?>", EntryPoint="searchParam")]		
		protected static extern IntPtr SearchParam(IntPtr node, string key);
		
		[DllImport("<?cppLibName?>", EntryPoint="hasParam")]
		protected static extern bool HasParam(IntPtr node, string key);
		
		[DllImport("<?cppLibName?>", EntryPoint="deleteParam")]
		protected static extern void DeleteParam(IntPtr node, string key);
		
		[DllImport("<?cppLibName?>", EntryPoint="freeCharPtr")]		
		protected static extern void FreeCharPtr(IntPtr ptr);
		
		[DllImport("<?cppLibName?>", EntryPoint="setParamInt")]		
		protected static extern void SetParam(IntPtr node,string key, int val);
		
		[DllImport("<?cppLibName?>", EntryPoint="setParamDouble")]		
		protected static extern void SetParam(IntPtr node,string key, double val);
		
		[DllImport("<?cppLibName?>", EntryPoint="setParamBool")]		
		protected static extern void SetParam(IntPtr node,string key, bool val);
		
		[DllImport("<?cppLibName?>", EntryPoint="setParamString")]		
		protected static extern void SetParam(IntPtr node,string key, string val);

		
	}
}


using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Mono.Unix;
using Mono.Unix.Native;

namespace RosCS
{
	public class Service
	{
		protected static Service mainNode;
		protected bool open;
		protected static object lockobj = new object();
		<?advertiseStructures?>

		public static Service MainNode {
			get {
				if (mainNode == null) {
					lock(lockobj) {
						if (mainNode == null) {
							mainNode = new Service();
						}
					}
				}
				return mainNode;
			}
		}
		protected Service() {
			this.open = true;
			this.service = GetService("");
			
		}
		IntPtr service;
		
		public Service (string name)
		{
			#pragma warning disable 0219
			Service mainNode = MainNode;
			#pragma warning restore 0219
			this.service = GetService(name);			
			this.open = true;
		}
		~Service() {
			if(this.open) {
			//	DestroyService(this.service);
			}
		}
		public void Close() {
			if (this.open) {
				this.open = false;
				<?advertiseClosingCalls?>
				DestroyService(this.service);
				this.service = IntPtr.Zero;
			}
		}

		public IntPtr ServiceClient(string topic,ulong messageType) {
				return ServiceClientCpp(this.service,topic, messageType);			
		}

		<?advertiseMethods?>
		<?unadvertiseMethods?>

		public bool CallService(IntPtr srvPub, AbstractService srv) {
			int len = srv.GetSize();
			IntPtr p = Marshal.AllocHGlobal(len);
			IntPtr response = Marshal.AllocHGlobal(IntPtr.Size);
			try {
				srv.GetData(p);
				bool test = CallServiceCpp(srvPub, srv.MessageType, p , response);
				if( test )
				{
					srv.GetResponse(response);
					//free
					// double free coruption??
					//Free(response);
					unsafe {
						char* tmp = *((char**)(response.ToPointer()));
						Free(tmp);
					}
					return true;
				}
				return false;
			}
			catch(Exception e) { Console.Error.WriteLine("Error while sending: "+e);
				return false;
			}
			finally {
				Marshal.FreeHGlobal(p);
				Marshal.FreeHGlobal(response);
//Console.WriteLine("freee");
			}
		}

		public bool Ok() {
			return IsServiceOk(this.service);
		}
		
		[DllImport("<?cppLibName?>", EntryPoint="getService")]
		protected extern static IntPtr GetService(string name);
		
		[DllImport("<?cppLibName?>", EntryPoint="destroyService")]
		protected extern static void DestroyService(IntPtr node);
		
		[DllImport("<?cppLibName?>", EntryPoint="isServiceOk")]
		protected static extern bool IsServiceOk(IntPtr service);

		[DllImport("<?cppLibName?>", EntryPoint="freePointer")]
		protected static extern unsafe void Free(char* pointer);

		[DllImport("<?cppLibName?>",EntryPoint="serviceClient")]
		protected extern static IntPtr ServiceClientCpp(IntPtr service, string topic, ulong messageType);

		[DllImport("<?cppLibName?>", EntryPoint="callservice")]
		protected extern static bool CallServiceCpp(IntPtr serviceClient, ulong serviceId, IntPtr request, IntPtr response);

	}
}

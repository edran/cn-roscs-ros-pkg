
using System;
using System.Runtime.InteropServices;


namespace RosCS
{
	

	public class RosSharp
	{

		public const uint None = 0;
		public const uint NoSigIntHandler = 1 << 0;
		public const uint AnonymousName = 1 << 1;
		public const uint NoRosOut = 1 << 2;		
		
		public static void Init(string name, string[] argv) {
			Init(name,argv,None);//NoSigIntHandler);
		}
		public static void Init(string name, string[] argv,uint initOptions) {
			if (argv==null) {
				RosInit(name,0,null,initOptions);
				return;
			}
			RosInit(name,argv.Length,argv,initOptions);		    
		}
		[DllImport("<?cppLibName?>", EntryPoint="getRosNow")]
		public extern static ulong Now();
		
		[DllImport("<?cppLibName?>", EntryPoint="getRosOk")]
		public extern static bool Ok();
		
		[DllImport("<?cppLibName?>", EntryPoint="rosSleep")]
		public extern static void Sleep(int ms);
		
		[DllImport("<?cppLibName?>", EntryPoint="rosInit")]
		protected extern static void RosInit(string name,int arc,string[] argv,uint InitOptions);
		
		[DllImport("<?cppLibName?>", EntryPoint="rosShutdown")]
		public extern static void Shutdown();
		
		
		[DllImport("<?cppLibName?>", EntryPoint="checkMaster")]
		public static extern bool CheckMaster();
		
		//[DllImport("<?cppLibName?>", EntryPoint="loadCode")]
		//protected static extern void LoadLib(string lib);
		/*
		[DllImport("<?cppLibName?>",EntryPoint="subscribe")]
		internal static extern void DoSubscription(IntPtr node, string topic,ulong messageId,InternalCallBack callback,int queueSize);
		*/

		
	}
}

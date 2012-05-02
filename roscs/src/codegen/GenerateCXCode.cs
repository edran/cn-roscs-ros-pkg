using System;
using System.Text;
using System.IO;
//using C5;
//using ros;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text.RegularExpressions;
namespace CSCodeGen
{



	public class CSharpTest
	{
		static string outputPath;
		static string namespaceS;
		static string basePackageName;
		static List<Message> msgList;
		static void Main(string[] args)
		{
			
			if( args.Length < 1 )
			{
				Console.WriteLine("Usage GenerateCode.exe <packageName>");	
				return;
			}
			
			// This is the code for the base process
            Process myProcess = new Process();
            // Start a new instance of this program but specify the 'spawned' version.
            ProcessStartInfo myProcessStartInfo = new ProcessStartInfo("rospack", "spawn");
            myProcessStartInfo.UseShellExecute = false;
            myProcessStartInfo.RedirectStandardOutput = true;
            myProcess.StartInfo = myProcessStartInfo;
			myProcess.StartInfo.Arguments = "find " + args[0];
            myProcess.Start();
            StreamReader myStreamReader = myProcess.StandardOutput;
            // Read the standard output of the spawned process.
            outputPath = myStreamReader.ReadLine();
            myProcess.WaitForExit();
            myProcess.Close();

			string templateDir = TemplateDir();
			if(!Directory.Exists(templateDir)) {
				Console.Error.WriteLine("Cannot find template directory: "+templateDir);
				return;
			}
			
			if(args[0].Contains("/")) {
				int idx = args[0].LastIndexOf('/');
				basePackageName = args[0].Substring(idx+1);
			} else {
				basePackageName = args[0];
			}
			if( String.IsNullOrEmpty(outputPath))
			{
				Console.WriteLine("Cannot find package name!");
				return;
			}
			outputPath += "/msg_cs_gen";
			//set namespace to packageName
			namespaceS = args[0];
			
			Console.WriteLine("Querying Ros for messages...");
			
			
			List<Message> directMessages = MessagesOfPackage(args[0]);
			
			
			for (int i=1; i<args.Length; i++) {
				if (args[i].EndsWith("/*")) {
					directMessages.AddRange(MessagesOfPackage(args[i].Substring(0,args[i].Length-2)));
				}
				else {
					directMessages.Add(new Message(args[i]));
				}
			}
				
			
			
			Dictionary<string,Message> allMessages = new Dictionary<string, Message>();
			
			foreach(Message m in directMessages) {
				allMessages.Add(m.FullName,m);
			}
			
			
			List<Message> todo = directMessages;
			do {				
				List<string> addMessages = new List<string>();
				foreach(Message m in todo) {
					List<string> reqmessage = m.RequiredMessages();			
					addMessages.AddRange(reqmessage);				
				}
				todo.Clear();
				foreach(string nmsg in addMessages) {
					if (!allMessages.ContainsKey(nmsg)) {		
						Message nmessage = new Message(nmsg);
						allMessages.Add(nmsg,nmessage);
						todo.Add(nmessage);
					}
				}
			} while(todo.Count > 0);
			
			foreach(Message m in allMessages.Values) {
				m.SetReferences(allMessages);	
			}
			
			Console.WriteLine("Start generating...");
			Console.WriteLine("Path for generating files : " + outputPath + "!");
			
			Console.WriteLine("Total msges: {0}",allMessages.Count);
			
			//clean autogen before generating
			//Console.WriteLine(pwd);
			if( Directory.Exists(outputPath) )
			{
				Console.WriteLine("Cleaning before...");
				Directory.Delete(outputPath,true);
			}
			msgList = new List<Message>(allMessages.Values);
			CheckForHashCollisions(msgList);
			
			Directory.CreateDirectory(outputPath);
			Directory.CreateDirectory(outputPath+"/cpp");
			Directory.CreateDirectory(outputPath+"/csharp");
			
			
			CreateCppCode(outputPath+"/cpp");
			
			ProcessTemplates(templateDir,outputPath);
			CreateCSMessages(outputPath+"/csharp");
			
			
		}
		public static void CreateCSMessages(string path) {
			foreach(Message m in msgList) {
				if (!Directory.Exists(path + "/"+m.RosPackage)) {
					Directory.CreateDirectory(path + "/"+m.RosPackage);
				}
				    
				StreamWriter file = new StreamWriter(path + "/"+m.RosPackage+"/"+m.CsClassName+".cs");
				file.WriteLine("using System;");
				file.WriteLine("using System.Collections.Generic;");
				file.WriteLine("namespace "+m.CsNameSpace+" {");//m.CsNameSpace);
				file.WriteLine("public class "+m.BaseName+" : RosCS.Message {");
								
				file.WriteLine(m.GetCSFields());
				
				
				
				file.WriteLine("public override int GetSize() {");
				file.WriteLine(m.GetCSSizeBody());
				file.WriteLine("}");
	
				
				file.WriteLine("public "+m.BaseName+"(bool deepInit) {");
				string ctorBody = m.GetCSConstructorBody();
				if (ctorBody!="") {
					file.WriteLine("\tif(deepInit) {");
					file.WriteLine(ctorBody);
					file.WriteLine("}");
				}
				file.WriteLine("}");
				file.WriteLine("public "+m.BaseName+"() : this(true) {}");
				file.WriteLine("public "+m.BaseName+"(IntPtr m) {");
				file.WriteLine("\tunsafe {");				
				file.WriteLine("\t\tvoid* p = m.ToPointer();");
				Message.PointersUsed.Clear();	
				Message.Mode = Message.GEN_CS;
				file.WriteLine(m.PointerDeclaration());
				foreach(string type in m.BaseTypes()) {
					Message.PointersUsed.Add(type,type+"p");
				}
				string ptrType = "void";
				string ptrName = "p";
				file.WriteLine(CleanLastLine(m.FromByte("this.",ref ptrType,ref ptrName,0)));
				file.WriteLine("}}");
				
				file.WriteLine("public override void GetData(IntPtr p) {");
				file.WriteLine("unsafe {");
				Message.PointersUsed.Clear();				
				file.WriteLine(m.PointerDeclaration());
				foreach(string type in m.BaseTypes()) {
					Message.PointersUsed.Add(type,type+"p");
				}
				ptrType = "void";
				ptrName = "pp";
					
				file.WriteLine("void* pp = p.ToPointer();");
				file.WriteLine(CleanLastLine(m.ToByte("this.",ref ptrType,ref ptrName,0)));
				file.WriteLine("}");				
				file.WriteLine("}");
				
				
				file.WriteLine("}}");
				
				file.Close();
			}
		}
		public static void ProcessTemplates(string tmplDir,string outDir) {
			string[] cstmpl = Directory.GetFiles(tmplDir,"*.cs");
			foreach(string tmpl in cstmpl) {
				Console.WriteLine("Template: {0}",tmpl);
				int idx = tmpl.LastIndexOf('/');
				string basename = tmpl.Substring(idx+1);
				StreamReader read = new StreamReader(tmpl);
				string content = read.ReadToEnd();
				string parsedContent = ProcessTemplate(content);
				StreamWriter file = new StreamWriter(outDir + "/csharp/"+basename);
				file.WriteLine(parsedContent);				
				file.Close();
			}	
			string[] cpptmpl = Directory.GetFiles(tmplDir,"*.cpp");
			foreach(string tmpl in cpptmpl) {
				Console.WriteLine("Template: {0}",tmpl);
				int idx = tmpl.LastIndexOf('/');
				string basename = tmpl.Substring(idx+1);
				StreamReader read = new StreamReader(tmpl);
				string content = read.ReadToEnd();
				string parsedContent = ProcessTemplate(content);
				StreamWriter file = new StreamWriter(outDir + "/cpp/"+basename);
				file.WriteLine(parsedContent);				
				file.Close();
			}	
			string[] mcstmpl = Directory.GetFiles(tmplDir,"*.cs.pm");
			foreach(string tmpl in mcstmpl) {
				Console.WriteLine("Template: {0}",tmpl);
				int idx = tmpl.LastIndexOf('/');
				string basename = tmpl.Substring(idx+1);
				StreamReader read = new StreamReader(tmpl);
				string content = read.ReadToEnd();
				foreach(Message m in msgList) {
					string parsedContent = ProcessTemplate(content,m);
					string newname = basename.Substring(0,basename.Length-6);
					newname += "."+m.CsClassName+".cs";
					if (!Directory.Exists(outDir + "/csharp/"+m.RosPackage)) {
						Directory.CreateDirectory(outDir + "/csharp/"+m.RosPackage);
					}
					StreamWriter file = new StreamWriter(outDir + "/csharp/"+m.RosPackage+"/"+newname);
					file.WriteLine(parsedContent);				
					file.Close();
				}
				
				
			}	
			
			
			
		}
		public static Message curMessage;
		public static string ProcessTemplate(string t,Message m) {
			curMessage = m;
			Regex markers = new Regex(@"<\?([^\?]*)\?>");
			return markers.Replace(t,new  MatchEvaluator(MatchReplacer));

		}
		
		public static string ProcessTemplate(string t) {
		
			Regex markers = new Regex(@"<\?(.*)\?>");
			return markers.Replace(t,new  MatchEvaluator(MatchReplacer));
		}
		public static string MatchReplacer(Match ma) {
			string s = ma.ToString();
			s = s.Substring(2,s.Length-4).Trim();
			string ret = "";
			switch(s) {
			case "cppLibName":
				return "lib"+basePackageName+".messages.so";
			case "subscriptionDelegateTypes":
				foreach(Message m in msgList) {
					ret += "public delegate void On"+m.BaseName+"("+m.CsClassName+" m);\n";
				}
				return ret;
			case "subscriptionMethods":
				foreach(Message m in msgList) {
					ret += "public void Subscribe(string topic,On"+m.BaseName+" callback, int queueSize) {\n";
					ret += "\t"+m.CsNameSpace+".Subscriber"+m.BaseName+" sub=null;\n";
					ret += "\tif(this."+m.BaseName+"Subscribers.TryGetValue(topic,out sub)) {\n";
					ret += "\t\tsub.Add(callback);\n";
					ret += "\t} else {\n";
					ret += "\t\tsub = new "+m.CsNameSpace+".Subscriber"+m.BaseName+"(this.node, topic,callback,queueSize);\n";
					ret += "\t\tthis."+m.BaseName+"Subscribers.Add(topic,sub);\n";
					ret += "\t}\n";
					ret += "}\n";
					ret += "public void Subscribe(string topic,On"+m.BaseName+" callback) {\n";
					ret += "\tthis.Subscribe(topic,callback,1);\n";
					ret += "}\n";
				}
				return ret;
			case "unsubscriptionMethods":
				foreach(Message m in msgList) {
					ret += "public void UnSubscribe(string topic,On"+m.BaseName+" callback) {\n";
					ret += "\t"+m.CsNameSpace+".Subscriber"+m.BaseName+" sub=null;\n";
					ret += "\tif(this."+m.BaseName+"Subscribers.TryGetValue(topic,out sub)) {\n";
					ret += "\t\tsub.Remove(callback);\n";
					ret += "\t\tif(sub.IsEmpty()) {\n"; 
					ret += "\t\t\tthis."+m.BaseName+"Subscribers.Remove(topic);\n";
					ret += "\t\t\tsub.Close();\n";
					ret +=	"\t\t}\n";
					ret += "\t}\n";
					ret += "}\n";
				}
				return ret;
			case "subscriberClosingCalls":
				foreach(Message m in msgList) {
					ret +="\tforeach("+m.CsNameSpace+".Subscriber"+m.BaseName+" sub in this."+m.BaseName+"Subscribers.Values) {\n";
					ret += "\t\tsub.Close();\n";
					ret += "\t}\n";
					ret += "\tthis."+m.BaseName+"Subscribers.Clear();\n";
				}
				return ret;
			case "sendMethods":
				foreach(Message m in msgList) {
					ret += "public void Send(IntPtr pub,"+m.CsClassName+" msg) {\n";
					ret += "\tint len = msg.GetSize();\n";
					ret += "\tIntPtr p = Marshal.AllocHGlobal(len);\n";
					ret += "\ttry {\n";
					ret += "\t\tmsg.GetData(p);\n";
					ret += "\t\tSendMessage(pub,"+m.Id+",p);\n";
					ret += "\t}\n";
					ret += "\tcatch(Exception e) { Console.Error.WriteLine(\"Error while sending: \"+e); }\n";
					ret += "\tfinally { Marshal.FreeHGlobal(p); }\n";
					ret += "}\n";
				}
				return ret;
			case "subscriberStructures":
				foreach(Message m in msgList) {
					ret += "protected Dictionary<string,"+m.CsNameSpace+".Subscriber"+m.BaseName+"> "+ m.BaseName+"Subscribers = new Dictionary<string,"+m.CsNameSpace+".Subscriber"+m.BaseName+">();\n";					
				}
				return ret;
				
			case "m.CsNameSpace":
				return curMessage.CsNameSpace;
			case "m.CsBaseClassName":
				return curMessage.BaseName;
			case "m.FullCsClassName":
				return curMessage.CsClassName;
			case "m.Id":
				return curMessage.Id.ToString();
			case "m.FullName":
				return curMessage.FullName;		
			case "m.SubscriptionDelegateType":
				return "public delegate void On"+curMessage.BaseName+"("+curMessage.CsClassName+" m);\n";				
			case "cppSubscriptionMethods":
				ret+= "extern \"C\" void subscribe(ros::NodeHandle* node, char* topic, uint64_t messageId, void (*handle)(char*),int queueSize, void** subscriberPtr) {\n";
				ret +="\tswitch(messageId) {\n";
				foreach(Message m in msgList) {
					ret+= "\t\tcase "+m.Id+"ull:\n";
					//ret += "\t\tRosCs::CsSubscriber<"+m.RosClassName+">(handle,translate).SubscribeAndListen(node,topic,queueSize);\n";
					//ret+= "\t\tRosCs::CsSubscriber<"+m.RosClassName+"> csub = RosCs::CsSubscriber<"+m.RosClassName+">(handle,translate);\n";										
					ret+= "\t\t{ RosCs::CsSubscriber<"+m.RosClassName+"> sub"+m.Id+"(handle,translate);\n";					
					ret+= "\t\t*subscriberPtr = (void*) &sub"+m.Id+";\n";					
					ret+="\t\tsub"+m.Id+".SubscribeAndListen(node,topic,queueSize);}\n";					
					ret+= "\t\tbreak;\n";					
				
				}
				ret+="\t\tdefault: std::cerr<<\"Unknown message id: \"<< messageId << std::endl;\n";
				ret+="\t}\n";
				ret+="}\n";
				return ret;
			case "cppSendBody":
				foreach(Message m in msgList) {
					ret+= "\tcase "+m.Id+"ull: {\n";
					ret+= "\t\t"+m.RosClassName+ " m"+m.Id+" = " +m.RosClassName+"();\n";
					ret+="\t\ttranslate2Ros(data,m"+m.Id+");\n";
					ret+="\t\tpub->publish<"+m.RosClassName+">(m"+m.Id+");\n";
					ret+= "\t\t} break;\n";				
				}
				return ret;
			case "cppMessageHandleMethods":
				foreach(Message m in msgList) {
					ret+= "char* translate(const "+m.RosClassName+"::ConstPtr& m) {\n";
					//ret+= "uint32_t size = getSizeOf(m);\n";
					ret+= "uint32_t size = getSizeOf(*m);\n";
					//ret+= "std::cout << \"Ros2Cpp message size: \"<< size << std::endl;\n";
					ret+= "char* ret = (char*)malloc(size);\n";
					//ret+= "int32_t* intp = (int*) ret;\n";
					Message.PointersUsed.Clear();
					Message.Mode = Message.GEN_CPP;
					ret+=m.PointerDeclaration();
					foreach(string type in m.BaseTypes()) {
						Message.PointersUsed.Add(type,type+"p");
					}
					//Message.PointersUsed.Add("int32","intp");
					string ptrType = "";//"int32";
					string ptrName = "ret";
					ret+= CleanLastLine(m.Ros2Char("m->",ref ptrType,ref ptrName,0));
					ret+= "\treturn ret;\n";
					ret+= "}\n";
				}
				return ret;
			case "cppArray2RosMethods":
				foreach(Message m in msgList) {
					ret+= "void translate2Ros(int32_t* p, "+m.RosClassName+"& m) {\n";					
					Message.PointersUsed.Clear();
					Message.Mode = Message.GEN_CPP;
					Message.PointersUsed.Add("int32","p");
					//string decl = m.PointerDeclaration();
					
					ret+=m.PointerDeclaration("int32");
					foreach(string type in m.BaseTypes()) {
						if(!type.Equals("int32")) {
							Message.PointersUsed.Add(type,type+"p");
						}
					}
					
					string ptrType = "int32";
					string ptrName = "p";
					ret+= CleanLastLine(m.Char2Ros("m.",ref ptrType,ref ptrName,0));					
					ret+= "}\n";
				}
				return ret;
			case "cppMakePublisher":
				ret+= "\tswitch(messageId) {\n";
				foreach(Message m in msgList) {
					ret+= "\t\tcase "+m.Id+"ull:\n";
					ret+= "\t\ttmp=node->advertise<"+m.RosClassName+">(topic,queueSize,false);\n";
					ret+= "\t\tbreak;\n";
				}
				ret+= "\t\tdefault: std::cerr<<\"Unknown messageId: \"<<messageId<<std::endl;\n";
				ret+= "\t}\n";
				return ret;
			case "cppGetSizeOfMessage":
				foreach(Message m in msgList) {
					//ret+= "uint32_t getSizeOf(const "+m.RosClassName+"::ConstPtr& m);\n";
					ret+= "uint32_t getSizeOf(const "+m.RosClassName+"& m);\n";
				}				
				ret +="\n";
				foreach(Message m in msgList) {
					ret+= "uint32_t getSizeOf(const "+m.RosClassName+"& m) {\n";
					//ret+= "uint32_t getSizeOf(const "+m.RosClassName+"::ConstPtr& m) {\n";
					ret+= m.GetCPPSizeBody();					
					ret+= "}\n";
				}
				return ret;				
			case "messageIncludes":
				foreach(Message m in msgList) {
					ret+= "#include \""+m.FullName+".h\"\n";
				}
				return ret;
				
			default: Console.Error.WriteLine("Unknown Marker: {0}",s);
				System.Environment.Exit(-1);
				break;
			}
			return ret;
		}
		private static string CleanLastLine(string str) {
			string instr = str.Trim();
			if(instr.EndsWith("++;")) {
				string[] arr = instr.Split('\n');
				StringBuilder sb = new StringBuilder();
				for(int i=0; i<arr.Length-1;i++) {
					sb.AppendLine(arr[i]);
					
				}
				return sb.ToString();
			}
			else return str;
		}
		
		public static string TemplateDir() {
			Process myProcess2 = new Process();
            // Start a new instance of this program but specify the 'spawned' version.
            ProcessStartInfo myProcessStartInfo2 = new ProcessStartInfo("rospack", "spawn");
            myProcessStartInfo2.UseShellExecute = false;
            myProcessStartInfo2.RedirectStandardOutput = true;
            myProcess2.StartInfo = myProcessStartInfo2;
			myProcess2.StartInfo.Arguments = "find roscs";
            myProcess2.Start();
            StreamReader myStreamReader2 = myProcess2.StandardOutput;
            // Read the standard output of the spawned process.
            string pwd = myStreamReader2.ReadLine();
            myProcess2.WaitForExit();
            myProcess2.Close();
			return pwd+"/templates/messages";
		}
		
		
		public static void CreateCppCode(string dir) {
			//CreateCppMessageMapper(dir);
			
		}
		/*public static void CreateCppMessageMapper(string dir) {
			StreamWriter file = new StreamWriter(dir + "/messageMapper.cpp");
			file.WriteLine("#include <iostream>");
			file.WriteLine("#include \"messageMapper.h\"");
			file.WriteLine();
			
			foreach(Message m in msgList) {
				file.WriteLine("#include \"{0}.h\"",m.FullName);
				
				
			}
			file.WriteLine();
			file.WriteLine("namespace RosCs {");
			file.WriteLine("void messageMapper::getTypeSpecificPublisher(ros::NodeHandle* node,ros::Publisher& pub,char* topic,int messageId,int queueSize) {");
			file.WriteLine("\tswitch(messageId) {");
			foreach(Message m in msgList) {
				file.WriteLine("\t\tcase {0}:",m.Id);
				file.WriteLine("\t\tpub=node->advertise<{0}>(topic,queueSize,false);",m.RosClassName);
				file.WriteLine("\t\tbreak;");
			}
			file.WriteLine("\t\tdefault: std::cerr<<\"Unknown messageId: \"<<messageId<<std::endl;");
			file.WriteLine("\t};");
			file.WriteLine("};");
		
			
			file.WriteLine("};");
	

			
			file.Close();
		}*/
		
		public static List<Message> MessagesOfPackage(string package) {
			Process plist = new Process();
			ProcessStartInfo psi = new ProcessStartInfo("rosmsg", "package "+package);
			plist.EnableRaisingEvents = false;
			
			
			psi.UseShellExecute = false;
			psi.ErrorDialog = false;
			psi.RedirectStandardError = true;
			psi.RedirectStandardOutput = true;
			plist.StartInfo = psi;		
			
			plist.Start();
			
			string allMsges = plist.StandardOutput.ReadToEnd();
			plist.WaitForExit();
			plist.Close();
			
			
			string[] strarr = allMsges.Split('\n');
			
			List<Message> ret = new List<Message>();
			foreach(string s in strarr) {
				if (!String.IsNullOrEmpty(s)) ret.Add(new Message(s));
			}
			return ret;
		}
		public static void CheckForHashCollisions(List<Message> messages) {
			List<ulong> ids = new List<ulong>();
			foreach(Message m in messages) {
				if (ids.Contains(m.Id)) {
					throw new Exception(String.Format("HashCollision on message {0}",m.FullName));
				} else {
					ids.Add(m.Id);
				}
			}
		}
		
	}
}

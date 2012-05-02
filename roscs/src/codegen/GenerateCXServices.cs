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
		static string basePackageName;
		static List<Service> msgList;
		static List<Message> msgListAll;
		static void Main(string[] args)
		{
			
			if( args.Length < 1 )
			{
				Console.WriteLine("Usage GenerateService.exe <packageName>");	
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
			outputPath += "/srv_cs_gen";
			//set namespace to packageName
			
			Console.WriteLine("Querying Ros for services...");
			
			List<Service> directServices = ServicesOfPackage(args[0]);
			List<Message> directMessages = MessagesOfPackage(args[0]);
			
			for (int i=1; i<args.Length; i++) {
				if (args[i].EndsWith("/*")) {
					directServices.AddRange(ServicesOfPackage(args[i].Substring(0,args[i].Length-2)));
					directMessages.AddRange(MessagesOfPackage(args[i].Substring(0,args[i].Length-2)));
				}
				else {
					List<List<string>> services = Service.GetServiceDescription(args[i]);
					directServices.Add(new Service(args[i] + "Request", services[0]));
					directServices.Add(new Service(args[i] + "Response", services[1]));
					/*List<String> serviceFields = new List<String>();
					serviceFields.Add(args[i] + "Request");
					serviceFields.Add(args[i] + "Response");
					directServices.Add(new Service(args[i], serviceFields));
					directMessages.Add(new Message(args[i]));*/
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
			
			//add services to allMessages
			foreach(Service m in directServices) {
				if( !allMessages.ContainsKey(m.BaseName) )
					allMessages.Add(m.BaseName,m);	
			}
			
			foreach(Service m in directServices) {
				m.SetReferences(allMessages);
			}
			
			Console.WriteLine("Start generating...");
			Console.WriteLine("Path for generating files : " + outputPath + "!");
			
			Console.WriteLine("Total services: {0}",directServices.Count);
			
			//clean autogen before generating
			//Console.WriteLine(pwd);
			if( Directory.Exists(outputPath) )
			{
				Console.WriteLine("Cleaning before...");
				Directory.Delete(outputPath,true);
			}
			msgList = directServices;
			msgListAll = new List<Message>(allMessages.Values);
			CheckForHashCollisions(msgList);
			
			Directory.CreateDirectory(outputPath);
			Directory.CreateDirectory(outputPath+"/cpp");
			Directory.CreateDirectory(outputPath+"/csharp");
			
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
				file.WriteLine("using System.Runtime.InteropServices;");
				file.WriteLine("namespace "+m.CsNameSpace+" {");//m.CsNameSpace);
				file.WriteLine("public class "+m.BaseName+" : RosCS.AbstractService {");
								
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
				
				//file.WriteLine("public "+m.BaseName+"(IntPtr m) {");
				file.WriteLine("public override void GetResponse(IntPtr p) {\n");
				file.WriteLine("\tunsafe {");				
				Message.PointersUsed.Clear();	
				Message.Mode = Message.GEN_CS;
				string ptrType = "void";
				string ptrName = "*tmp";
				if( !m.FullName.EndsWith("Response") && !m.FullName.EndsWith("Request") ) {
					file.WriteLine("\t\tvoid* m = p.ToPointer();");
					file.WriteLine("char** tmp = (char**)m;");
					file.WriteLine(m.PointerDeclaration());
					foreach(string type in m.BaseTypes()) {
						Message.PointersUsed.Add(type,type+"p");
					}
					file.WriteLine(CleanLastLine(((Service)m).FromByteResponse("this.",ref ptrType,ref ptrName,0)));
				}
				file.WriteLine("}}");
				
				file.WriteLine("public override void GetRequest(IntPtr p) {\n");
				file.WriteLine("\tunsafe {");				
				Message.PointersUsed.Clear();	
				Message.Mode = Message.GEN_CS;
				ptrType = "void";
				ptrName = "*tmp";
				if( !m.FullName.EndsWith("Response") && !m.FullName.EndsWith("Request") ) {
					file.WriteLine("\t\tvoid* m = p.ToPointer();");
					file.WriteLine("char** tmp = (char**)m;");
					file.WriteLine(m.PointerDeclaration());
					foreach(string type in m.BaseTypes()) {
						Message.PointersUsed.Add(type,type+"p");
					}
					file.WriteLine(CleanLastLine(((Service)m).FromByteRequest("this.",ref ptrType,ref ptrName,0)));
				}
				file.WriteLine("}}");
				
				file.WriteLine("public override void SetResponse(IntPtr p) {\n");
				file.WriteLine("\tunsafe {");				
				Message.PointersUsed.Clear();	
				Message.Mode = Message.GEN_CS;
				ptrType = "void";
				ptrName = "*m";
				if( !m.FullName.EndsWith("Response") && !m.FullName.EndsWith("Request") ) {
					file.WriteLine("\t\tchar** m = (char**)p.ToPointer();");
					file.WriteLine("char* tmp = (char*)Marshal.AllocHGlobal(Response.GetSize());");
					file.WriteLine("*m = tmp;");
					file.WriteLine(m.PointerDeclaration());
					foreach(string type in m.BaseTypes()) {
						Message.PointersUsed.Add(type,type+"p");
					}
					file.WriteLine(CleanLastLine(((Service)m).ToByteResponse("this.",ref ptrType,ref ptrName,0)));
				}
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
					if( !m.BaseName.EndsWith("Response") && !m.BaseName.EndsWith("Request") ) {
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
				return "lib"+basePackageName+".services.so";
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
			case "advertiseStructures":
				foreach(Message m in msgList) {
					if( !m.BaseName.EndsWith("Response") && !m.BaseName.EndsWith("Request") ) {
						ret += "protected Dictionary<string,"+m.CsNameSpace+".ServiceAdvertise"+m.BaseName+"> "+ m.BaseName+"Advertisers = new Dictionary<string,"+m.CsNameSpace+".ServiceAdvertise"+m.BaseName+">();\n";					
					}
				}
				return ret;
			case "advertiseClosingCalls":
				foreach(Message m in msgList) {
					if( !m.BaseName.EndsWith("Response") && !m.BaseName.EndsWith("Request") ) {
						ret +="\tforeach("+m.CsNameSpace+".ServiceAdvertise"+m.BaseName+" sub in this."+m.BaseName+"Advertisers.Values) {\n";
						ret += "\t\tsub.Close();\n";
						ret += "\t}\n";
						ret += "\tthis."+m.BaseName+"Advertisers.Clear();\n";
					}
				}
				return ret;
			case "m.advertisingDelegateTypes":
				if( !curMessage.BaseName.EndsWith("Response") && !curMessage.BaseName.EndsWith("Request") ) {
					ret += "public delegate bool On"+curMessage.BaseName+"("+curMessage.CsClassName+"Request request, " + curMessage.CsClassName + "Response response);\n";
				}
				return ret;
			case "cppCallServiceBody":
				foreach(Service m in msgList) {
					if( !m.FullName.EndsWith("Response") && !m.FullName.EndsWith("Request") ) {
						ret+= "\tcase "+m.Id+"ull: {\n";
						ret+= "\t\t"+m.RosClassName+ " m"+m.Id+" = " +m.RosClassName+"();\n";
						ret+="\t\ttranslate2Ros(request,m"+m.Id+".request);\n";
						ret+="\t\tbool test = service->call(m"+m.Id+");\n";
						ret+="\t\tif( test ) {\n";
						ret+="\t\t\ttranslate(m" + m.Id + ".response, response);\n";
						ret+="\t\t}\n";
						ret+="\t\treturn test;\n";
						ret+= "\t\t} break;\n";
					}
				}
				return ret;
			case "cppServiceHandleMethods":
				foreach(Message m in msgList) {
					if( m.FullName.EndsWith("Response") || m.FullName.EndsWith("Request")) {
						ret+= "void translate("+m.RosClassName+"& m, char** response) {\n";
						ret+= "uint32_t size = getSizeOf(m);\n";
						//ret+= "std::cout << \"Ros2Cpp message size: \"<< size << std::endl;\n";
						//ret+="char** res;\n";
						ret+= "*response = (char*)malloc(size);\n";
						//ret+= "int32_t* intp = (int*) ret;\n";
						Message.PointersUsed.Clear();
						Message.Mode = Message.GEN_CPP;
						ret+=m.PointerDeclaration();
						foreach(string type in m.BaseTypes()) {
							Message.PointersUsed.Add(type,type+"p");
						}
						//Message.PointersUsed.Add("int32","intp");
						string ptrType = "";//"int32";
						string ptrName = "*response";
						ret+= CleanLastLine(m.Ros2Char("m.",ref ptrType,ref ptrName,0));
						//ret+= "\treturn ret;\n";
						//ret+="return res;\n";
						ret+= "}\n";
					}
				}
				return ret;
			case "cppArray2RosMethods":
				foreach(Message m in msgList) {
					if( m.FullName.EndsWith("Request") ) {
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
					if( m.FullName.EndsWith("Response") ) {
						ret+= "void translate2Ros("+m.RosClassName+"& m, char** response) {\n";					
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
						string ptrName = "*response";
						ret+= CleanLastLine(m.Char2Ros("m.",ref ptrType,ref ptrName,0));					
						ret+= "}\n";
					}
				}
				return ret;
			case "cppServiceClient":
				ret+= "\tswitch(messageId) {\n";
				foreach(Service m in msgList) {
					if( !m.FullName.EndsWith("Response") && !m.FullName.EndsWith("Request") ) {
						ret+= "\t\tcase "+m.Id+"ull:\n";
						ret+= "\t\ttmp=node->serviceClient<"+m.RosClassName+">(topic);\n";
						ret+= "\t\tbreak;\n";
					}
				}
				ret+= "\t\tdefault: std::cerr<<\"Unknown messageId: \"<<messageId<<std::endl;\n";
				ret+= "\t}\n";
				return ret;
			case "cppGetSizeOfMessage":
				foreach(Message m in msgListAll) {
					//ret+= "uint32_t getSizeOf(const "+m.RosClassName+"::ConstPtr& m);\n";
					ret+= "uint32_t getSizeOf(const "+m.RosClassName+"& m);\n";
				}				
				ret +="\n";
				foreach(Message m in msgListAll) {
					ret+= "uint32_t getSizeOf(const "+m.RosClassName+"& m) {\n";
					//ret+= "uint32_t getSizeOf(const "+m.RosClassName+"::ConstPtr& m) {\n";
					ret+= m.GetCPPSizeBody();					
					ret+= "}\n";
				}
				return ret;
			case "advertiseService":
				foreach(Service m in msgList) {
					if( !m.FullName.EndsWith("Response") && !m.FullName.EndsWith("Request") ) {
						ret+= "case "+m.Id+"ull:\n";
						ret+= "RosCs::CsAdvertiseService<" + m.RosClassName + "," + m.RosClassName + "Response," +
							m.RosClassName + "Request>(handle,translate,translate2Ros,handleFree).AdvertiseAndListen(node,topic);\n";
						ret+= "\t\tbreak;\n";
					}
				}
				return ret;
			case "messageIncludes":
				foreach(Message m in msgListAll) {
					if( !m.FullName.EndsWith("Response") && !m.FullName.EndsWith("Request") )
						ret+= "#include \""+m.FullName+".h\"\n";
				}
				return ret;
			case "advertiseMethods":
				foreach(Message m in msgList) {
					if( !m.BaseName.EndsWith("Response") && !m.BaseName.EndsWith("Request") ) {
						ret += "public void Advertise(string topic,On"+m.BaseName+" callback) {\n";
						ret += "\t"+m.CsNameSpace+".ServiceAdvertise"+m.BaseName+" sub=null;\n";
						ret += "\tif(this."+m.BaseName+"Advertisers.TryGetValue(topic,out sub)) {\n";
						ret += "\t\tsub.Add(callback);\n";
						ret += "\t} else {\n";
						ret += "\t\tsub = new "+m.CsNameSpace+".ServiceAdvertise"+m.BaseName+"(this.service, topic,callback);\n";
						ret += "\t\tthis."+m.BaseName+"Advertisers.Add(topic,sub);\n";
						ret += "\t}\n";
						ret += "}\n";
					}
				}
				return ret;
			case "unadvertiseMethods":
				foreach(Message m in msgList) {
					if( !m.BaseName.EndsWith("Response") && !m.BaseName.EndsWith("Request") ) {
						ret += "public void UnAdvertise(string topic,On"+m.BaseName+" callback) {\n";
						ret += "\t"+m.CsNameSpace+".ServiceAdvertise"+m.BaseName+" sub=null;\n";
						ret += "\tif(this."+m.BaseName+"Advertisers.TryGetValue(topic,out sub)) {\n";
						ret += "\t\tsub.Remove(callback);\n";
						ret += "\t\tif(sub.IsEmpty()) {\n"; 
						ret += "\t\t\tthis."+m.BaseName+"Advertisers.Remove(topic);\n";
						ret += "\t\t\tsub.Close();\n";
						ret +=	"\t\t}\n";
						ret += "\t}\n";
						ret += "}\n";
					}
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
			return pwd+"/templates/services";
		}
		
		
		public static void CreateCppCode(string dir) {
			//CreateCppMessageMapper(dir);
			
		}
		
		public static List<Service> ServicesOfPackage(string package) {
			Process plist = new Process();
			ProcessStartInfo psi = new ProcessStartInfo("rossrv", "package "+package);
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
			List<Service> ret = new List<Service>();
			foreach(string s in strarr) {
				if ( !String.IsNullOrEmpty(s) ) {
					List<List<string>> services = Service.GetServiceDescription(s);
					ret.Add(new Service(s + "Request", services[0]));
					ret.Add(new Service(s + "Response", services[1]));
					List<String> serviceFields = new List<String>();
					String[] splitS = s.Split('/');
					serviceFields.Add(splitS[1] + "Request request");
					serviceFields.Add(splitS[1] + "Response response");
					ret.Add(new Service(s, serviceFields));
				}
			}
			return ret;
		}
		
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
				
		public static void CheckForHashCollisions(List<Service> messages) {
			List<ulong> ids = new List<ulong>();
			foreach(Service m in messages) {
				if (ids.Contains(m.Id)) {
					throw new Exception(String.Format("HashCollision on message {0}",m.FullName));
				} else {
					ids.Add(m.Id);
				}
			}
		}
		
	}
}
